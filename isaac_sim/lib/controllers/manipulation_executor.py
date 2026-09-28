"""로봇팔 실행기 — Isaac 안에서 `/manipulation/sim/command` 를 받아 실행하고 `/manipulation/sim/state` 를 낸다.

`place_book_server.py` 의 로봇팔 부분을 그대로 옮긴 것이다. **통신 규약과 판정 기준은 바꾸지 않았다.**
    수신 /manipulation/sim/command   발행 /manipulation/sim/state
    오류 코드 M401·402·403·404·405·406·409·410·411·412 (book_placer.ERRORS)

Isaac 실행 자체(앱 생성·USD 로드·루프)는 `run_simulation.py` 가 맡는다.
"""
import os
import sys
import time
import uuid

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_msgs.msg import String

from shelving_manipulation.book_placer import (
    COMMAND_CANCEL,
    COMMAND_PLACE,
    COMMAND_SWEEP,
    decode,
    encode,
    SIM_CANCELLED,
    SIM_FAILED,
    SIM_IDLE,
    SIM_RUNNING,
    SIM_SUCCEEDED,
)

from arm_primitives import Sequence, Status, Wait
from book_scene import (
    GRASP_KINEMATIC,
    JointPath,
    MoveJoint,
    SHELF_ROW_Z,
    VEL_LIMIT,
    named,
)

class Job:
    def __init__(self, cmd):
        self.cmd = cmd
        self.token = cmd.get("token")
        self.job_id = cmd.get("job_id", "")
        self.plan = None
        self.state = {"token": self.token, "job_id": self.job_id, "status": SIM_RUNNING, "phase": "plan"}
        self.spikes = 0
        self.peak = 0.0
        self.watch = {}
        self.started = time.time()
        self.steps = 0
        self.render_steps = 0


class ManipulationExecutor:
    """작업 한 건을 받아 계획·실행·판정하고 상태를 발행한다 (한 번에 한 건)"""

    def __init__(self, scene, node, say, command_topic="/manipulation/sim/command",
                 state_topic="/manipulation/sim/state", gate=None, sensor_policy="gated",
                 start_home="move", render=False, gui=False, tray_runtime=None):
        self.scene = scene
        self.arm = scene.arm
        self.robot = scene.robot
        self.world = scene.world
        self.node = node
        self.say = say
        self.gate = gate
        self.sensor_policy = sensor_policy
        self.render = render
        self.gui = gui
        self.command_topic = command_topic
        self.tray_runtime = tray_runtime


        self.pub = node.create_publisher(String, state_topic, 10)
        self.inbox = []
        node.create_subscription(String, command_topic, lambda m: self.inbox.append(m.data), 10)

        self.job = None
        self.ready = False
        self.announced = False
        self.step = 0
        self.render_steps = 0
        self.last_pub = 0.0
        self.q_prev = self.robot.get_joint_positions()[scene.idx_arm].copy()

        # 시작 자세: 기본은 접은 채 홈으로 이동(검증된 경로). snap 은 시작이 빠르지만 첫 작업이 실패한다
        if start_home == "snap":
            scene.snap_to_home()
            self.arm.enqueue(MoveJoint(scene.q_home, speed_scale=0.6, timeout_s=5))
        else:
            for p in scene.home_moves():
                self.arm.enqueue(p)

    # ---------------------------------------------------------------- 발행
    def publish(self, state):
        self.pub.publish(String(data=encode(dict(state, stamp=time.time(), ready=self.ready))))

    def finish(self, status, **extra):
        if self.job is not None and self.job.cmd.get('type') == COMMAND_SWEEP:
            try:
                self.scene.set_scan_tray_guard(False)
            except Exception as error:
                self.say(
                    "[스캔안전] 트레이 고정 해제 실패: "
                    f"{type(error).__name__}: {error}"
                )
        if self.gate is not None and self.sensor_policy == "gated":
            self.gate.all(True, "— 작업 끝, 관측 대기")
        self.scene.job_active = False    # 다시 주행해도 책이 따라오게 한다
        extra = dict(extra, sim_steps=self.job.steps, render_steps=self.job.render_steps)
        self.job.state = dict(self.job.state, status=status, **extra)
        self.publish(self.job.state)
        self.say(f"작업 {self.job.job_id} {status} {extra} ({time.time() - self.job.started:.1f}s)")

    # ---------------------------------------------------------------- 명령
    def start_job(self, cmd):
        """명령 → 책 선택 → 계획. 실패하면 움직이지 않고 끝낸다 (M401/402/410/411)"""
        scene = self.scene
        self.job = Job(cmd)
        self.publish(self.job.state)
        if not self.ready:
            return self.finish(SIM_FAILED, error_code=411, message="Isaac 작업 실행기 초기화 중 (시작 홈 이동)")
        if cmd.get("frame_id") != "arm_base_link":
            return self.finish(SIM_FAILED, error_code=410, message=f"frame_id {cmd.get('frame_id')}")
        # **좌표를 월드로 바꾸기 전에** 팔 베이스 자세를 다시 읽는다. 주행한 뒤라면
        # 시작할 때 읽은 값이 그만큼 낡아 있어 pick/place 가 통째로 어긋난다 (2026-09-21).
        scene.job_active = True          # follow_tray 가 책을 건드리지 않게 한다
        _moved = scene.refresh_base()
        if _moved > 0.01:
            self.say(f"팔 베이스가 {_moved*100:.1f}cm 움직였다 — 좌표 기준을 다시 잡았다")
        pick_w = scene.to_world(cmd["pick"]["center"])
        place_w = scene.to_world(cmd["place"]["center"])
        book, dist = scene.book_on_tray_near(pick_w)
        if book is None:
            # **왜 못 찾았는지 숫자로 남긴다** — "책 없음" 만으로는 좌표 문제인지
            # 책이 정말 없는지 못 가른다 (2026-09-20 M0609 에서 여기서 막혔다)
            near = sorted(((float(np.linalg.norm(scene.center(b) - pick_w)), b) for b in scene.books))[:2]
            detail = ", ".join(f"{b.rsplit('/',1)[-1]} {d*100:.1f}cm" for d, b in near)
            self.say(f"트레이 칸 판정 실패: 찾는 곳(월드) {np.round(pick_w, 4).tolist()} "
                     f"= 팔기준 {np.round(cmd['pick']['center'], 4).tolist()}")
            self.say(f"    가장 가까운 책: {detail}")
            for b in scene.books[:3]:
                c = scene.center(b)
                self.say(f"    {b.rsplit('/',1)[-1]} 월드 {np.round(c,3).tolist()} "
                         f"팔기준 {np.round(scene.to_arm(c),4).tolist()}")
            return self.finish(SIM_FAILED, error_code=411,
                               message=f"트레이 칸 {cmd['pick'].get('tray_slot')} 에 책 없음 "
                                       f"(가장 가까운 것 {near[0][0]*100:.1f}cm)")
        dimensions = cmd.get("book", {})
        plan, code, err = scene.plan_job(
            book,
            place_w,
            pick_center_world=pick_w,
            book_dimensions=(
                dimensions.get("thickness", scene.dims[book][0]),
                dimensions.get("height", scene.dims[book][1]),
                dimensions.get("width", scene.dims[book][2]),
            ),
        )
        if plan is None:
            return self.finish(
                SIM_FAILED,
                error_code=code,
                message=f"계획 실패 {err}",
            )

        if self.tray_runtime is not None:
            if not self.tray_runtime.loaded:
                return self.finish(
                    SIM_FAILED,
                    error_code=411,
                    message="트레이 적재가 완료되지 않았습니다.",
                )

            if book not in self.tray_runtime.book_paths:
                return self.finish(
                    SIM_FAILED,
                    error_code=411,
                    message=(
                        "선택된 책이 TrayRuntime 관리 대상이 아닙니다: "
                        f"{book}"
                    ),
                )

            try:
                self.tray_runtime.claim_book(book)
            except Exception as error:
                return self.finish(
                    SIM_FAILED,
                    error_code=411,
                    message=(
                        "책 제어권 인계 실패: "
                        f"{type(error).__name__}: {error}"
                    ),
                )

        self.job.plan = plan
        self.say(f"작업 {self.job.job_id}: {book.split('/')[-1]} (칸 {cmd['pick'].get('tray_slot')}, "
                 f"{dist * 100:.1f}cm) → x {plan['place_x']:.3f} 계획 OK 최대 인접변화 {plan['worst']:.3f} rad")
        self.arm.enqueue(scene.job_sequence("job", plan))

    def _shelf_gaps_arm(self, boards):
        """월드에서 계산한 실제 책 사이 빈칸을 arm_base_link 기준으로 변환합니다."""
        try:
            from shelf_gap import gaps

            if isinstance(boards, (int, float)):
                boards = [float(boards)]

            arm_base_z = float(
                self.scene.l0p[2]
            )

            shelf_bounds = np.asarray(
                self.scene.shelf_aabb_world,
                dtype=float,
            )

            # 로봇에서 가까운 책장 앞면의 월드 Y입니다.
            shelf_front_y_world = min(
                (
                    float(shelf_bounds[1]),
                    float(shelf_bounds[4]),
                ),
                key=lambda y: abs(
                    y - float(self.scene.l0p[1])
                ),
            )

            result = []

            for board_z_world in boards:
                # 빈칸 계산 자체는 월드 X에서 수행합니다.
                # 책 깊이를 arm X 폭으로 투영하면 실제 빈칸이 축소됩니다.
                world_boxes = []

                for path, bounds in self.scene.shelf_book_boxes(
                    float(board_z_world)
                ):
                    bounds = np.asarray(
                        bounds,
                        dtype=float,
                    )

                    world_boxes.append((
                        path,
                        float(bounds[0]),
                        float(bounds[3]),
                    ))

                if not world_boxes:
                    continue

                world_gaps = gaps(world_boxes)

                board_z_arm = round(
                    float(board_z_world)
                    - arm_base_z,
                    4,
                )

                board_gaps = []

                for gap in world_gaps:
                    # 빈칸 양 끝을 동일한 책장 앞면 위의 점으로 변환합니다.
                    # 이렇게 해야 로봇 yaw는 반영하면서 책 깊이 때문에
                    # 빈칸 폭이 가짜로 줄어들지 않습니다.
                    point_lo_arm = self.scene.to_arm(
                        np.array(
                            [
                                float(gap.lo),
                                shelf_front_y_world,
                                float(board_z_world),
                            ],
                            dtype=float,
                        )
                    )

                    point_hi_arm = self.scene.to_arm(
                        np.array(
                            [
                                float(gap.hi),
                                shelf_front_y_world,
                                float(board_z_world),
                            ],
                            dtype=float,
                        )
                    )

                    arm_lo, arm_hi = sorted((
                        float(point_lo_arm[0]),
                        float(point_hi_arm[0]),
                    ))

                    board_gaps.append([
                        round(arm_lo, 4),
                        round(arm_hi, 4),
                        board_z_arm,
                    ])

                result.extend(board_gaps)

                self.say(
                    f"[빈칸] 판 {board_z_world:.3f}, "
                    f"arm_z={board_z_arm:.3f}: "
                    f"책={len(world_boxes)}, "
                    f"빈칸={len(board_gaps)} "
                    + " ".join(
                        (
                            f"{(gap[1] - gap[0]) * 1000:.0f}mm"
                            f"@{(gap[0] + gap[1]) / 2:+.3f}"
                        )
                        for gap in board_gaps
                    )
                )

            return result or None

        except Exception as error:
            self.say(
                "[빈칸] 실측 실패: "
                f"{type(error).__name__}: {error}"
            )
            return None

    def start_sweep(self, cmd):
        """선반을 수평으로 훑고 각 정지점의 phase를 상태로 발행합니다."""
        import arm_kinematics as ak

        token = cmd.get("token") or uuid.uuid4().hex
        job_id = cmd.get("job_id", "sweep")
        dwell = float(cmd.get("dwell_s", 1.2))

        env_boards = os.environ.get(
            "SIM_SWEEP_BOARDS",
            "",
        ).strip()

        if env_boards:
            default_boards = [
                float(value)
                for value in env_boards.replace(" ", "").split(",")
                if value
            ]
        else:
            # 운영 장면에서 지정한 하나의 배치 단만 훑니다.
            # 과거 R&D용 [1.042, 0.498]을 모두 훑으면 배치 불가능한
            # 상단 후보가 더 높은 confidence로 선택될 수 있습니다.
            default_boards = [1.042,0.498]

        boards = [
            float(value)
            for value in cmd.get("boards", default_boards)
        ]

        x_from = float(cmd.get(
            "x_from",
            os.environ.get("SIM_SWEEP_X_FROM", "-0.35"),
        ))
        x_to = float(cmd.get(
            "x_to",
            os.environ.get("SIM_SWEEP_X_TO", '0.35'),
        ))
        points = int(cmd.get(
            "points",
            os.environ.get("SIM_SWEEP_POINTS", "5"),
        ))

        scan_joint_tolerance = float(
            self.scene.conf.get(
                "tolerance",
                {},
            ).get(
                "scan_joint_rad",
                0.12,
            )
        )

        try:
            self.scene.refresh_base()

            shelf_box = np.asarray(
                self.scene.shelf_aabb_world,
                dtype=float,
            )

            corners = [
                np.array([x, y, z], dtype=float)
                for x in (shelf_box[0], shelf_box[3])
                for y in (shelf_box[1], shelf_box[4])
                for z in (shelf_box[2], shelf_box[5])
            ]

            front_candidates = [
                float(self.scene.to_arm(corner)[1])
                for corner in corners
                if float(self.scene.to_arm(corner)[1]) > 0.10
            ]

            if not front_candidates:
                raise RuntimeError(
                    "선반이 arm_base_link +Y 방향에 없습니다."
                )

            shelf_face_y = min(front_candidates)

            self.say(
                "[스윙] 운영 영역: "
                f"shelf={os.environ.get('SIM_SHELF_PRIM', '')}, "
                f"face_y_arm={shelf_face_y:.4f}, "
                f"boards_world={boards}, "
                f"x={x_from:.3f}..{x_to:.3f}, points={points}"
            )

            q_current = np.asarray(
                self.robot.get_joint_positions()[
                    self.scene.idx_arm
                ],
                dtype=float,
            )

            self.say(
                "[스윕] 시작 관절 자세: "
                f"{np.round(q_current, 4).tolist()}"
            )

            steps = []
            hold_count = 0
            q_previous = q_current
            board_z_arm_values = []
            phase_floor_z = {}

            for board_z_world in boards:
                board_z_arm = (
                    board_z_world
                    - float(self.scene.l0p[2])
                )
                board_z_arm_values.append(float(board_z_arm))

                plan = ak.plan_board_sweep(
                    q_previous,
                    shelf_face_y,
                    board_z_arm,
                    x_from,
                    x_to,
                    points,
                )

                statistics = (
                    ak.path_stats(plan.qs)
                    if plan.qs
                    else {}
                )

                self.say(
                    f"[스윕] 선반판 z={board_z_world:.3f}, "
                    f"arm z={board_z_arm:.3f}, "
                    f"정지점={len(plan.hold_idx)}/{points}, "
                    f"관절 여유={plan.min_margin:.3f} rad, "
                    f"최대 변화="
                    f"{statistics.get('max_step_rad', 0.0):.2f} rad"
                    + (
                        f", 사유={plan.reason}"
                        if plan.reason
                        else ""
                    )
                )

                if not plan.hold_idx:
                    continue

                segment_start = 0

                for index, hold_index in enumerate(
                    plan.hold_idx
                ):
                    segment = plan.qs[
                        segment_start:hold_index + 1
                    ]
                    segment_start = hold_index + 1

                    phase_name = (
                        f"sweep_{board_z_world:.3f}_{index}"
                    )

                    phase_floor_z[
                        phase_name
                    ] = float(board_z_arm)

                    phase_floor_z[
                        f"{phase_name}_hold"
                    ] = float(board_z_arm)

                    steps.append(
                        named(
                            JointPath(
                                phase_name,
                                segment,
                                speed=0.4,
                                tolerance=scan_joint_tolerance,
                            ),
                            phase_name,
                        )
                    )

                    steps.append(
                        named(
                            Wait(dwell),
                            f"{phase_name}_hold",
                        )
                    )

                    hold_count += 1

                q_previous = plan.qs[
                    plan.hold_idx[-1]
                ]

            if not steps:
                raise RuntimeError(
                    "스윕 가능한 관절 경로가 없습니다."
                )

            q_observe = np.asarray(
                getattr(
                    self.scene,
                    "q_observe",
                    self.scene.q_home,
                ),
                dtype=float,
            )

            steps.append(
                named(
                    JointPath(
                        "sweep_return_home",
                        ak.move_j(
                            q_previous,
                            q_observe,
                            0.05,
                        ),
                        speed=0.25,
                    ),
                    "sweep_return_home",
                )
            )

            steps.append(
                named(
                    MoveJoint(
                        q_observe,
                        speed_scale=0.3,
                        timeout_s=12.0,
                        tolerance=0.01,
                    ),
                    "sweep_home_settle",
                )
            )

            # MoveJoint가 허용 오차에 들어온 직후 성공을 내면 카메라에는
            # 이동 중 프레임이 남아 있을 수 있다. 관절을 목표에 둔 채 새 RGB/depth
            # 프레임이 충분히 발행될 때까지 기다린 뒤 스캔 완료를 알린다.
            steps.append(
                named(
                    Wait(max(1.0, dwell)),
                    "sweep_sensor_settle",
                )
            )

            self.scene.set_scan_tray_guard(True)

        except Exception as error:
            try:
                self.scene.set_scan_tray_guard(False)
            except Exception:
                pass

            self.publish({
                "token": token,
                "job_id": job_id,
                "status": SIM_FAILED,
                "phase": "plan",
                "error_code": 410,
                "message": (
                    "스윕 계획 실패 "
                    f"{type(error).__name__}: {error}"
                ),
            })
            return

        # 팔 스캔 중에는 차체 루트를 강제로 고정하지 않습니다.
        self.scene.job_active = False

        self.job = Job(
            dict(
                cmd,
                token=token,
                job_id=job_id,
            )
        )

        self.job.state = {
            "token": token,
            "job_id": job_id,
            "status": SIM_RUNNING,
            "phase": "scan",
            "scan_poses": hold_count,
            "shelf_plane_y": float(shelf_face_y),
            "shelf_floor_z": float(board_z_arm_values[0]),
        }

        self.job.watch[
            "phase_floor_z"
        ] = phase_floor_z

        self.publish(self.job.state)

        self.publish({
            **self.job.state,
            "phase": "scan_plan",
            "shelf_gaps": self._shelf_gaps_arm(
                boards
            ),
            "boards": [
                {
                    "board_z": float(board),
                    "name": (
                        f"sweep_{board:.3f}"
                    ),
                    "reachable": True,
                }
                for board in boards
            ],
        })

        self.say(
            f"스윕 시작: 선반판={boards}, "
            f"정지점={hold_count}, "
            f"정지시간={dwell:.1f}s"
        )

        self.arm.enqueue(
            Sequence("scan", steps)
        )

    def handle(self, text):
        cmd = decode(text)
        if cmd is None:
            self.say(f"명령 형식 오류: {text[:80]}")
            return

        kind = cmd.get("type")

        if kind in (COMMAND_PLACE, COMMAND_SWEEP):
            if (
                self.job is not None
                and self.job.state["status"] == SIM_RUNNING
            ):
                self.publish({
                    "token": cmd.get("token"),
                    "job_id": cmd.get("job_id", ""),
                    "status": SIM_FAILED,
                    "phase": "plan",
                    "error_code": 411,
                    "message": (
                        f"작업 {self.job.job_id} 실행 중"
                    ),
                })
                return

        if kind == COMMAND_PLACE:
            self.start_job(cmd)

        elif kind == COMMAND_SWEEP:
            self.start_sweep(cmd)

        elif (
            kind == COMMAND_CANCEL
            and self.job is not None
            and self.job.token == cmd.get("token")
            and self.job.state["status"] == SIM_RUNNING
        ):
            self.arm.cancel()
            self.finish(
                SIM_CANCELLED,
                message=(
                    "취소 — 그 자리 정지 "
                    "(그리퍼 유지)"
                ),
            )

    # ---------------------------------------------------------------- 루프
    def need_render(self):
        if self.gate is None:
            return self.render
        if self.sensor_policy == "always":
            return self.render or self.gate.lidar_on or self.gate.camera_on
        return self.gate.need_render(self.step, gui=self.gui)

    def spin(self):
        """한 시뮬 스텝. run_simulation.py 의 루프가 매 스텝 부른다"""
        import rclpy

        scene, arm, world = self.scene, self.arm, self.world
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while self.inbox:
            self.handle(self.inbox.pop(0))

        status = arm.update()
        r_now = self.need_render()
        world.step(render=r_now)
        self.render_steps += int(r_now)
        if self.job is not None:
            self.job.steps += 1
            self.job.render_steps += int(r_now)
        self.step += 1

        q_now = self.robot.get_joint_positions()[scene.idx_arm]
        ratio = np.abs(q_now - self.q_prev) / world.get_physics_dt() / VEL_LIMIT
        self.q_prev = q_now.copy()

        job = self.job

        running = (
            job is not None
            and job.state["status"] == SIM_RUNNING
            and job.plan is not None
        )

        scanning = (
            job is not None
            and job.state["status"] == SIM_RUNNING
            and job.cmd.get("type") == COMMAND_SWEEP
        )

        if scanning:
            scan_phase = arm.phase.split(":")[-1]

            if scan_phase != "idle":
                job.state["phase"] = scan_phase

                phase_floor_z = job.watch.get('phase_floor_z', {})

                if scan_phase in phase_floor_z:
                    job.state['shelf_floor_z'] = float(phase_floor_z[scan_phase])

            if (
                scan_phase.endswith("_hold")
                and job.watch.get("scan_phase")
                != scan_phase
            ):
                job.watch["scan_phase"] = scan_phase
                self.say(
                    f"[스캔] {scan_phase}: "
                    "perception 검출 가능한 정지 구간"
                )

        if running:
            name = arm.phase.split(":")[-1]
            if name != "idle":
                job.state["phase"] = name
            job.peak = max(job.peak, float(ratio.max()))
            job.spikes += int(ratio.max() > 0.8)
            book = job.plan["book"]
            # M405: 들어 올린 뒤 책이 따라 올라오지 않음 / M406: 운반 중 손 안에서 어긋남
            if name == "lift" and "z0" not in job.watch:
                job.watch["z0"] = scene.center(book)[2]
                job.watch["rel0"] = scene.book_in_hand(book)
            if name == "carry_rotate" and "z0" in job.watch and "rise" not in job.watch:
                job.watch["rise"] = scene.center(book)[2] - job.watch["z0"]
                # 기대 상승량은 **실제 들어올림 높이**에 맞춰야 한다. 0.08 이 박혀 있어서
                # carry_lift_m 을 0.06 으로 낮춘 M0609 는 원리상 통과할 수 없었다 (2026-09-20).
                _need = float(scene.conf["grasp"].get("lift_check_m",
                              max(0.03, scene.conf["grasp"].get("carry_lift_m", 0.17) * 0.5)))
                if job.watch["rise"] < _need:
                    arm.cancel()
                    self.finish(SIM_FAILED, error_code=405,
                                message=f"들어 올린 뒤 책 상승 {job.watch['rise'] * 100:.1f}cm "
                                        f"(기대 {_need * 100:.1f}cm 이상)")
                elif self.gate is not None and self.sensor_policy == "gated":
                    self.gate.all(False, f"— 파지 확인 (책 상승 {job.watch['rise'] * 100:.1f}cm), 작업 끝까지")
            # 추적할 때는 lift 구간도 잰다 — 어긋남이 **거기서** 생기는지 보기 위함이다
            _trace = os.environ.get("SIM_TRACE_SLIP", "0") != "0"
            _watch_names = ("lift", "carry_rotate", "wedge") if _trace else ("carry_rotate", "wedge")
            if not GRASP_KINEMATIC \
                    and name in _watch_names \
                    and "rel0" in job.watch \
                    and job.state["status"] == SIM_RUNNING:
                _rel = scene.book_in_hand(book)
                dev = float(np.linalg.norm(_rel - job.watch["rel0"]))
                # **어긋남이 어떻게 커지는지 남긴다.** 갑자기 튀면 구속이 밀린 것이고,
                # 서서히 자라면 미끄러지는 것이다 — 둘은 고치는 방법이 다르다.
                # 값만 보고는 못 가른다 (2026-09-21: 책 원점을 고쳐도 4.7cm 로 똑같았다).
                if _trace:
                    job.watch.setdefault("slip", [])
                    job.watch["slip"].append(dev)
                    if len(job.watch["slip"]) % 3 == 1:
                        self.say(f"[어긋남] step {job.steps} {name} {dev*100:.2f}cm "
                                 f"손기준차 {[round(float(v)*100, 1) for v in (_rel - job.watch['rel0'])]}")
                if dev > 0.03 and name != "lift":
                    arm.cancel()
                    self.finish(SIM_FAILED, error_code=406, message=f"운반 중 손 안에서 책 {dev * 100:.1f}cm 어긋남")

        # 위 감시에서 이미 끝냈으면 이번 스텝에 다시 판정하지 않는다
        running = running and job.state["status"] == SIM_RUNNING
        if status is Status.FAILED:
            code, err = arm.error_code or 404, arm.error
            arm.cancel()
            if not self.ready:
                self.say(f"시작 홈 이동 실패 M{code} {err} — 명령을 받지 않는다")
            elif running or scanning:
                self.finish(SIM_FAILED, error_code=code, message=err)
        elif arm.idle:
            if not self.announced:
                self.announced = True
                self.ready = True
                self.say(f"준비 완료 (step {self.step}) — 명령 대기 {self.command_topic}")
            elif scanning:
                self.finish(SIM_SUCCEEDED, phase='scan_done', message='서가 스캔 완료', placement_verified=False)
            elif running:
                for _ in range(60):
                    world.step(render=self.need_render())
                ok, checks, bb = scene.verify(job.plan)
                # **꽂은 책만 보면 놓친다** — 장면 전체를 훑어 쓰러진 책을 찾는다.
                # 이게 없어서 "4권 4/4" 라고 보고한 녹화에 누운 책이 있었다 (2026-09-20).
                states, fallen = scene.survey()
                if fallen:
                    self.say(f"**자세가 이상한 책 {len(fallen)}권** {[f['book'] for f in fallen]}")
                    for f in fallen:
                        self.say(f"    {f['book']} {f['위치']} 밑면z {f['밑면z']} 기대수직 {f['기대수직']} 크기 {f['크기']}")
                extra = {"placement_verified": bool(ok), "checks": {k: bool(v) for k, v in checks.items()},
                         "fallen_books": [f["book"] for f in fallen], "scene_books": states,
                         "book_aabb_center_arm": np.round(scene.to_arm((bb[:3] + bb[3:]) / 2), 4).tolist(),
                         "joint_peak_ratio": round(job.peak, 3), "joint_over80_steps": job.spikes}
                if job.spikes:
                    self.finish(SIM_FAILED, error_code=403,
                                message=f"관절 각속도 80% 초과 {job.spikes}스텝 (최대 {job.peak * 100:.0f}%)",
                                **extra)
                else:
                    self.finish(SIM_SUCCEEDED, phase="verify",
                                message=("배치 확인" if ok else f"배치 확인 실패 {checks}"), **extra)

        now = time.time()
        if now - self.last_pub >= 0.1:
            self.publish(job.state if job is not None else {"token": None, "status": SIM_IDLE})
            self.last_pub = now
