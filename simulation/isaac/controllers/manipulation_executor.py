"""로봇팔 실행기 — Isaac 안에서 `/manipulation/sim/command` 를 받아 실행하고 `/manipulation/sim/state` 를 낸다.

`place_book_server.py` 의 로봇팔 부분을 그대로 옮긴 것이다. **통신 규약과 판정 기준은 바꾸지 않았다.**
    수신 /manipulation/sim/command   발행 /manipulation/sim/state
    오류 코드 M401·402·403·404·405·406·409·410·411·412 (book_placer.ERRORS)

Isaac 실행 자체(앱 생성·USD 로드·루프)는 `run_simulation.py` 가 맡는다.
"""
import math
import os
import sys
import time
import uuid

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_msgs.msg import Bool, String

from shelving_manipulation.book_placer import (
    COMMAND_CANCEL, COMMAND_PLACE, COMMAND_ROTATE_BASE, COMMAND_SCAN, COMMAND_SWEEP, decode, encode, pick_cancel, SIM_CANCELLED, SIM_FAILED,
    SIM_IDLE, SIM_RUNNING, SIM_SUCCEEDED)

from base_move import refuse_far_move
from arm_primitives import Status
from book_scene import MoveJoint, VEL_LIMIT


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


class _RotateCancelled(Exception):
    """회전을 취소하라는 말을 **회전 도중에** 들었다. 실패와 가른다."""


class ManipulationExecutor:
    """작업 한 건을 받아 계획·실행·판정하고 상태를 발행한다 (한 번에 한 건)"""

    def __init__(self, scene, node, say, command_topic="/manipulation/sim/command",
                 state_topic="/manipulation/sim/state", gate=None, sensor_policy="gated",
                 start_home="move", render=False, gui=False):
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

        self.pub = node.create_publisher(String, state_topic, 10)
        self.detect_pub = node.create_publisher(Bool, "/perception/detect_request", 10)
        self.inbox = []
        node.create_subscription(String, command_topic, lambda m: self.inbox.append(m.data), 10)

        self.job = None
        self.ready = False
        self.announced = False
        self.step = 0
        self.render_steps = 0
        self.last_pub = 0.0
        self.q_prev = self.robot.get_joint_positions()[scene.idx_arm].copy()

        # 시작 자세
        #   move : 접은 채 홈으로 이동 (검증된 경로, 느리다)
        #   snap : 홈으로 순간이동 뒤 한 번 더 맞춘다 (빠르지만 첫 작업이 실패한 적 있다)
        #   keep : **레벨에 배치된 자세를 그대로 둔다.** 아무것도 건드리지 않는다.
        #
        # keep 이 필요한 이유: 레벨에 놓인 로봇 자세·각도가 시작하자마자 바뀌면
        # 장면이 설계와 달라진다. 홈 자세는 예전 배치(트레이가 뒤쪽)에 맞춘 값이라
        # 지금 배치에서는 팔이 뒤로 161° 돌아간 모양이 된다.
        if start_home == "keep":
            self.say("시작 자세: **레벨 그대로** (팔을 건드리지 않는다) [--start-home keep]")
        elif start_home == "snap":
            scene.snap_to_home()
            self.arm.enqueue(MoveJoint(scene.q_home, speed_scale=0.6, timeout_s=5))
        else:
            for p in scene.home_moves():
                self.arm.enqueue(p)

    # ---------------------------------------------------------------- 발행
    def publish(self, state):
        self.pub.publish(String(data=encode(dict(state, stamp=time.time(), ready=self.ready))))

    def finish(self, status, **extra):
        if self.job is not None and self.job.cmd.get("type") in (COMMAND_SCAN, COMMAND_SWEEP):
            self.scene.set_scan_tray_guard(False)
        if self.gate is not None and self.sensor_policy == "gated":
            self.gate.all(True, "— 작업 끝, 관측 대기")
        if "grip_w0" in getattr(self.job, "watch", {}):
            _w0 = self.job.watch["grip_w0"]
            _wm = self.job.watch.get("grip_w_min", _w0)
            # 운반 중에 폭이 줄었나 = 책이 빠져나갔나. **차이**라서 단위 혼동은 없다.
            # 다만 키네마틱 파지에서는 손가락이 지령에 고정돼 있어 늘 0 이 나온다 —
            # 그때는 '통과' 가 아니라 '못 가림' 이다.
            try:
                from book_scene import GRASP_KINEMATIC as _KIN
            except Exception:      # noqa: BLE001
                _KIN = False
            _verdict = "판별불가[키네마틱]" if _KIN else ("ok" if _w0 - _wm <= 0.002 else "ng")
            self.say(f"[JUDGE] width_hold={_verdict} "
                     f"({_w0*1000:.1f} → 최소 {_wm*1000:.1f} mm, 한쪽 기준)")
        if os.environ.get("SIM_DIAG_M406") == "1":
            # **작업 중에 루트를 몇 번 옮겼나** (가설 H-a). 0 이 아니면 손 안의 책이
            # 밀릴 수 있다 — 주행 실행기의 _write_root 가 센다
            self.say(f"[DIAG] root_writes_after_job="
                     f"{getattr(self.scene, 'root_writes_after_job', 0)}")
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
        scene.root_writes_after_job = 0   # 계측용 — 이 작업 동안의 루트 쓰기만 센다
        _moved = scene.refresh_base()
        if _moved > 0.01:
            self.say(f"팔 베이스가 {_moved*100:.1f}cm 움직였다 — 좌표 기준을 다시 잡았다")
        pick_w = scene.to_world(cmd["pick"]["center"])
        place_w = scene.to_world(cmd["place"]["center"])
        book, dist = scene.book_on_tray_near(pick_w)
        # 야간 시험 드라이버(night/drive_job.py) 전용: 비전 없이 책을 번호로 고른다.
        # 이 필드는 ROS 노드가 보내지 않으므로 기존 경로는 그대로다.
        if cmd["pick"].get("book_index") is not None:
            try:
                from isaacsim.core.prims import SingleXFormPrim as _X
                from book_scene import BASE_LINK as _BL, HAND_LINK as _HL
                _yaw = math.degrees(math.atan2(float(scene.Rl0[1, 0]), float(scene.Rl0[0, 0])))
                self.say(f"[드라이버] l0p {np.round(scene.l0p, 3).tolist()} yaw {_yaw:+.1f}° | {_BL} USD "
                         f"{np.round(_X(_BL).get_world_pose()[0], 3).tolist()} | 손 "
                         f"{np.round(_X(_HL).get_world_pose()[0], 3).tolist()} | 로봇(물리) "
                         f"{np.round(self.robot.get_world_pose()[0], 3).tolist()}")
                from book_scene import R as _RR
                _qj = self.robot.get_joint_positions()
                self.say(f"[드라이버] 루트 {_RR} {np.round(_X(_RR).get_world_pose()[0], 3).tolist()} "
                         f"q {np.round(_X(_RR).get_world_pose()[1], 3).tolist()} | 로봇q "
                         f"{np.round(self.robot.get_world_pose()[1], 3).tolist()} | 베이스관절 "
                         f"{[(self.robot.dof_names[i], round(float(_qj[i]), 3)) for i in scene.base_idx]} "
                         f"hold {np.round(scene.base_hold, 3).tolist()}")
            except Exception as _exc:      # noqa: BLE001
                self.say(f"[드라이버] 자세 기록 실패 {_exc}")
            for _i, _b in enumerate(sorted(scene.books)):
                self.say(f"[드라이버] {_i}: {_b.rsplit('/', 1)[-1]} 월드 {np.round(scene.center(_b), 3).tolist()} "
                         f"팔기준 {np.round(scene.to_arm(scene.center(_b)), 4).tolist()}")
            book = sorted(scene.books)[int(cmd["pick"]["book_index"])]
            dist = float(np.linalg.norm(scene.center(book) - pick_w))
            self.say(f"[드라이버] 책 번호 {cmd['pick']['book_index']} → {book} "
                     f"팔기준 {np.round(scene.to_arm(scene.center(book)), 4).tolist()}")
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
        if os.environ.get("SIM_PLAN_INPUTS", "") or os.environ.get("SIM_SAY_PLAN_INPUTS", "0") != "0":
            scene.say_plan_inputs(book, place_w, tag="실행")
        plan, code, err = scene.plan_job(book, place_w)
        if plan is None:
            return self.finish(SIM_FAILED, error_code=code, message=f"계획 실패 {err}")
        self.job.plan = plan
        self.say(f"작업 {self.job.job_id}: {book.split('/')[-1]} (칸 {cmd['pick'].get('tray_slot')}, "
                 f"{dist * 100:.1f}cm) → x {plan['place_x']:.3f} 계획 OK 최대 인접변화 {plan['worst']:.3f} rad")
        self.arm.enqueue(scene.job_sequence("job", plan))

    def start_scan(self, cmd):
        """서가를 훑는다 — **베이스는 그대로**, 팔만 접어 자세를 바꾼다.

        각 자세에서 잠깐 멈춘다. 그 사이에 비전이 찍어 빈칸을 기억한다.
        멈추는 이유는 렌더·검출이 한 프레임 안에 끝나지 않기 때문이고,
        움직이는 중에 찍으면 깊이가 흐려진다.
        """
        from arm_primitives import MoveJoint, Sequence, Wait
        from book_scene import named

        token = cmd.get("token") or uuid.uuid4().hex
        job_id = cmd.get("job_id", "scan")
        dwell = float(cmd.get("dwell_s", 1.0))
        # **스캔 중엔 차체를 고정하지 않는다.** 차체고정(hold_base_tick)은 매 스텝 아티큘레이션 루트를 같은
        # 자세로 다시 쓰는데, 스캔처럼 관절이 3 rad 씩 도는 동작과 겹치면 PhysX 가 터져 관절이 NaN 이 됐다
        # (2026-09-23 실측 3회: 진입 방식·거리와 무관하게 첫 자세에서 폭발). 파지·삽입 때만 건다.
        self.scene.job_active = False
        self.scene.root_writes_after_job = 0
        try:
            poses = self.scene.plan_scan()
        except Exception as exc:      # noqa: BLE001 - 계획 실패를 작업 실패로 돌려준다
            self.scene.job_active = False
            self.publish({"token": token, "job_id": job_id, "status": SIM_FAILED,
                          "phase": "plan", "error_code": 410,
                          "message": f"스캔 계획 실패 {type(exc).__name__}: {exc}"})
            return
        # 실제 삽입까지 가능한 아래/가운데 두 층만 스캔한다. 위 층은 카메라로
        # 볼 수 있어도 팔이 책을 넣을 수 없으므로 후보를 만들지 않는다.
        poses = [pose for pose in poses if pose[2].get("reachable", False)]
        if not poses:
            self.scene.job_active = False
            self.publish({"token": token, "job_id": job_id, "status": SIM_FAILED,
                          "phase": "plan", "error_code": 410, "message": "스캔 자세 해가 없다"})
            return

        # **홈에서 첫 스캔 자세로 곧장 간다.** stow 자세를 거쳐 접는 '안전 진입' 은 팔을 데크 쪽으로
        # 숙이게 해서, 고정된 트레이·차체를 밀다 관절이 NaN 으로 터졌다 (2026-09-23 실측: 카트가 날아감).
        # 홈(트레이 위 30 cm, 아래 보기)에서 서가 쪽 스캔 자세로의 관절 보간은 temp 브랜치 실측에서
        # 첫 자세까지 문제없이 닿았다. 관절이 3 rad 넘게 돌 수 있어 시간 제한은 25 s 로 둔다.
        steps = []
        for name, q, meta in poses:
            # 관절 6 이 0.23 rad 못 미친 채 멈추는 자세가 있다(scan_0.498_C, 한계 근처). 카메라가 13° 쯤
            # 돌아가도 depth 빈칸 검출은 되므로 스캔 이동은 0.3 rad 까지 도달로 본다
            steps.append(named(MoveJoint(q, speed_scale=0.45, timeout_s=15.0, tolerance=0.3), name))
            steps.append(named(Wait(dwell), f"{name}_hold"))
        steps.append(named(MoveJoint(self.scene.q_home, speed_scale=0.45, timeout_s=25.0, tolerance=0.05), "scan_return_home"))
        self.job = Job(dict(cmd, token=token, job_id=job_id))
        self.job.state = {"token": token, "job_id": job_id, "status": SIM_RUNNING,
                          "phase": "scan", "scan_poses": len(poses)}
        self.publish(self.job.state)
        self.say(f"스캔 시작: 자세 {len(poses)}개, 자세마다 {dwell:.1f}초 정지 "
                 f"(최대 관절 변화 {max(m['step_rad'] for _, _, m in poses):.2f} rad)")
        # **꽂을 수 있는 판과 아닌 판을 미리 알린다** — 비전이 준 빈칸을 나중에 거를 때 쓴다
        _boards = sorted({float(m["board_z"]) for _, _, m in poses})
        try:
            self.scene.shadow_boards(_boards + [float(self.scene.shelf_floor_z)])   # 로그만
        except Exception as _exc:      # noqa: BLE001 - 계측이 스캔을 막으면 안 된다
            self.say(f"[판목록] 못 견줬다: {type(_exc).__name__}: {_exc}")
        self.publish(dict(self.job.state, phase="scan_plan", shelf_box=self._shelf_box_arm(),
                          shelf_gaps=self._shelf_gaps_arm(_boards),
                          boards=[{"board_z": m["board_z"], "name": n,
                                   "reachable": m["reachable"]} for n, _, m in poses]))
        self.scene.set_scan_tray_guard(True)
        self.arm.enqueue(Sequence("scan", steps))

    def _shelf_gaps_arm(self, boards):
        """선반 판들의 **실제 빈칸**을 팔 기준으로 → `[[lo, hi, 판z_팔기준], ...]`.

        왜: 비전의 빈칸 x 가 판마다 크게 흔들린다. 깊이(y)는 서가 앞면 실측으로
        끌어왔지만 x 는 끌어올 기준이 없었다 — 이것이 그 기준이다.

        **판을 여러 개 잰다** (2026-09-24). 예전에는 `shelf_floor_z` 한 판만 쟀고,
        그래서 다른 판의 빈칸은 목록에 아예 없었다. 도윤님 목표가 **3·4번 선반에
        한 권씩**인데 한 판만 재면 두 권째가 갈 곳이 없다.

        **빈칸마다 어느 판인지를 같이 보낸다.** 안 보내면 맞춤이 층을 섞어,
        3번 선반에서 본 빈칸을 4번 선반 x 로 끌어당길 수 있다.
        """
        try:
            import sys as _s
            import os as _o
            _s.path.insert(0, _o.path.dirname(_o.path.abspath(__file__)))
            from shelf_gap import gaps as _gaps
            if isinstance(boards, (int, float)):
                boards = [float(boards)]
            ax = float(self.scene.l0p[0])
            az = float(self.scene.l0p[2])
            out = []
            for bz in boards:
                boxes = [(path, float(bb[0]) - ax, float(bb[3]) - ax)
                         for path, bb in self.scene.shelf_book_boxes(float(bz))]
                if not boxes:
                    continue
                z_arm = round(float(bz) - az, 4)
                got = [[round(g.lo, 4), round(g.hi, 4), z_arm] for g in _gaps(boxes)]
                out += got
                self.say(f"[빈칸] 판 {bz:.3f} (팔기준 z {z_arm:.3f}): 책 {len(boxes)}권 · "
                         f"빈칸 {len(got)}개 "
                         + " ".join(f"{(g[1]-g[0])*1000:.0f}mm@{(g[0]+g[1])/2:+.3f}" for g in got))
            return out or None
        except Exception as exc:      # noqa: BLE001 - 진단값이 없다고 스캔을 막지 않는다
            self.say(f"[빈칸] 실측 실패 {type(exc).__name__}: {exc}")
            return None

    def _select_shelf(self, cmd):
        """명령의 `shelf_id` 로 현재 서가를 고른다 (`SIM_SHELF_PRIMS`). 매핑이 없으면 아무것도 안 한다."""
        sid = str(cmd.get("shelf_id") or "").strip()
        if not sid:
            return
        from shelf_gap import parse_shelf_prims
        prims = parse_shelf_prims(os.environ.get("SIM_SHELF_PRIMS", ""))
        if not prims:
            return                      # 서가 하나 — 지금까지의 동작
        prim = prims.get(sid)
        if prim is None:
            self._shelf_warn = getattr(self, "_shelf_warn", set())
            if sid not in self._shelf_warn:
                self._shelf_warn.add(sid)
                self.say(f"[서가] shelf_id {sid!r} 가 SIM_SHELF_PRIMS 에 없다 {sorted(prims)} — 현재 서가 그대로")
            return
        self.scene.set_shelf(prim)

    def _shelf_box_arm(self):
        """서가 AABB 를 팔 기준으로 — [x_min, x_max, y_front, y_back, z_min, z_max]. 빈칸 관측을 서가 안으로 거를 때 쓴다."""
        try:
            bb = np.asarray(self.scene.shelf_aabb_world, float)
            cs = [self.scene.to_arm(np.array([x, y, z], float)) for x in (bb[0], bb[3]) for y in (bb[1], bb[4]) for z in (bb[2], bb[5])]
            xs, ys, zs = [float(c[0]) for c in cs], [float(c[1]) for c in cs], [float(c[2]) for c in cs]
            return [round(min(xs), 3), round(max(xs), 3), round(min(ys), 3), round(max(ys), 3), round(min(zs), 3), round(max(zs), 3)]
        except Exception:      # noqa: BLE001 - 진단값이 없다고 스캔을 막지 않는다
            return None

    def start_sweep(self, cmd):
        """서가를 **수평으로 훑는다** — 판마다 왼쪽→오른쪽 moveL, 정지점마다 비전 검출.

        자세 표 + 관절별 IK 대신 arm_kinematics 가 한 단의 경로를 통째로 계획한다: 첫 자세는 여러 시드 중
        한계 여유가 가장 큰 가지, 그다음은 직전 해를 시드로 이어가 가지가 안 바뀐다. 카메라는 늘 0.6 m
        위에 있어 트레이·차체로 내려가지 않는다 (2026-09-23 지적: 자세 표 스캔이 책과 몸체까지 닿았다).
        cmd: boards [월드 z], x_from/x_to (팔 기준), points, dwell_s
        """
        import arm_kinematics as ak
        from arm_primitives import MoveJoint, Sequence, Wait
        from book_scene import named, JointPath

        token = cmd.get("token") or uuid.uuid4().hex
        job_id = cmd.get("job_id", "sweep")
        dwell = float(cmd.get("dwell_s", 1.0))
        # **훑을 판을 밖에서 고를 수 있게 한다** (`SIM_SWEEP_BOARDS="1.042"`, 진단용).
        # 기본은 **두 판 다** 훑는다 — 그게 비전팀이 구현한 동작이다.
        #
        # 2026-09-24 SW1 실측: 위 판 5/5(여유 0.210), **아래 판 3/5**(여유 0.100).
        # 실패 지점은 `x=+0.17` 의 경유점 8/9, 그 자리 한계 여유가 **0.059 rad(3.4°)**.
        #   — `정지점 5`(멈춰서 찍는 곳)와 `경유점 9`(그 사이 보간점)는 **층위가 다르다.**
        #     둘 다 스윕의 숫자다. 내가 한때 `8/9` 를 자세 표(`plan_scan`)의 것으로
        #     잘못 읽고 "안 풀리던 쪽으로 갈아탔다" 고 적었다 — **그 정정은 취소한다.**
        # 반쪽 계획을 그대로 실행해서 404 가 났다. 이제 `sweep_retry` 가 방향을
        # 뒤집고 시드를 바꿔 가며 **다 채워지는 계획만** 받는다.
        # (조작 노드는 `boards` 를 안 실어 보낸다 — 그래서 여기 기본값이 쓰인다)
        _env_boards = os.environ.get("SIM_SWEEP_BOARDS", "").strip()
        _default = ([float(b) for b in _env_boards.replace(" ", "").split(",") if b]
                    if _env_boards else [1.042, 0.498])
        boards = [float(b) for b in cmd.get("boards", _default)]
        if _env_boards:
            self.say(f"[스윕] 판을 {boards} 로 골랐다 [SIM_SWEEP_BOARDS]")
        x_from, x_to = float(cmd.get("x_from", -0.35)), float(cmd.get("x_to", 0.35))
        points = int(cmd.get("points", 5))
        try:
            self.scene.refresh_base()
            bb = np.asarray(self.scene.shelf_aabb_world, float)
            corners = [np.array([x, y, z], float) for x in (bb[0], bb[3]) for y in (bb[1], bb[4]) for z in (bb[2], bb[5])]
            fronts = [float(self.scene.to_arm(c)[1]) for c in corners if float(self.scene.to_arm(c)[1]) > 0.10]
            if not fronts:
                raise RuntimeError("서가가 팔 +Y 에 없다")
            face = min(fronts)
            q_now = np.asarray(self.robot.get_joint_positions()[self.scene.idx_arm], float)
            # **계획은 이 자세에서 시작한다.** 이 값을 안 남기면 Isaac 없이 재현할 수
            # 없다 — 2026-09-24 에 오프라인 재현이 실제와 갈린 이유가 이것이었다.
            self.say(f"[스윕] 시작 자세 q={np.round(q_now, 4).tolist()}")
            steps, holds, q = [], 0, q_now
            for bz in boards:
                z_arm = bz - float(self.scene.l0p[2])
                plan = ak.plan_board_sweep(q, face, z_arm, x_from, x_to, points)
                st = ak.path_stats(plan.qs) if plan.qs else {}
                self.say(f"[스윕] 판 {bz:.3f} (팔기준 z {z_arm:.3f}, 앞면 {face:.3f}) 정지점 {len(plan.hold_idx)}/{points} "
                         f"여유 {plan.min_margin:.3f} rad 최대변화 {st.get('max_step_rad', 0):.2f} 길이 {st.get('length_rad', 0):.2f}"
                         + (f" — {plan.reason}" if plan.reason else ""))
                if not plan.hold_idx:
                    continue
                prev = 0
                for k, hi in enumerate(plan.hold_idx):
                    seg = plan.qs[prev:hi + 1]
                    prev = hi + 1
                    name = f"sweep_{bz:.3f}_{k}"
                    steps.append(named(JointPath(name, seg, speed=0.4), name))
                    steps.append(named(Wait(dwell), f"{name}_hold"))
                    holds += 1
                q = plan.qs[plan.hold_idx[-1]]
            if not steps:
                raise RuntimeError("스윕 경로가 하나도 안 풀렸다")
            # 홈 복귀는 천천히(0.25 rad/s) 가고, **홈에 정말 도착할 때까지** 기다린 뒤 끝낸다. JointPath 는 지령을 다
            # 보내면 끝나는데 실제 팔은 0.43 rad 뒤처져 있었고(2026-09-23 18:55 진단), 그 상태에서 스캔이 '완료' 되어
            # 책 관측(비전)이 시작돼 카메라가 바닥을 보고 있었다 → "책 좌표 없음".
            q_obs = np.asarray(getattr(self.scene, "q_observe", self.scene.q_home), float)   # 책 관측 자세 (홈과 분리)
            steps.append(named(JointPath("sweep_return_home", ak.move_j(q, q_obs, 0.05), speed=0.25),
                               "sweep_return_home"))
            steps.append(named(MoveJoint(q_obs, speed_scale=0.3, timeout_s=12.0, tolerance=0.03),
                               "sweep_home_settle"))
        except Exception as exc:      # noqa: BLE001 - 계획 실패를 작업 실패로 돌려준다
            self.publish({"token": token, "job_id": job_id, "status": SIM_FAILED, "phase": "plan",
                          "error_code": 410, "message": f"스윕 계획 실패 {type(exc).__name__}: {exc}"})
            return
        self.scene.job_active = False            # 스캔 중 차체고정은 걸지 않는다 (start_scan 참조)
        self.job = Job(dict(cmd, token=token, job_id=job_id))
        self.job.state = {"token": token, "job_id": job_id, "status": SIM_RUNNING, "phase": "scan", "scan_poses": holds}
        self.publish(self.job.state)
        self.say(f"스윕 시작: 판 {boards}, 정지점 {holds}개, 정지 {dwell:.1f}s")
        # 판 목록 그림자 — **이 분기가 실제로 도는 분기다** (기본 scan_command: scan_sweep).
        # 처음엔 포즈표 분기(위)에만 넣어 두 판 동안 0건이 찍혔다 (2026-09-25 09:08, 데스크탑 지적).
        # 계측을 넣으면 한 줄이라도 찍히는지부터 본다 — 어젯밤 SIM_CARRY_MODE 와 같은 모양의 실수.
        try:
            self.scene.shadow_boards(list(boards) + [float(self.scene.shelf_floor_z)])   # 로그만
        except Exception as _exc:      # noqa: BLE001 - 계측이 스캔을 막으면 안 된다
            self.say(f"[판목록] 못 견줬다: {type(_exc).__name__}: {_exc}")
        self.publish(dict(self.job.state, phase="scan_plan", shelf_box=self._shelf_box_arm(),
                          shelf_gaps=self._shelf_gaps_arm(boards),
                          boards=[{"board_z": b, "name": f"sweep_{b:.3f}", "reachable": True} for b in boards]))
        self.scene.set_scan_tray_guard(True)
        self.arm.enqueue(Sequence("scan", steps))

    def start_rotate_base(self, cmd):
        """차체를 **팔 베이스를 축으로** 목표 yaw(월드, 도) 로 돌린다 — 작업 전에 서가를 팔 +Y 에 두려고.

        루트 자세를 `set_world_pose` 로 바꾼다 (주행 실행기가 루트를 옮기는 것과 같은 방식). 트레이·책은
        추종(anchor)으로 따라온다. 예전 구현은 없는 속성(robot_prim_path)을 읽어 Isaac 전체가 죽었다 (2026-09-23).
        """
        from isaacsim.core.prims import SingleXFormPrim
        from book_scene import BASE_LINK, R as ROOT

        token = cmd.get("token") or uuid.uuid4().hex
        job_id = cmd.get("job_id", "rotate_base")
        yaw_deg = float(cmd.get("yaw", 90.0))
        self.job = Job(dict(cmd, token=token, job_id=job_id))
        self.job.state = {"token": token, "job_id": job_id, "status": SIM_RUNNING, "phase": "rotate_base"}
        self.publish(self.job.state)
        try:
            pv, _ = SingleXFormPrim(BASE_LINK).get_world_pose()
            pv = np.asarray(pv, float)
            rp, rq = SingleXFormPrim(ROOT).get_world_pose()
            rp, rq = np.asarray(rp, float), np.asarray(rq, float)          # Isaac: (w, x, y, z)
            cur = math.atan2(2.0 * (rq[0] * rq[3] + rq[1] * rq[2]), 1.0 - 2.0 * (rq[2] ** 2 + rq[3] ** 2))
            delta = (math.radians(yaw_deg) - cur + math.pi) % (2.0 * math.pi) - math.pi
            self.say(f"베이스 회전 시작: 현재 {math.degrees(cur):+.1f}° → 목표 {yaw_deg:+.1f}° "
                     f"(차이 {math.degrees(delta):+.1f}°, 축 = 팔 베이스 {np.round(pv[:2], 3).tolist()})")
            tp0, tq0 = SingleXFormPrim(self.scene.tray).get_world_pose()
            tp0, tq0 = np.asarray(tp0, float), np.asarray(tq0, float)
            # **트레이·책도 같이 옮긴다.** 추종(follow_tray)은 실행기 spin 에서만 돌아서, 이 안에서 world.step 만
            # 하면 로봇만 가고 트레이는 남는다 (2026-09-23 실측: 옆이동 뒤 책이 트레이 ROI 밖 → 파지 실패).
            _tr = getattr(self.scene, '_move_tray_group', None)
            def _carry_tray(p, q):
                if _tr is not None:
                    try:
                        _tr(np.asarray(p, float), np.asarray(q, float))
                    except Exception as _e:      # noqa: BLE001 - 트레이 동반 실패가 회전을 막지 않게
                        self.say(f'[회전] 트레이 동반 실패 {type(_e).__name__}: {_e}')
            if abs(delta) > 1e-3:
                # **부드럽게 돈다** (기본 3 s, SIM_TURN_S). 한 번에 90° 를 틀면 화면에서 카트가 순간이동하고
                # 트레이·책이 원심력처럼 튕긴다 — 주행 실행기의 도착 회전과 같은 방식으로 매 스텝 조금씩 쓴다.
                n = max(1, int(float(os.environ.get("SIM_TURN_S", "3.0")) * 60))
                d = rp - pv
                w, x, y, z = rq
                for i in range(1, n + 1):
                    u = i / n
                    u = u * u * (3.0 - 2.0 * u)
                    a = delta * u
                    c, sn = math.cos(a), math.sin(a)
                    new_p = pv + np.array([c * d[0] - sn * d[1], sn * d[0] + c * d[1], d[2]])
                    cw, sz = math.cos(a / 2.0), math.sin(a / 2.0)
                    new_q = np.array([cw * w - sz * z, cw * x - sz * y, cw * y + sz * x, cw * z + sz * w], float)
                    SingleXFormPrim(ROOT).set_world_pose(new_p, new_q)
                    _td = tp0 - pv
                    _carry_tray(pv + np.array([c * _td[0] - sn * _td[1], sn * _td[0] + c * _td[1], _td[2]]),
                                np.array([cw * tq0[0] - sz * tq0[3], cw * tq0[1] - sz * tq0[2],
                                          cw * tq0[2] + sz * tq0[1], cw * tq0[3] + sz * tq0[0]], float))
                    self.world.step(render=self.need_render())
                    if i % 10 == 0 and self._cancel_pending(token):
                        raise _RotateCancelled("회전 중 취소")
                for _ in range(30):
                    self.world.step(render=self.need_render())
            self.scene.refresh_base()
            # **서가까지 거리 맞추기** (`standoff`, m). 카트는 꽂기 좋게 서가에 바짝(0.36 m) 붙어 서는데,
            # 스캔 자세 표는 앞면 0.75 m 기준이라 그대로 풀면 손이 차체 뒤 트레이 속으로 들어가 관절이
            # NaN 으로 터졌다 (2026-09-23 실측, 카트가 날아감). 스캔 전엔 물러나고 꽂기 전엔 다시 붙는다.
            standoff = cmd.get("standoff")
            if standoff is not None:
                bb = np.asarray(self.scene.shelf_aabb_world, float)
                corners = [np.array([x, y, z], float) for x in (bb[0], bb[3]) for y in (bb[1], bb[4]) for z in (bb[2], bb[5])]
                fronts = [float(self.scene.to_arm(c)[1]) for c in corners if float(self.scene.to_arm(c)[1]) > 0.10]
                if not fronts:
                    raise RuntimeError("서가가 팔 +Y 에 없어 거리를 맞출 수 없다")
                front = min(fronts)
                shift = front - float(standoff)                     # +: 서가 쪽으로, -: 물러남
                fwd = self.scene.Rl0 @ np.array([0.0, 1.0, 0.0])     # 팔 +Y 의 월드 방향
                fwd[2] = 0.0
                # **서가 가운데에 선다.** 주행 경유점은 서가 왼쪽에 치우쳐 있어(팔 x 2.22 vs 서가 중심 2.58) 스캔이
                # 서가 오른쪽 절반을 못 봤다 (2026-09-23 지적). 서가 AABB 의 x 중심이 팔 x=0 에 오게 옆으로도 옮긴다.
                lateral = 0.0
                if cmd.get("lateral") is not None:
                    lateral = float(cmd["lateral"])          # 조작 노드가 정한 옆이동 (틈을 검증된 x 에 두려고)
                elif cmd.get("center", True):
                    xs = [float(self.scene.to_arm(c)[0]) for c in corners]
                    lateral = (min(xs) + max(xs)) / 2.0
                right = self.scene.Rl0 @ np.array([1.0, 0.0, 0.0])   # 팔 +X 의 월드 방향
                right[2] = 0.0
                move = fwd * shift + right * lateral
                dist = float(np.linalg.norm(move))
                self.say(f"서가 앞면 {front:.3f} m → 목표 {float(standoff):.3f} m (앞뒤 {shift:+.3f}), "
                         f"서가 중심 팔기준 x {lateral:+.3f} → 옆으로 {lateral:+.3f}: 차체를 {dist:.3f} m 옮긴다")
                # **이상치를 자른다.** 실측 43회에서 정상 최대가 0.447 m 인데, VD2 는
                # 키오스크에 선 채로 서가 자리를 맞추려다 2.676 m 를 끌고 가려 했다.
                # 상한을 올려 통과시키는 것이 아니라 6배 떨어진 값 하나를 거절한다.
                _why = refuse_far_move(dist)
                if _why is not None:
                    raise RuntimeError(_why)
                if dist > 0.005:
                    p0, q0 = SingleXFormPrim(ROOT).get_world_pose()
                    p0 = np.asarray(p0, float)
                    tp1, tq1 = SingleXFormPrim(self.scene.tray).get_world_pose()
                    tp1, tq1 = np.asarray(tp1, float), np.asarray(tq1, float)
                    n = max(1, int(dist / 0.2 * 60))                  # 0.2 m/s 로 천천히
                    for i in range(1, n + 1):
                        u = i / n
                        u = u * u * (3.0 - 2.0 * u)                  # 양 끝 속도 0 — 트레이가 튕기지 않게
                        SingleXFormPrim(ROOT).set_world_pose(p0 + move * u, np.asarray(q0, float))
                        _carry_tray(tp1 + move * u, tq1)
                        self.world.step(render=self.need_render())
                        if i % 10 == 0 and self._cancel_pending(token):
                            raise _RotateCancelled("자리 맞추기 중 취소")
                    for _ in range(20):
                        self.world.step(render=self.need_render())
                    self.scene.refresh_base()
                    fronts2 = [float(self.scene.to_arm(c)[1]) for c in corners if float(self.scene.to_arm(c)[1]) > 0.10]
                    xs2 = [float(self.scene.to_arm(c)[0]) for c in corners]
                    self.say(f"서가 앞면 실측 {min(fronts2):.3f} m (목표 {float(standoff):.3f}), 서가 x 범위 팔기준 {min(xs2):+.2f}~{max(xs2):+.2f}")
            pv2, _ = SingleXFormPrim(BASE_LINK).get_world_pose()
            _, rq2 = SingleXFormPrim(ROOT).get_world_pose()
            rq2 = np.asarray(rq2, float)
            now = math.atan2(2.0 * (rq2[0] * rq2[3] + rq2[1] * rq2[2]), 1.0 - 2.0 * (rq2[2] ** 2 + rq2[3] ** 2))
            moved = float(np.linalg.norm(np.asarray(pv2, float) - pv)) * 1000.0
            self.say(f"베이스 회전 완료: yaw {math.degrees(now):+.1f}°, 팔 베이스 이동 {moved:.1f}mm")
            self.finish(SIM_SUCCEEDED, phase="rotate_base", message=f"베이스 {yaw_deg:+.0f}° 회전 완료")
        except _RotateCancelled as exc:
            # **취소는 실패가 아니다.** 그 자리에 세우고 그렇게 보고한다 — 둘을 섞으면
            # 완주율 표에서 "사고" 와 "그만두라고 해서 그만둠" 이 한 칸에 들어간다.
            self.say(f"베이스 회전 취소: {exc} (그 자리 정지)")
            self.scene.refresh_base()
            self.finish(SIM_CANCELLED, phase="rotate_base", message=f"베이스 회전 취소: {exc}")
        except Exception as exc:      # noqa: BLE001 - 회전 실패가 시뮬 전체를 죽이면 안 된다
            self.say(f"베이스 회전 실패 {type(exc).__name__}: {exc}")
            self.finish(SIM_FAILED, phase="rotate_base", error_code=410,
                        message=f"베이스 회전 실패 {type(exc).__name__}: {exc}")

    def _yaw_of(self, path):
        """prim의 현재 yaw와 위치를 반환한다."""
        from isaacsim.core.prims import SingleXFormPrim
        p, q = SingleXFormPrim(path).get_world_pose()
        p = np.asarray(p, float)
        q = np.asarray(q, float)
        # quaternion → yaw
        siny = 2.0 * (q[3] * q[2] + q[0] * q[1])
        cosy = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        return math.atan2(siny, cosy), p

    def _cancel_pending(self, token):
        """긴 블로킹 동작 **중에** 취소를 들을 수 있게 한다.

        `handle()` 은 `spin()` 안에서 불리는데, `start_rotate_base` 는 그 안에서
        수백 스텝을 직접 돌린다. 그동안 `spin()` 이 돌아오지 않으니 `rclpy.spin_once`
        도 안 돌고 `inbox` 도 안 비워진다 — **취소 메시지는 받아지지도 않는다.**
        조작 노드가 30 초에 포기하고 취소를 보내도 시뮬은 끝까지 돈다.
        2026-09-24 VD2 가 그래서 418 초를 먹었다.

        여기서 직접 한 번 퍼 올리고, 내 토큰의 취소만 꺼낸다. 다른 명령은 그대로
        둔다 — `spin()` 이 돌아가면 제 차례에 처리된다.
        """
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.inbox[:], hit = pick_cancel(self.inbox, token)
        return hit

    def handle(self, text):
        cmd = decode(text)
        if cmd is None:
            self.say(f"명령 형식 오류: {text[:80]}")
            return
        kind = cmd.get("type")
        if kind in (COMMAND_PLACE, COMMAND_SCAN, COMMAND_SWEEP):
            self._select_shelf(cmd)
        if kind == COMMAND_PLACE:
            if self.job is not None and self.job.state["status"] == SIM_RUNNING:
                self.publish({"token": cmd.get("token"), "job_id": cmd.get("job_id", ""),
                              "status": SIM_FAILED, "phase": "plan", "error_code": 411,
                              "message": f"작업 {self.job.job_id} 실행 중"})
                return
            self.start_job(cmd)
        elif kind == COMMAND_SCAN:
            if self.job is not None and self.job.state["status"] == SIM_RUNNING:
                self.publish({"token": cmd.get("token"), "job_id": cmd.get("job_id", ""),
                              "status": SIM_FAILED, "phase": "plan", "error_code": 411,
                              "message": f"작업 {self.job.job_id} 실행 중"})
                return
            self.start_scan(cmd)
        elif kind == COMMAND_SWEEP:
            if self.job is not None and self.job.state["status"] == SIM_RUNNING:
                self.publish({"token": cmd.get("token"), "job_id": cmd.get("job_id", ""),
                              "status": SIM_FAILED, "phase": "plan", "error_code": 411,
                              "message": f"작업 {self.job.job_id} 실행 중"})
                return
            self.start_sweep(cmd)
        elif kind == COMMAND_ROTATE_BASE:
            if self.job is not None and self.job.state["status"] == SIM_RUNNING:
                self.publish({"token": cmd.get("token"), "job_id": cmd.get("job_id", ""),
                              "status": SIM_FAILED, "phase": "plan", "error_code": 411,
                              "message": f"작업 {self.job.job_id} 실행 중"})
                return
            self.start_rotate_base(cmd)
        elif kind == COMMAND_CANCEL and self.job is not None and self.job.token == cmd.get("token") \
                and self.job.state["status"] == SIM_RUNNING:
            # 안전 동작: 그 자리 정지. 책을 잡고 있을 수 있으므로 그리퍼·고정 조인트는 그대로 둔다 (HOLD_GRIP)
            self.arm.cancel()
            self.finish(SIM_CANCELLED, message="취소 — 그 자리 정지 (그리퍼 유지)")

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
        running = job is not None and job.state["status"] == SIM_RUNNING and job.plan is not None
        scanning = (job is not None and job.state["status"] == SIM_RUNNING
                    and job.cmd.get("type") in (COMMAND_SCAN, COMMAND_SWEEP))
        if scanning:
            name = arm.phase.split(":")[-1]
            if name != "idle":
                job.state["phase"] = name
            if name.endswith("_hold") and job.watch.get("scan_phase") != name:
                job.watch["scan_phase"] = name
                self.detect_pub.publish(Bool(data=True))
                self.say(f"[스캔] {name}: 비전 검출 요청")
        if running:
            name = arm.phase.split(":")[-1]
            if name != "idle":
                job.state["phase"] = name
            # 구간이 바뀔 때 한 줄 (SIM_TRACE_PHASE=1, NIGHTLY 0-1i). 녹화 프레임 ≈ step / record_every
            # (녹화 루프가 executor.spin 한 번에 한 번 센다) — '어느 구간이 솟는가'를 프레임으로 짚기 위함
            if os.environ.get("SIM_TRACE_PHASE", "0") != "0" and job.watch.get("last_phase") != name:
                job.watch["last_phase"] = name
                self.say(f"[단계] {name} 시작 step={self.step} 작업step={job.steps}")
            job.peak = max(job.peak, float(ratio.max()))
            job.spikes += int(ratio.max() > 0.8)
            # **403 이 어느 구간에서 났는지 남긴다** (SIM_TRACE_SPIKE=1, 판정은 안 바꾼다).
            # 403 은 작업 전체의 80% 초과 스텝 수를 **끝에서** 한 번 판정한다. 그래서
            # "RETREATING 에서 403" 은 마지막 구간 이름일 뿐, 튄 곳이 거기라는 뜻이 아니다.
            if float(ratio.max()) > 0.8 and os.environ.get("SIM_TRACE_SPIKE", "0") != "0":
                _sp = job.watch.setdefault("spike_by_phase", {})
                _sp[name] = _sp.get(name, 0) + 1
                if _sp[name] <= 3:
                    _j = int(np.argmax(ratio))
                    self.say(f"[403추적] step {job.steps} {name} 관절 {_j + 1} "
                             f"{float(ratio[_j]) * 100:.0f}% (|Δq| "
                             f"{float(ratio[_j] * VEL_LIMIT[_j] * world.get_physics_dt()):.4f} rad/스텝, "
                             f"q {float(q_now[_j]):+.3f})")
            book = job.plan["book"]
            # M405: 들어 올린 뒤 책이 따라 올라오지 않음 / M406: 운반 중 손 안에서 어긋남
            if name == "lift" and "z0" not in job.watch:
                job.watch["z0"] = scene.center(book)[2]
                job.watch["rel0"] = scene.book_in_hand(book)
                # **키네마틱 파지에서는 406 이 원리적으로 뜰 수 없다.**
                # follow_hand() 가 매 물리 스텝마다 책의 월드 자세를 손에서 다시 써
                # 넣으므로(set_world_pose(hp + Rh@rel, ...)), book_in_hand 는 우리가
                # 방금 넣은 값이다 — 원점 기준이든 형상중심 기준이든 정의상 안 변한다.
                # 이 조합에서 406 이 안 뜨는 것은 "파지가 튼튼하다" 가 아니라
                # "재지 않았다" 는 뜻이다. 조용히 통과시키지 않고 그렇다고 적는다.
                try:
                    from book_scene import GRASP_KINEMATIC as _KIN406
                except Exception:      # noqa: BLE001
                    _KIN406 = False
                if _KIN406 and not job.watch.get("said_kin406"):
                    job.watch["said_kin406"] = True
                    self.say("[JUDGE] 손 안 어긋남(406) **판별불가** — 키네마틱 파지는 "
                             "매 스텝 책을 손에 맞춰 다시 써 넣는다. 이 신호는 "
                             "마찰 파지(SIM_GRASP_KINEMATIC=0)에서만 의미가 있다")
                if os.environ.get("SIM_DRIFT_LOG", "0") != "0" or \
                        os.environ.get("SIM_DRIFT_METRIC", "center") in ("center", "grip"):
                    try:
                        job.watch["relc0"] = scene.book_in_hand_center(book)
                        job.watch["relg0"] = scene.book_in_hand_grip(book)
                        # **증폭 배율을 먼저 말한다.** 원점·형상중심 기준의 어긋남은
                        # 회전 × 이 지렛대다 — 값을 보기 전에 얼마나 뻥튀기되는지 알아야
                        # 한다 (2026-09-24: 0.3° × 1.7 m ≈ 9 mm 가 '미끄러짐' 으로 찍혔다).
                        _lev = scene.book_origin_lever(book)
                        job.watch["lever"] = _lev
                        self.say(f"[어긋남] 지렛대(책 원점↔형상중심) {_lev*100:.1f} cm — "
                                 f"손 안에서 1° 돌면 원점·형상중심 기준이 "
                                 f"{_lev*math.radians(1)*1000:.1f} mm 움직인 것으로 찍힌다. "
                                 f"쥔점 기준은 이 증폭이 없다 (SIM_DRIFT_METRIC=grip)")
                    except Exception as _exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                        self.say(f"[어긋남] 형상중심 기준을 못 잡았다: {type(_exc).__name__}: {_exc}")
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
                # **판정을 바꾸지 않는다.** 실물에서도 읽을 수 있는 신호(그리퍼 폭)를
                # 나란히 적어 두고, 두 판정이 몇 판에서 일치하는지 나중에 센다.
                # 책의 월드 자세는 실물에서 못 읽으므로 지금 기준은 시뮬 전용이다
                try:
                    from book_scene import GRASP_KINEMATIC as _KIN
                    _w = scene.grip_width()          # **한쪽 손가락** 기준이다 (전체 벌림의 절반)
                    _t = float(job.book_thickness) if getattr(job, "book_thickness", 0) else \
                        float(scene.dims.get(book, (0, 0, 0))[0])
                    # `grip` 단계에서 실제로 명령한 폭 — book_scene.job_sequence 와 같은 식
                    _cmd = max(0.0, _t / 2 - min(0.004, 0.15 * _t)) if _t > 0 else 0.0
                    job.watch["grip_w0"] = _w
                    job.watch["grip_w_min"] = _w
                    # 예전에는 **한쪽 폭을 책 전체 두께와** 견줘서 늘 'ng' 가 나왔다
                    # (2026-09-22: 13.6 mm vs 35.2 mm). 13.6 은 지령값 그대로다.
                    if _t <= 0:
                        self.say(f"[JUDGE] width={_w*1000:.1f}mm — 책 두께를 몰라 판단 못 함")
                    elif _KIN:
                        # 키네마틱 파지는 운반 동안 책 충돌을 끈다. 그래서 손가락은 책이
                        # 있든 없든 지령까지 닫힌다 — **이 신호에는 판별력이 없다.**
                        # 통과시키려고 문턱을 낮추는 대신, 못 가린다고 적는다.
                        self.say(f"[JUDGE] rise={job.watch['rise']*100:.1f}cm(ok) "
                                 f"width={_w*1000:.1f}mm = 지령 {_cmd*1000:.1f}mm "
                                 f"(책 반두께 {_t/2*1000:.1f}) → **판별불가** "
                                 f"[키네마틱 파지라 빈손이어도 같은 값이 나온다]")
                    else:
                        # 마찰 파지면 책이 손가락을 막아 **반두께에서 멈춘다**.
                        # 지령값까지 닫혔으면 사이에 아무것도 없었다는 뜻이다.
                        _half = _t / 2
                        _ok = ("ok" if _w >= _half - 0.002
                               else "ng(헛쥠)" if _w <= _cmd + 0.001 else "ng")
                        self.say(f"[JUDGE] rise={job.watch['rise']*100:.1f}cm(ok) "
                                 f"width={_w*1000:.1f}mm (책 반두께 {_half*1000:.1f}−2 이상이어야 "
                                 f"쥔 것, 지령 {_cmd*1000:.1f} 까지 닫혔으면 헛쥠 → {_ok})")
                except Exception as _exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                    self.say(f"[JUDGE] 폭을 못 읽었다: {type(_exc).__name__}: {_exc}")
            # 추적할 때는 lift 구간도 잰다 — 어긋남이 **거기서** 생기는지 보기 위함이다
            _trace = os.environ.get("SIM_TRACE_SLIP", "0") != "0"
            _watch_names = ("lift", "carry_rotate", "wedge") if _trace else ("carry_rotate", "wedge")
            if name in _watch_names and "rel0" in job.watch \
                    and job.state["status"] == SIM_RUNNING:
                _rel = scene.book_in_hand(book)
                dev = float(np.linalg.norm(_rel - job.watch["rel0"]))
                if "grip_w_min" in job.watch:
                    # 폭이 줄면 책이 빠지는 중이다 — 실물에서 알아챌 수 있는 유일한 신호
                    job.watch["grip_w_min"] = min(job.watch["grip_w_min"], scene.grip_width())
                # **문턱을 설정으로 뺀다** (`SIM_HAND_DRIFT_M`, 기본 0.03).
                # 이 값은 손 좌표계에서 본 책 **원점**의 이동량인데, 이 레벨의 책들은
                # 원점이 형상에서 75~177 cm 떨어져 있어(2026-09-22 실측) 손이 조금만
                # 돌아도 cm 단위로 벌어진다. 키네마틱 파지 + 충돌 끄기로 물리적으로
                # 빠질 수 없는 상태에서도 3.9 cm 가 찍혀 작업이 취소됐다.
                # 형상중심 기준 (SIM_DRIFT_LOG=1 기록 / SIM_DRIFT_METRIC=center 판정 전환, NIGHTLY 1-1·1-2)
                _dc = _ang = _dg = None
                if "relc0" in job.watch:
                    try:
                        _c, _Rr = scene.book_in_hand_center(book)
                        _c0, _R0 = job.watch["relc0"]
                        _dc = float(np.linalg.norm(_c - _c0))
                        if "relg0" in job.watch:
                            _dg = float(np.linalg.norm(
                                scene.book_in_hand_grip(book) - job.watch["relg0"]))
                            job.watch["dg_max"] = max(job.watch.get("dg_max", 0.0), _dg)
                        _cos = (float(np.trace(_R0.T @ _Rr)) - 1.0) / 2.0
                        _ang = float(np.degrees(np.arccos(min(1.0, max(-1.0, _cos)))))
                        job.watch["dc_max"] = max(job.watch.get("dc_max", 0.0), _dc)
                        job.watch["ang_max"] = max(job.watch.get("ang_max", 0.0), _ang)
                        job.watch["dev_max"] = max(job.watch.get("dev_max", 0.0), dev)
                        job.watch["n_drift"] = job.watch.get("n_drift", 0) + 1
                        if os.environ.get("SIM_DRIFT_LOG", "0") != "0" and job.watch["n_drift"] % 30 == 1:
                            self.say(f"[어긋남] step {job.steps} phase={name}  원점기준 {dev * 100:.2f} cm | "
                                     f"형상중심기준 {_dc * 100:.2f} cm | "
                                     f"쥔점기준 {'--' if _dg is None else f'{_dg * 100:.2f}'} cm | "
                                     f"손기준 회전 {_ang:.1f}°  "
                                     f"(최대 {job.watch['dev_max'] * 100:.2f} / {job.watch['dc_max'] * 100:.2f} / "
                                     f"{job.watch.get('dg_max', 0.0) * 100:.2f} cm / "
                                     f"{job.watch['ang_max']:.1f}°)")
                    except Exception as _exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                        _dc = _ang = None
                        if not job.watch.get("drift_err"):
                            job.watch["drift_err"] = True
                            self.say(f"[어긋남] 형상중심 계산 실패: {type(_exc).__name__}: {_exc}")
                # **어긋남이 어떻게 커지는지 남긴다.** 갑자기 튀면 구속이 밀린 것이고,
                # 서서히 자라면 미끄러지는 것이다 — 둘은 고치는 방법이 다르다.
                #
                # **추적은 판정과 같은 자로 잰다.** 2026-09-24 에 추적이 원점 기준(0.07 cm)
                # 이고 판정이 쥔점 기준(0.50 cm)이라 7배 차이가 났고, 그걸 "튐" 으로
                # 읽을 뻔했다. 다른 자로 잰 두 값을 같은 줄에 놓으면 안 된다.
                if _trace:
                    _tv = {"grip": _dg, "center": _dc}.get(
                        os.environ.get("SIM_DRIFT_METRIC", "center"), dev)
                    _tv = dev if _tv is None else _tv
                    job.watch.setdefault("slip", []).append(_tv)
                    if len(job.watch["slip"]) % 3 == 1:
                        self.say(f"[어긋남] step {job.steps} {name} "
                                 f"{os.environ.get('SIM_DRIFT_METRIC', 'center')} {_tv*100:.2f}cm "
                                 f"(원점 {dev*100:.2f} · 형상중심 "
                                 f"{'--' if _dc is None else f'{_dc*100:.2f}'} · 쥔점 "
                                 f"{'--' if _dg is None else f'{_dg*100:.2f}'} cm · "
                                 f"회전 {'--' if _ang is None else f'{_ang:.2f}'}°)")
                # **기본을 형상중심으로 바꿨다** (2026-09-23). 원점 기준은 재는 대상이
                # 틀렸다 — 이 레벨 책들은 원점이 형상에서 75~177 cm 떨어져 있어 손이
                # 1° 만 돌아도 cm 가 찍힌다. 그걸 막으려고 문턱을 3 cm → 25 cm 로 올려
                # 쓰고 있었는데(검증 조합의 SIM_HAND_DRIFT_M=0.25), 그건 문턱을 올려
                # 통과시키는 일이다. 틀린 자로 재면서 자를 늘릴 게 아니라 자를 바꾼다.
                # `SIM_DRIFT_METRIC=origin` 으로 옛 판정으로 되돌릴 수 있다.
                _metric = os.environ.get("SIM_DRIFT_METRIC", "center")
                if _metric == "grip" and _dg is not None:
                    # **쥔 점이 움직였는가.** 미끄러짐의 정의 그대로다 — 손가락이 닿은
                    # 자리가 책 위에서 옮겨가는 것. 지렛대가 없어 회전에 증폭되지 않는다.
                    _bad = (_dg > float(os.environ.get("SIM_DRIFT_GRIP_M", "0.005"))
                            or _ang > float(os.environ.get("SIM_DRIFT_ROT_DEG", "5")))
                    _hist = job.watch.get("slip") or []
                    _shape = ("자람" if len(_hist) >= 6 and
                              max(_hist[-3:]) > max(_hist[:3]) * 1.5 + 1e-9 else "평평/튐")
                    _msg = (f"운반 중 손 안에서 쥔점 {_dg * 100:.2f}cm · 회전 {_ang:.1f}° 어긋남 "
                            f"(형상중심 {_dc * 100:.1f}cm, 원점 {dev * 100:.1f}cm, "
                            f"쥔점 최대 {job.watch.get('dg_max', 0.0) * 100:.2f}cm, "
                            f"추적 {len(_hist)}점 {_shape}, 지렛대 회전분 "
                            f"{job.watch.get('lever', 0.0) * math.radians(_ang) * 1000:.1f}mm)")
                elif _metric == "center" and _dc is not None:
                    _bad = (_dc > float(os.environ.get("SIM_DRIFT_CENTER_M", "0.01"))
                            or _ang > float(os.environ.get("SIM_DRIFT_ROT_DEG", "5")))
                    _msg = f"운반 중 손 안에서 책 형상중심 {_dc * 100:.1f}cm · 회전 {_ang:.1f}° 어긋남 (원점기준 {dev * 100:.1f}cm)"
                else:
                    if _metric == "center" and not job.watch.get("drift_fallback"):
                        # 중심 기준을 쓰기로 했는데 못 쟀다 — 조용히 옛 자로 돌아가지 않는다
                        job.watch["drift_fallback"] = True
                        self.say("[어긋남] 형상중심을 못 재서 **원점 기준으로 되돌아간다** "
                                 "— 이 판의 406 판정은 믿지 말 것")
                    _bad = dev > float(os.environ.get("SIM_HAND_DRIFT_M", "0.03"))
                    _msg = f"운반 중 손 안에서 책 {dev * 100:.1f}cm 어긋남 [원점기준]"
                if _bad and name != "lift":
                    arm.cancel()
                    self.finish(SIM_FAILED, error_code=406, message=_msg)

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
                self.finish(SIM_SUCCEEDED, phase="scan_done",
                            message="서가 스캔 완료", placement_verified=False)
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
                if job.watch.get("spike_by_phase"):
                    self.say(f"[403추적] 구간별 80% 초과 스텝: {job.watch['spike_by_phase']}")
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
