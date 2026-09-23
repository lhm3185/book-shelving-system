"""로봇팔 실행기 — Isaac 안에서 `/manipulation/sim/command` 를 받아 실행하고 `/manipulation/sim/state` 를 낸다.

`place_book_server.py` 의 로봇팔 부분을 그대로 옮긴 것이다. **통신 규약과 판정 기준은 바꾸지 않았다.**
    수신 /manipulation/sim/command   발행 /manipulation/sim/state
    오류 코드 M401·402·403·404·405·406·409·410·411·412 (book_placer.ERRORS)

Isaac 실행 자체(앱 생성·USD 로드·루프)는 `run_simulation.py` 가 맡는다.
"""
import os
import sys
import math
import time
import uuid

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_msgs.msg import Bool, String

from shelving_manipulation.book_placer import (
    COMMAND_CANCEL, COMMAND_PLACE, COMMAND_ROTATE_BASE, COMMAND_SCAN, COMMAND_SWEEP, decode, encode, SIM_CANCELLED, SIM_FAILED,
    SIM_IDLE, SIM_RUNNING, SIM_SUCCEEDED)

from arm_primitives import Status
from arm_planning import tucked_joint_moves
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
            self.say(f"[JUDGE] width_hold={'ok' if _w0 - _wm <= 0.002 else 'ng'} "
                     f"({_w0*1000:.1f} → 최소 {_wm*1000:.1f} mm)")
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
        self.publish(dict(self.job.state, phase="scan_plan", shelf_box=self._shelf_box_arm(),
                          boards=[{"board_z": m["board_z"], "name": n,
                                   "reachable": m["reachable"]} for n, _, m in poses]))
        self.scene.set_scan_tray_guard(True)
        self.arm.enqueue(Sequence("scan", steps))

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
        boards = [float(b) for b in cmd.get("boards", [1.042, 0.498])]
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
        self.publish(dict(self.job.state, phase="scan_plan", shelf_box=self._shelf_box_arm(),
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

    def handle(self, text):
        cmd = decode(text)
        if cmd is None:
            self.say(f"명령 형식 오류: {text[:80]}")
            return
        kind = cmd.get("type")
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
                # **판정을 바꾸지 않는다.** 실물에서도 읽을 수 있는 신호(그리퍼 폭)를
                # 나란히 적어 두고, 두 판정이 몇 판에서 일치하는지 나중에 센다.
                # 책의 월드 자세는 실물에서 못 읽으므로 지금 기준은 시뮬 전용이다
                try:
                    _w = scene.grip_width()
                    _t = float(job.book_thickness) if getattr(job, "book_thickness", 0) else \
                        float(scene.dims.get(book, (0, 0, 0))[0])
                    _ok = "ok" if _t > 0 and abs(_w - _t) <= 0.003 else "ng"
                    job.watch["grip_w0"] = _w
                    job.watch["grip_w_min"] = _w
                    self.say(f"[JUDGE] rise={job.watch['rise']*100:.1f}cm(ok) "
                             f"width={_w*1000:.1f}mm (book {_t*1000:.1f}±3 → {_ok})")
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
                # **어긋남이 어떻게 커지는지 남긴다.** 갑자기 튀면 구속이 밀린 것이고,
                # 서서히 자라면 미끄러지는 것이다 — 둘은 고치는 방법이 다르다.
                # 값만 보고는 못 가른다 (2026-09-21: 책 원점을 고쳐도 4.7cm 로 똑같았다).
                if _trace:
                    job.watch.setdefault("slip", [])
                    job.watch["slip"].append(dev)
                    if len(job.watch["slip"]) % 3 == 1:
                        self.say(f"[어긋남] step {job.steps} {name} {dev*100:.2f}cm "
                                 f"손기준차 {[round(float(v)*100, 1) for v in (_rel - job.watch['rel0'])]}")
                # **문턱을 설정으로 뺀다** (`SIM_HAND_DRIFT_M`, 기본 0.03).
                # 이 값은 손 좌표계에서 본 책 **원점**의 이동량인데, 이 레벨의 책들은
                # 원점이 형상에서 75~177 cm 떨어져 있어(2026-09-22 실측) 손이 조금만
                # 돌아도 cm 단위로 벌어진다. 키네마틱 파지 + 충돌 끄기로 물리적으로
                # 빠질 수 없는 상태에서도 3.9 cm 가 찍혀 작업이 취소됐다.
                if dev > float(os.environ.get("SIM_HAND_DRIFT_M", "0.03")) and name != "lift":
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
