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
from std_msgs.msg import String

from shelving_manipulation.book_placer import (
    COMMAND_CANCEL, COMMAND_PLACE, COMMAND_SCAN, decode, encode, SIM_CANCELLED, SIM_FAILED, SIM_IDLE, SIM_RUNNING,
    SIM_SUCCEEDED)

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
        self.scene.job_active = True
        self.scene.root_writes_after_job = 0
        try:
            poses = self.scene.plan_scan()
        except Exception as exc:      # noqa: BLE001 - 계획 실패를 작업 실패로 돌려준다
            self.scene.job_active = False
            self.publish({"token": token, "job_id": job_id, "status": SIM_FAILED,
                          "phase": "plan", "error_code": 410,
                          "message": f"스캔 계획 실패 {type(exc).__name__}: {exc}"})
            return
        if not poses:
            self.scene.job_active = False
            self.publish({"token": token, "job_id": job_id, "status": SIM_FAILED,
                          "phase": "plan", "error_code": 410, "message": "스캔 자세 해가 없다"})
            return

        steps = []
        for name, q, meta in poses:
            steps.append(named(MoveJoint(q, speed_scale=0.6), name))
            steps.append(named(Wait(dwell), f"{name}_hold"))
        self.job = Job(dict(cmd, token=token, job_id=job_id))
        self.job.state = {"token": token, "job_id": job_id, "status": SIM_RUNNING,
                          "phase": "scan", "scan_poses": len(poses)}
        self.publish(self.job.state)
        self.say(f"스캔 시작: 자세 {len(poses)}개, 자세마다 {dwell:.1f}초 정지 "
                 f"(최대 관절 변화 {max(m['step_rad'] for _, _, m in poses):.2f} rad)")
        # **꽂을 수 있는 판과 아닌 판을 미리 알린다** — 비전이 준 빈칸을 나중에 거를 때 쓴다
        self.publish(dict(self.job.state, phase="scan_plan",
                          boards=[{"board_z": m["board_z"], "name": n,
                                   "reachable": m["reachable"]} for n, _, m in poses]))
        self.arm.enqueue(Sequence("scan", steps))

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
                if os.environ.get("SIM_DRIFT_LOG", "0") != "0" or \
                        os.environ.get("SIM_DRIFT_METRIC", "origin") == "center":
                    try:
                        job.watch["relc0"] = scene.book_in_hand_center(book)
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
                # 형상중심 기준 (SIM_DRIFT_LOG=1 기록 / SIM_DRIFT_METRIC=center 판정 전환, NIGHTLY 1-1·1-2)
                _dc = _ang = None
                if "relc0" in job.watch:
                    try:
                        _c, _Rr = scene.book_in_hand_center(book)
                        _c0, _R0 = job.watch["relc0"]
                        _dc = float(np.linalg.norm(_c - _c0))
                        _cos = (float(np.trace(_R0.T @ _Rr)) - 1.0) / 2.0
                        _ang = float(np.degrees(np.arccos(min(1.0, max(-1.0, _cos)))))
                        job.watch["dc_max"] = max(job.watch.get("dc_max", 0.0), _dc)
                        job.watch["ang_max"] = max(job.watch.get("ang_max", 0.0), _ang)
                        job.watch["dev_max"] = max(job.watch.get("dev_max", 0.0), dev)
                        job.watch["n_drift"] = job.watch.get("n_drift", 0) + 1
                        if os.environ.get("SIM_DRIFT_LOG", "0") != "0" and job.watch["n_drift"] % 30 == 1:
                            self.say(f"[어긋남] step {job.steps} phase={name}  원점기준 {dev * 100:.2f} cm | "
                                     f"형상중심기준 {_dc * 100:.2f} cm | 손기준 회전 {_ang:.1f}°  "
                                     f"(최대 {job.watch['dev_max'] * 100:.2f} / {job.watch['dc_max'] * 100:.2f} cm / "
                                     f"{job.watch['ang_max']:.1f}°)")
                    except Exception as _exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                        _dc = _ang = None
                        if not job.watch.get("drift_err"):
                            job.watch["drift_err"] = True
                            self.say(f"[어긋남] 형상중심 계산 실패: {type(_exc).__name__}: {_exc}")
                if os.environ.get("SIM_DRIFT_METRIC", "origin") == "center" and _dc is not None:
                    _bad = (_dc > float(os.environ.get("SIM_DRIFT_CENTER_M", "0.01"))
                            or _ang > float(os.environ.get("SIM_DRIFT_ROT_DEG", "5")))
                    _msg = f"운반 중 손 안에서 책 형상중심 {_dc * 100:.1f}cm · 회전 {_ang:.1f}° 어긋남 (원점기준 {dev * 100:.1f}cm)"
                else:
                    _bad = dev > float(os.environ.get("SIM_HAND_DRIFT_M", "0.03"))
                    _msg = f"운반 중 손 안에서 책 {dev * 100:.1f}cm 어긋남"
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
            elif running:
                self.finish(SIM_FAILED, error_code=code, message=err)
        elif arm.idle:
            if not self.announced:
                self.announced = True
                self.ready = True
                self.say(f"준비 완료 (step {self.step}) — 명령 대기 {self.command_topic}")
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
