"""로봇팔 실행기 — Isaac 안에서 `/manipulation/sim/command` 를 받아 실행하고 `/manipulation/sim/state` 를 낸다.

`place_book_server.py` 의 로봇팔 부분을 그대로 옮긴 것이다. **통신 규약과 판정 기준은 바꾸지 않았다.**
    수신 /manipulation/sim/command   발행 /manipulation/sim/state
    오류 코드 M401·402·403·404·405·406·409·410·411·412 (book_placer.ERRORS)

Isaac 실행 자체(앱 생성·USD 로드·루프)는 `run_simulation.py` 가 맡는다.
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_msgs.msg import String

from shelving_manipulation.book_placer import (
    COMMAND_CANCEL, COMMAND_PLACE, decode, encode, SIM_CANCELLED, SIM_FAILED, SIM_IDLE, SIM_RUNNING,
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
<<<<<<< HEAD

        scene.refresh_arm_frame()
=======
        # **좌표를 월드로 바꾸기 전에** 팔 베이스 자세를 다시 읽는다. 주행한 뒤라면
        # 시작할 때 읽은 값이 그만큼 낡아 있어 pick/place 가 통째로 어긋난다 (2026-09-21).
        scene.job_active = True          # follow_tray 가 책을 건드리지 않게 한다
        _moved = scene.refresh_base()
        if _moved > 0.01:
            self.say(f"팔 베이스가 {_moved*100:.1f}cm 움직였다 — 좌표 기준을 다시 잡았다")
>>>>>>> origin/feature/amr_patrol_pickplace
        pick_w = scene.to_world(cmd["pick"]["center"])
        place_w = scene.to_world(cmd["place"]["center"])
        book, dist = scene.book_on_tray_near(pick_w)

        # 시연 전용: 계산된 트레이 칸에서 멀어도 실제 가장 가까운 책을 사용한다.
        demo_nearest = os.environ.get(
            "SIM_DEMO_NEAREST_BOOK",
            "0",
        ) == "1"

        if book is None and demo_nearest and scene.books:
            book = min(
                scene.books,
                key=lambda path: float(
                    np.linalg.norm(scene.center(path) - pick_w)
                ),
            )
            dist = float(np.linalg.norm(scene.center(book) - pick_w))
            self.say(
                "시연 모드: 트레이 칸 거리 검사를 무시하고 "
                f"{book.rsplit('/', 1)[-1]} 선택 "
                f"(기준점과 {dist * 100:.1f}cm)"
            )

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
            if name in _watch_names and "rel0" in job.watch \
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
