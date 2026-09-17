"""Isaac Sim 쪽 PlaceBook 작업 실행기 (방식 '나').

    manipulation_node --/manipulation/sim/command (std_msgs/String JSON)--> 이 스크립트
                      <--/manipulation/sim/state   (std_msgs/String JSON)--

- 명령 규약·단계 이름·오류 코드는 팀 저장소 shelving_manipulation/book_placer.py 를 그대로 import 한다
- 궤적은 여기서 사전 계획(5mm·2°)하고 관절 보간으로 실행한다. 매 스텝 관절 목표를 ROS 로 주고받지 않는다
- 장면은 레벨 v3 + 트레이 + 책 N권 + 1차 고정 칸 북엔드 (book_scene.BookScene)

GPU PC (ROS 환경변수는 run_place_book_server.sh 가 설정):
    ~/arm/isaac/run_place_book_server.sh            # 헤드리스
    ~/arm/isaac/run_place_book_server.sh --gui      # 화면 보기
"""
import argparse
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/Desktop/ing_library_env_v3.usd"))
ap.add_argument("--tray", default=os.path.expanduser("~/book_dataset/assets/tray/tray_v1.usdc"))
ap.add_argument("--tray-center", type=float, nargs=2, default=[2.36, -2.94])
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27],
                help="1차 고정 칸 (팔 원점 x 기준). 북엔드를 세운다")
ap.add_argument("--command-topic", default="/manipulation/sim/command")
ap.add_argument("--state-topic", default="/manipulation/sim/state")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--camera", action="store_true",
                help="손목 카메라 + /rgb /depth /camera_info /tf /clock 발행 (비전 연동 시험)")
ap.add_argument("--max-seconds", type=float, default=0.0, help="0 이면 계속 실행")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": not args.gui})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    app.update()

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

sys.path.insert(0, os.path.expanduser("~/shelving_manipulation_py"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shelving_manipulation.book_placer import (  # noqa: E402
    COMMAND_CANCEL, COMMAND_PLACE, decode, encode, SIM_CANCELLED, SIM_FAILED, SIM_IDLE, SIM_RUNNING,
    SIM_SUCCEEDED)
from book_scene import BookScene, VEL_LIMIT  # noqa: E402
from arm_primitives import Status  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n"); sys.stderr.flush()


scene = BookScene(app, args.usd, args.tray, args.tray_center, args.books, args.place_dx, say)
world, arm, robot = scene.world, scene.arm, scene.robot
render = args.gui or args.camera
if args.camera:
    import ros_sensors  # noqa: E402
    from book_scene import R  # noqa: E402
    cam_path = ros_sensors.add_wrist_camera(scene.stage, R)
    ros_sensors.build_ros_graph(cam_path, R)
    world.play()
    say(f"손목 카메라 {cam_path} → /rgb /depth /camera_info (frame {ros_sensors.OPTICAL_FRAME}), /tf, /clock")

rclpy.init()
node = rclpy.create_node("isaac_place_book_executor")
pub = node.create_publisher(String, args.state_topic, 10)
inbox = []
node.create_subscription(String, args.command_topic, lambda m: inbox.append(m.data), 10)


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


job = None           # 실행 중 또는 마지막으로 끝난 작업
ready = False        # 시작 홈 이동을 마쳐야 명령을 받는다
q_prev = robot.get_joint_positions()[scene.idx_arm].copy()

# 시작: 팔을 접은 채 트레이 위 홈으로 (arm_planning.tucked_joint_moves)
for p in scene.home_moves():
    arm.enqueue(p)


def publish(state):
    pub.publish(String(data=encode(dict(state, stamp=time.time(), ready=ready))))


def finish(status, **extra):
    job.state = dict(job.state, status=status, **extra)
    publish(job.state)
    say(f"작업 {job.job_id} {status} {extra} ({time.time() - job.started:.1f}s)")


def start_job(cmd):
    """명령 → 책 선택 → 계획. 실패하면 움직이지 않고 끝낸다 (M401/402/410/411)"""
    global job
    job = Job(cmd)
    publish(job.state)
    if not ready:
        return finish(SIM_FAILED, error_code=411, message="Isaac 작업 실행기 초기화 중 (시작 홈 이동)")
    if cmd.get("frame_id") != "arm_base_link":
        return finish(SIM_FAILED, error_code=410, message=f"frame_id {cmd.get('frame_id')}")
    pick_w = scene.to_world(cmd["pick"]["center"])
    place_w = scene.to_world(cmd["place"]["center"])
    book, dist = scene.book_on_tray_near(pick_w)
    if book is None:
        return finish(SIM_FAILED, error_code=411,
                      message=f"트레이 칸 {cmd['pick'].get('tray_slot')} 에 책 없음 (3cm 안)")
    plan, code, err = scene.plan_job(book, place_w)
    if plan is None:
        return finish(SIM_FAILED, error_code=code, message=f"계획 실패 {err}")
    job.plan = plan
    say(f"작업 {job.job_id}: {book.split('/')[-1]} (칸 {cmd['pick'].get('tray_slot')}, {dist * 100:.1f}cm) → "
        f"x {plan['place_x']:.3f} 계획 OK 최대 인접변화 {plan['worst']:.3f} rad")
    arm.enqueue(scene.job_sequence("job", plan))


def handle(text):
    cmd = decode(text)
    if cmd is None:
        say(f"명령 형식 오류: {text[:80]}"); return
    kind = cmd.get("type")
    if kind == COMMAND_PLACE:
        if job is not None and job.state["status"] == SIM_RUNNING:
            publish({"token": cmd.get("token"), "job_id": cmd.get("job_id", ""), "status": SIM_FAILED,
                     "phase": "plan", "error_code": 411, "message": f"작업 {job.job_id} 실행 중"})
            return
        start_job(cmd)
    elif kind == COMMAND_CANCEL and job is not None and job.token == cmd.get("token") \
            and job.state["status"] == SIM_RUNNING:
        # 안전 동작: 그 자리 정지. 책을 잡고 있을 수 있으므로 그리퍼·고정 조인트는 그대로 둔다 (HOLD_GRIP)
        arm.cancel()
        finish(SIM_CANCELLED, message="취소 — 그 자리 정지 (그리퍼 유지)")


say("작업 실행기 시작 — 시작 홈 이동 중")
t0 = time.time()
last_pub = 0.0
step = 0
while app.is_running():
    rclpy.spin_once(node, timeout_sec=0.0)
    while inbox:
        handle(inbox.pop(0))

    status = arm.update()
    world.step(render=render)
    step += 1

    q_now = robot.get_joint_positions()[scene.idx_arm]
    ratio = np.abs(q_now - q_prev) / world.get_physics_dt() / VEL_LIMIT
    q_prev = q_now.copy()

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
            job.watch["z0"] = scene.center(book)[2]; job.watch["rel0"] = scene.book_in_hand(book)
        if name == "carry_rotate" and "z0" in job.watch and "rise" not in job.watch:
            job.watch["rise"] = scene.center(book)[2] - job.watch["z0"]
            if job.watch["rise"] < 0.08:
                arm.cancel(); finish(SIM_FAILED, error_code=405, message=f"들어 올린 뒤 책 상승 {job.watch['rise'] * 100:.1f}cm")
        if name in ("carry_rotate", "wedge") and "rel0" in job.watch and job.state["status"] == SIM_RUNNING:
            dev = float(np.linalg.norm(scene.book_in_hand(book) - job.watch["rel0"]))
            if dev > 0.03:
                arm.cancel(); finish(SIM_FAILED, error_code=406, message=f"운반 중 손 안에서 책 {dev * 100:.1f}cm 어긋남")

    # 위 감시에서 이미 끝냈으면 이번 스텝에 다시 판정하지 않는다
    running = running and job.state["status"] == SIM_RUNNING
    if status is Status.FAILED:
        code, err = arm.error_code or 404, arm.error
        arm.cancel()
        if not ready:
            say(f"시작 홈 이동 실패 M{code} {err} — 명령을 받지 않는다")
        elif running:
            finish(SIM_FAILED, error_code=code, message=err)
    elif arm.idle:
        if not ready:
            ready = True
            say(f"준비 완료 (step {step}) — 명령 대기 {args.command_topic}")
        elif running:
            for _ in range(60):
                world.step(render=render)
            ok, checks, bb = scene.verify(job.plan)
            spine_arm = scene.to_arm([0, bb[1], 0])[1]
            extra = {"placement_verified": bool(ok), "checks": {k: bool(v) for k, v in checks.items()},
                     "book_aabb_center_arm": np.round(scene.to_arm((bb[:3] + bb[3:]) / 2), 4).tolist(),
                     "joint_peak_ratio": round(job.peak, 3), "joint_over80_steps": job.spikes}
            if job.spikes:
                finish(SIM_FAILED, error_code=403, message=f"관절 각속도 80% 초과 {job.spikes}스텝 (최대 {job.peak * 100:.0f}%)", **extra)
            else:
                finish(SIM_SUCCEEDED, phase="verify",
                       message=("배치 확인" if ok else f"배치 확인 실패 {checks}"), **extra)

    now = time.time()
    if now - last_pub >= 0.1:
        publish(job.state if job is not None else {"token": None, "status": SIM_IDLE})
        last_pub = now
    if args.max_seconds and now - t0 > args.max_seconds:
        say("max-seconds 도달 — 종료"); break

node.destroy_node()
rclpy.shutdown()
app.close()
