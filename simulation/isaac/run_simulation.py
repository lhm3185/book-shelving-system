"""B-1 Isaac Sim 통합 실행기 — 팀 전체의 공식 진입점 (Isaac Sim 5.1.0).

    SimulationApp 생성 → 통합 USD 로드 → ROS2 Bridge → 카메라·센서 → 실행기 등록 → 시뮬 루프

실행은 `scripts/run_isaac_sim.sh` 로 한다 (환경 정리·Isaac python.sh 호출 포함).
경로는 저장소 기준으로 계산하므로 clone 한 위치와 상관없이 돈다. 개인 절대경로를 기본값으로 쓰지 않는다.

통신 규약은 예전 `place_book_server.py` 와 같다 (바꾸지 않았다):
    수신 /manipulation/sim/command      발행 /manipulation/sim/state
"""
import argparse
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="", help="열 USD. 비우면 simulation/library_system.usd (환경변수 SIM_USD 도 가능)")
ap.add_argument("--tray", default="", help="트레이 USD. 비우면 simulation/assets/tray.usd")
ap.add_argument("--tray-center", type=float, nargs=2, default=[2.36, -2.94])
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--book-variants", default="", help="'mixed' 면 색·크기가 다른 기본 6종, 쉼표 목록도 가능")
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27],
                help="1차 고정 칸 (팔 원점 x 기준). 북엔드를 세운다")
ap.add_argument("--command-topic", default="/manipulation/sim/command")
ap.add_argument("--state-topic", default="/manipulation/sim/state")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--headless", action="store_true", help="--gui 와 반대. 둘 다 없으면 headless")
ap.add_argument("--camera", action="store_true", help="레벨에 카메라가 없을 때 손목 카메라를 만든다")
ap.add_argument("--camera-prim", default="", help="레벨 로봇에 이미 있는 카메라 prim 경로")
ap.add_argument("--camera-hz", type=float, default=10.0)
ap.add_argument("--sensor-policy", choices=["gated", "always"], default="gated")
ap.add_argument("--amr-test-overrides", action="store_true",
                help="AMR 에셋의 라이다 fullScan·TF 네임스페이스를 실행에서만 보완 (파일 미수정)")
ap.add_argument("--start-home", choices=["move", "snap"], default="move",
                help="move: 접은 채 홈으로 이동(검증된 경로). snap 은 첫 작업이 M406 으로 실패한다")
ap.add_argument("--max-seconds", type=float, default=0.0, help="0 이면 계속 실행")
ap.add_argument("--no-manipulation", action="store_true", help="로봇팔 실행기를 붙이지 않는다 (월드만 확인)")
args = ap.parse_args()

# 저장소 코드를 그대로 쓴다 (~/arm 으로 복사하지 않는다)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "controllers"))
sys.path.insert(0, str(HERE / "sensors"))
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "shelving_manipulation"))

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": not args.gui})

import ros_bridge  # noqa: E402
import world_loader  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


ros_bridge.enable_bridge(app, say=say)

usd = world_loader.resolve_usd(args.usd or None)
tray = args.tray or os.environ.get("SIM_TRAY") or str(world_loader.DEFAULT_TRAY)
if not os.path.exists(tray) or world_loader.is_placeholder(tray):
    legacy = Path(os.path.expanduser("~/book_dataset/assets/tray/tray_v1.usdc"))
    if legacy.exists():
        say(f"트레이 USD 가 비어 있어 예전 경로를 쓴다: {legacy}")
        tray = str(legacy)

from book_scene import BookScene, R  # noqa: E402
import camera_bridge  # noqa: E402
from manipulation_executor import ManipulationExecutor  # noqa: E402

MIXED_BOOKS = [
    "book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15",
    "decorative_book_set_01_2k__book_hardcover_01_cover02",
    "decorative_book_set_01_2k__book_softcover_01_cover14",
    "decorative_book_set_01_2k__book_hardcover_01_cover08",
    "oldbook__OldBook001",
    "book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book01",
]


def _before_reset(stage):
    """재생(초기화) 전에만 반영되는 설정 — OmniGraph 값은 실행 중 바꾸면 무시된다"""
    if (args.camera_prim or args.amr_test_overrides) and args.camera_hz < 60:
        camera_bridge.SensorGate.preset_camera_hz(stage, R, args.camera_hz, say)
    if args.amr_test_overrides:
        camera_bridge.apply_amr_test_overrides(stage, R, say)


variants = (MIXED_BOOKS if args.book_variants.strip() == "mixed"
            else [v.strip() for v in args.book_variants.split(",") if v.strip()] or None)

# 월드: USD 를 열고 Prim 을 검사한 뒤 트레이·책·로봇팔을 준비한다
stage0 = world_loader.open_world(app, usd, say)
world_loader.check_prims(stage0, say)
scene = BookScene(app, usd, tray, args.tray_center, args.books, args.place_dx, say,
                  before_reset=_before_reset, book_variants=variants)
world = scene.world

# 센서
render = args.gui or args.camera or bool(args.camera_prim)
gate = None
if args.camera_prim:
    if not scene.stage.GetPrimAtPath(args.camera_prim).IsValid():
        say(f"카메라 prim 없음: {args.camera_prim}")
    camera_bridge.build_supplement_graph(args.camera_prim, R)
    world.play()
    say(f"기존 카메라 사용 {args.camera_prim} → TF panda_link0→{args.camera_prim.split('/')[-1]}, /clock 추가")
elif args.camera:
    cam_path = camera_bridge.add_wrist_camera(scene.stage, R)
    camera_bridge.build_ros_graph(cam_path, R)
    world.play()
    say(f"손목 카메라 {cam_path} → /rgb /depth /camera_info (frame {camera_bridge.OPTICAL_FRAME}), /tf, /clock")
if args.camera or args.camera_prim or args.amr_test_overrides:
    gate = camera_bridge.SensorGate(scene.stage, R, say)
    if args.camera_hz < 60:
        gate.set_camera_hz(args.camera_hz)
    say(f"센서 정책 {args.sensor_policy}: 카메라 노드 {len(gate.camera_nodes)}, 라이다 노드 {len(gate.lidar_nodes)}")

# 실행기 등록 (지금은 로봇팔. AMR 은 navigation_executor.py 자리에 붙인다)
node = ros_bridge.make_node("isaac_place_book_executor")
executor = None
if not args.no_manipulation:
    executor = ManipulationExecutor(scene, node, say, command_topic=args.command_topic,
                                    state_topic=args.state_topic, gate=gate,
                                    sensor_policy=args.sensor_policy, start_home=args.start_home,
                                    render=render, gui=args.gui)
    say("작업 실행기 시작 — 시작 홈 이동 중" if args.start_home == "move" else "작업 실행기 시작 — 홈 자세 고정")
else:
    say("월드만 실행한다 (로봇팔 실행기 없음)")

t0 = time.time()
while app.is_running():
    if executor is not None:
        executor.spin()
    else:
        world.step(render=render)
    if args.max_seconds and time.time() - t0 > args.max_seconds:
        say("max-seconds 도달 — 종료")
        break

ros_bridge.shutdown(node)
app.close()
