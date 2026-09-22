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
# **팔 기준(arm_base_link) 좌표**다. 예전에는 월드 좌표(2.36, -2.94)였는데, 로봇이 다른
# 자리로 가면 그대로 깨진다. 기본값은 book_profiles.yaml 의 트레이 칸 평균과 같다.
ap.add_argument("--tray-center", type=float, nargs=2, default=[-0.4748, 0.0788],
                help="트레이 중앙 (arm_base_link 기준 x y)")
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--book-variants", default="", help="'mixed' 면 색·크기가 다른 기본 6종, 쉼표 목록도 가능")
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27],
                help="1차 고정 칸 (팔 원점 x 기준). 북엔드를 세운다")
# 카메라 토픽 이름. **기본값은 지금과 같다** — 바꾸지 않으면 동작이 달라지지 않는다.
# 왜 필요한가: AMR 담당 카메라도 같은 ROS_DOMAIN_ID 에서 `/rgb` 로 나가면 rqt 에 어느 쪽이
# 보이는지 보장되지 않는다 (2026-09-19 실제로 AMR 담당 시험 화면에 우리 손목 카메라가 잡혔다).
# 팀이 네임스페이스를 나누기로 하면 `--camera-ns /arm` 한 줄로 전환한다.
ap.add_argument("--camera-ns", default="", metavar="접두사",
                help="카메라 토픽 앞에 붙일 네임스페이스 (예: /arm → /arm/rgb). 비우면 지금 그대로")
ap.add_argument("--command-topic", default="/manipulation/sim/command")
ap.add_argument("--state-topic", default="/manipulation/sim/state")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--headless", action="store_true", help="--gui 와 반대. 둘 다 없으면 headless")
ap.add_argument("--record-dir", default="", metavar="경로",
                help="시연 녹화: 장면을 내려다보는 카메라를 만들어 프레임을 PNG 로 저장한다. "
                     "**헤드리스로 동작**하므로 GPU PC 바탕화면(팀원이 쓰는 화면)을 건드리지 않는다. "
                     "화면 녹화로 찍으면 남의 작업 화면이 찍힌다 (2026-09-19 실제로 그랬다)")
ap.add_argument("--record-every", type=int, default=6, help="몇 스텝마다 한 장 (60 Hz 기준 6 = 10 fps)")
# 좌표를 추측하면 엉뚱한 곳(서가 벽)을 찍는다 — 기본은 **로봇 AABB 에 자동으로 맞춘다**
ap.add_argument("--record-eye", type=float, nargs=3, default=None, help="녹화 카메라 위치 (기본: 자동)")
ap.add_argument("--record-look", type=float, nargs=3, default=None, help="보는 점 (기본: 로봇 중심)")
ap.add_argument("--camera", action="store_true", help="레벨에 카메라가 없을 때 손목 카메라를 만든다")
ap.add_argument("--camera-prim", default="", help="레벨 로봇에 이미 있는 카메라 prim 경로")
ap.add_argument("--camera-hz", type=float, default=10.0)
ap.add_argument("--sensor-policy", choices=["gated", "always"], default="gated")
ap.add_argument("--amr-test-overrides", action="store_true",
                help="AMR 에셋의 라이다 fullScan·TF 네임스페이스를 실행에서만 보완 (파일 미수정)")
ap.add_argument("--start-home", choices=["move", "snap", "keep"], default="move",
                help="move: 접은 채 홈으로 이동(검증된 경로). snap 은 첫 작업이 M406 으로 실패한다")
ap.add_argument("--max-seconds", type=float, default=0.0, help="0 이면 계속 실행")
ap.add_argument("--no-manipulation", action="store_true", help="로봇팔 실행기를 붙이지 않는다 (월드만 확인)")
ap.add_argument("--no-navigation", action="store_true",
                help="주행 실행기를 붙이지 않는다")
ap.add_argument("--drive-speed", type=float, default=0.4, help="주행 속도 (m/s)")
ap.add_argument("--probe-tray", action="store_true",
                help="장면만 세우고 **설정 칸 좌표 vs 실제 책 위치**를 찍은 뒤 끝낸다 "
                     "(파지하지 않는다). 세션을 여러 번 돌려 계통/무작위를 가른다")
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
from navigation_executor import NavigationExecutor  # noqa: E402

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
    # 부모 프레임 이름은 로봇마다 다르다 (Isaac 은 prim **이름**을 frame 으로 낸다).
    # 예전엔 panda_link0 이 문구에 박혀 있어 M0609 에서 로그가 거짓말을 했다 (2026-09-20).
    _parent = world_loader.BOT.base_link.split("/")[-1]
    say(f"기존 카메라 사용 {args.camera_prim} → TF {_parent}→{args.camera_prim.split('/')[-1]}, /clock 추가")
elif args.camera:
    cam_path = camera_bridge.add_wrist_camera(scene.stage, R)
    _ns = args.camera_ns.rstrip("/")
    camera_bridge.build_ros_graph(cam_path, R, rgb=f"{_ns}/rgb", depth=f"{_ns}/depth",
                                  info=f"{_ns}/camera_info")
    world.play()
    say(f"손목 카메라 {cam_path} → {_ns}/rgb {_ns}/depth {_ns}/camera_info "
        f"(frame {camera_bridge.OPTICAL_FRAME}), /tf, /clock")
if args.camera or args.camera_prim or args.amr_test_overrides:
    gate = camera_bridge.SensorGate(scene.stage, R, say)
    if args.camera_hz < 60:
        gate.set_camera_hz(args.camera_hz)
    say(f"센서 정책 {args.sensor_policy}: 카메라 노드 {len(gate.camera_nodes)}, 라이다 노드 {len(gate.lidar_nodes)}")

# --- 트레이 오차 측정: 장면만 세우고 표를 찍은 뒤 끝낸다
if args.probe_tray:
    import json as _json
    for _ in range(int(os.environ.get("SIM_PROBE_SETTLE_STEPS", "120"))):
        world.step(render=False)
    # **로봇이 실제로 쓰는 설정값**과 비교한다 (시뮬이 아는 값이 아니라).
    # 이 파일이 파지 목표의 출처다 — 여기가 틀리면 파지가 틀린다
    _cfg = None
    try:
        import yaml as _yaml
        _pf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                           "ros2_ws/src/shelving_manipulation/config/book_profiles.yaml")
        with open(os.path.abspath(_pf), encoding="utf-8") as _fh:
            _slots = ((_yaml.safe_load(_fh) or {}).get("tray") or {}).get("slots") or []
        _cfg = [sl["center"] for sl in sorted(_slots, key=lambda x: x.get("index", 0))]
        say(f"[PROBE] 설정 칸 {len(_cfg)}개 ({os.path.basename(_pf)})")
    except Exception as _exc:      # noqa: BLE001
        say(f"[PROBE] 설정 칸을 못 읽었다: {type(_exc).__name__}: {_exc}")
    _rows = []
    for _i, _b in enumerate(scene.books):
        _c = scene.center(_b)
        _arm = scene.to_arm(_c)
        _row = {"slot": _i, "book": _b.rsplit("/", 1)[-1],
                "world": [round(float(v), 5) for v in _c],
                "arm": [round(float(v), 5) for v in _arm]}
        if _cfg is not None and _i < len(_cfg):
            _row["cfg"] = [round(float(v), 5) for v in _cfg[_i]]
            _row["err_mm"] = [round(float((_arm[_k] - _cfg[_i][_k]) * 1000), 2) for _k in (0, 1, 2)]
        _rows.append(_row)
        say(f"[PROBE] {_json.dumps(_row, ensure_ascii=False)}")
    say(f"[PROBE] done books={len(_rows)}")
    app.close()
    raise SystemExit(0)

# 실행기 등록 — 로봇팔과 주행. 둘 다 같은 노드를 쓰고 자기 토픽만 만든다
node = ros_bridge.make_node("isaac_place_book_executor")
executor = None
nav = None
if not args.no_navigation:
    nav = NavigationExecutor(scene, node, say, speed=args.drive_speed)
if not args.no_manipulation:
    executor = ManipulationExecutor(scene, node, say, command_topic=args.command_topic,
                                    state_topic=args.state_topic, gate=gate,
                                    sensor_policy=args.sensor_policy, start_home=args.start_home,
                                    render=render, gui=args.gui)
    say("작업 실행기 시작 — 시작 홈 이동 중" if args.start_home == "move" else "작업 실행기 시작 — 홈 자세 고정")
else:
    say("월드만 실행한다 (로봇팔 실행기 없음)")

# --- 녹화 준비: 장면 카메라 + 렌더 프로덕트 (헤드리스에서도 동작한다)
rec = None
if args.record_dir:
    import numpy as _np
    import omni.replicator.core as _rep
    from PIL import Image as _Image
    from pxr import Gf as _Gf, UsdGeom as _UsdGeom, UsdLux as _UsdLux
    from isaacsim.core.prims import SingleXFormPrim as _XForm

    _out = os.path.expanduser(args.record_dir)
    os.makedirs(_out, exist_ok=True)
    from isaacsim.core.utils.stage import get_current_stage as _get_stage
    _st = _get_stage()
    _UsdLux.DomeLight.Define(_st, "/World/rec_dome").CreateIntensityAttr(600.0)
    _cam_path = "/World/rec_cam"
    _cam = _UsdGeom.Camera.Define(_st, _cam_path)
    _cam.CreateFocalLengthAttr(22.0)
    _cam.CreateHorizontalApertureAttr(36.0)
    _cam.CreateVerticalApertureAttr(20.25)
    _cam.CreateClippingRangeAttr(_Gf.Vec2f(0.05, 100.0))
    # 로봇·트레이가 한 화면에 들어오도록 **로봇 AABB 로 자동 구도**를 잡는다
    from isaacsim.core.utils.bounds import compute_aabb as _aabb, create_bbox_cache as _bbc
    _c = _bbc(); _c.Clear()
    _bb = _np.array(_aabb(_c, world_loader.BOT.root, include_children=True), float)
    if not _np.all(_np.isfinite(_bb)):
        _bb = _np.array([-1, -1, 0, 1, 1, 1.5], float)
    _mid = (_bb[:3] + _bb[3:]) / 2
    _rad = float(_np.linalg.norm(_bb[3:] - _bb[:3]))
    _tgt = _np.array(args.record_look, float) if args.record_look else _mid
    # 로봇 앞쪽 비스듬히 위에서 — 트레이(앞)와 팔이 같이 보이는 각도
    _eye = (_np.array(args.record_eye, float) if args.record_eye
            else _mid + _np.array([_rad * 0.9, -_rad * 1.0, _rad * 0.55]))
    say(f"녹화 구도: 로봇 중심 {_np.round(_mid,2).tolist()} 크기 {_rad:.2f} m "
        f"→ 카메라 {_np.round(_eye,2).tolist()}")
    _f = _tgt - _eye; _f /= (_np.linalg.norm(_f) or 1.0)
    _up = _np.array([0.0, 0.0, 1.0])
    _r = _np.cross(_f, _up); _r /= (_np.linalg.norm(_r) or 1.0)
    _u = _np.cross(_r, _f)
    _m = _np.eye(3); _m[:, 0], _m[:, 1], _m[:, 2] = _r, _u, -_f
    _tr = _np.trace(_m)
    if _tr > 0:
        _sq = _np.sqrt(_tr + 1.0) * 2
        _q = _np.array([0.25 * _sq, (_m[2, 1] - _m[1, 2]) / _sq,
                        (_m[0, 2] - _m[2, 0]) / _sq, (_m[1, 0] - _m[0, 1]) / _sq])
    else:
        _i = int(_np.argmax(_np.diag(_m))); _j, _k = (_i + 1) % 3, (_i + 2) % 3
        _sq = _np.sqrt(1.0 + _m[_i, _i] - _m[_j, _j] - _m[_k, _k]) * 2
        _q = _np.zeros(4); _q[0] = (_m[_k, _j] - _m[_j, _k]) / _sq
        _q[_i + 1] = 0.25 * _sq
        _q[_j + 1] = (_m[_j, _i] + _m[_i, _j]) / _sq
        _q[_k + 1] = (_m[_k, _i] + _m[_i, _k]) / _sq
        _q /= _np.linalg.norm(_q)
    _XForm(_cam_path).set_world_pose(_eye, _q)
    _rp = _rep.create.render_product(_cam_path, (1280, 720))
    _ann = _rep.AnnotatorRegistry.get_annotator("rgb")
    _ann.attach(_rp)
    rec = {"n": 0, "saved": 0, "out": _out, "ann": _ann, "img": _Image, "np": _np}
    say(f"녹화 시작 → {_out} ({args.record_every} 스텝마다 1장, 1280x720)")

t0 = time.time()
while app.is_running():
    # **주행이 먼저다.** 자세만 갱신하고 world.step() 은 부르지 않는다 —
    # 스텝을 부르는 쪽은 하나여야 한다 (로봇팔 실행기, 없으면 아래 else)
    if nav is not None:
        nav.spin()
    if executor is not None:
        executor.spin()
    else:
        world.step(render=render)
    if rec is not None:
        rec["n"] += 1
        if rec["n"] % args.record_every == 0:
            _d = rec["ann"].get_data()
            if _d is not None and getattr(_d, "size", 0):
                _a = rec["np"].asarray(_d)[:, :, :3].astype("uint8")
                rec["img"].fromarray(_a).save(
                    os.path.join(rec["out"], f"f{rec['saved']:06d}.png"))
                rec["saved"] += 1
    if args.max_seconds and time.time() - t0 > args.max_seconds:
        say("max-seconds 도달 — 종료")
        break

if rec is not None:
    say(f"녹화 종료 — {rec['saved']}장 저장 ({rec['out']})")
ros_bridge.shutdown(node)
app.close()
