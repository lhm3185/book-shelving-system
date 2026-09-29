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

# 레벨에 저장된 관절 자세를 쓰는 모드에서는 USD의 drive target도 그대로 둔다.
# 그렇지 않으면 BookScene 초기화가 target을 0도로 바꿔 카메라가 책장을 보던
# 7축 자세가 재생 직후 풀린다.
if args.start_home == "keep":
    os.environ["SIM_PRESERVE_DRIVE_TARGETS"] = "1"

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
    # **녹화용 돔 라이트.** 헤드리스 렌더가 너무 어두워 붙인 것인데, 레벨의 조명을
    # 도윤님이 손으로 맞춘 뒤로는 그림을 바꿔 버린다. `SIM_REC_DOME=0` 이면 안 만든다.
    _dome = float(os.environ.get("SIM_REC_DOME", "600"))
    if _dome > 0:
        _UsdLux.DomeLight.Define(_st, "/World/rec_dome").CreateIntensityAttr(_dome)
    else:
        say("녹화: 돔 라이트 안 붙인다 (SIM_REC_DOME=0) — 레벨 조명 그대로 찍는다")
    # 카메라는 아래에서 **시점마다** 만든다 (`/World/rec_cam_<이름>`).
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
    # 자동 구도는 **시작할 때 로봇 자리**에 고정된다. 로봇이 곧 서가로 떠나므로
    # 판 대부분에서 빈 바닥만 찍는다 (2026-09-29 녹화 판 실측). `SIM_REC_AUTO=0` 이면 뺀다 —
    # 시점 하나가 렌더 프로덕트 하나라, 빼면 그만큼 시뮬이 덜 느려진다.
    _shots = {} if os.environ.get("SIM_REC_AUTO", "1") == "0" else {"": (_eye, _tgt)}
    # **시점을 더 붙인다.** `SIM_REC_SHOTS="이름=ex,ey,ez@tx,ty,tz;…"`.
    # 자동 구도는 시작할 때 로봇 AABB 로 한 번 잡고 고정이라 로봇을 따라가지 않는다 —
    # 서가까지 주행하는 판을 한 화면에 담으려면 넓은 시점을 따로 줘야 한다 (2026-09-29).
    for _spec in os.environ.get("SIM_REC_SHOTS", "").split(";"):
        _spec = _spec.strip()
        if not _spec:
            continue
        try:
            _nm, _rest = _spec.split("=", 1)
            _e_s, _t_s = _rest.split("@", 1)
            _shots[_nm.strip()] = (_np.array([float(v) for v in _e_s.split(",")], float),
                                   _np.array([float(v) for v in _t_s.split(",")], float))
        except (ValueError, IndexError):
            say(f"녹화: 시점을 못 읽었다 — {_spec!r} (꼴: 이름=ex,ey,ez@tx,ty,tz)")
    say(f"녹화 구도: 로봇 중심 {_np.round(_mid,2).tolist()} 크기 {_rad:.2f} m "
        f"→ 카메라 {_np.round(_eye,2).tolist()}")
    def _look_at(_e, _t):
        """눈 → 목표를 보는 카메라 쿼터니언 (w, x, y, z). USD 카메라는 −z 를 본다."""
        _f = _np.asarray(_t, float) - _np.asarray(_e, float)
        _f = _f / (_np.linalg.norm(_f) or 1.0)
        _r = _np.cross(_f, _np.array([0.0, 0.0, 1.0]))
        _r = _r / (_np.linalg.norm(_r) or 1.0)
        _u = _np.cross(_r, _f)
        _m = _np.eye(3)
        _m[:, 0], _m[:, 1], _m[:, 2] = _r, _u, -_f
        _tr = _np.trace(_m)
        if _tr > 0:
            _sq = _np.sqrt(_tr + 1.0) * 2
            _q = _np.array([0.25 * _sq, (_m[2, 1] - _m[1, 2]) / _sq,
                            (_m[0, 2] - _m[2, 0]) / _sq, (_m[1, 0] - _m[0, 1]) / _sq])
        else:
            _i = int(_np.argmax(_np.diag(_m)))
            _j, _k = (_i + 1) % 3, (_i + 2) % 3
            _sq = _np.sqrt(1.0 + _m[_i, _i] - _m[_j, _j] - _m[_k, _k]) * 2
            _q = _np.zeros(4)
            _q[0] = (_m[_k, _j] - _m[_j, _k]) / _sq
            _q[_i + 1] = 0.25 * _sq
            _q[_j + 1] = (_m[_j, _i] + _m[_i, _j]) / _sq
            _q[_k + 1] = (_m[_k, _i] + _m[_i, _k]) / _sq
        return _q / (_np.linalg.norm(_q) or 1.0)

    if not _shots:
        say("녹화: 시점이 하나도 없다 — SIM_REC_AUTO=0 인데 SIM_REC_SHOTS 가 비었다")
    _cams = []
    for _n, (_e, _t) in _shots.items():
        _path = f"/World/rec_cam_{_n or 'auto'}"
        _c2 = _UsdGeom.Camera.Define(_st, _path)
        _c2.CreateFocalLengthAttr(22.0)
        _c2.CreateHorizontalApertureAttr(36.0)
        _c2.CreateVerticalApertureAttr(20.25)
        _c2.CreateClippingRangeAttr(_Gf.Vec2f(0.05, 100.0))
        _XForm(_path).set_world_pose(_e, _look_at(_e, _t))
        _dir = os.path.join(_out, _n) if _n else _out
        os.makedirs(_dir, exist_ok=True)
        _rp2 = _rep.create.render_product(_path, (1280, 720))
        _an2 = _rep.AnnotatorRegistry.get_annotator("rgb")
        _an2.attach(_rp2)
        _cams.append({"name": _n or "auto", "dir": _dir, "ann": _an2})
        say(f"  [{_n or 'auto'}] 눈 {_np.round(_e, 2).tolist()} → {_np.round(_t, 2).tolist()}  →  {_dir}")
    # --- **레벨에 놓인 카메라를 그대로 쓴다** (`SIM_REC_CAMS`)
    #
    # 도윤님이 GUI 에서 카메라를 놓아 두면 그 시점들을 각각 렌더한다. 자리를 코드가 정하지
    # 않으므로 눈으로 보고 잡은 구도가 그대로 나온다 (2026-09-30).
    #
    #   SIM_REC_CAMS=auto              레벨의 카메라를 **전부** 찾는다 (로봇에 달린 것과
    #                                  우리가 만든 rec_cam_* 은 뺀다)
    #   SIM_REC_CAMS=/World/cam_a;…    그 prim 들만. `이름=/World/cam_a` 로 이름을 줘도 된다
    _cam_spec = os.environ.get("SIM_REC_CAMS", "").strip()
    # 해상도. **render product 하나가 매 프레임 그려지므로 대수와 화소가 곧 부하다**
    # (1280x720 3대에 판이 2.5배 느려졌다 — 2026-09-29 실측).
    try:
        _rw, _rh = (int(v) for v in os.environ.get("SIM_REC_RES", "1280x720").lower().split("x"))
    except ValueError:
        _rw, _rh = 1280, 720
    if _cam_spec:
        _found = []
        if _cam_spec.lower() == "auto":
            _skip = (world_loader.BOT.root, "/World/rec_cam_")
            for _p in _st.Traverse():
                if not _p.IsA(_UsdGeom.Camera):
                    continue
                _sp = str(_p.GetPath())
                if any(_sp.startswith(_k) for _k in _skip):
                    continue
                _found.append((_p.GetName(), _sp))
        else:
            for _one in _cam_spec.split(";"):
                _one = _one.strip()
                if not _one:
                    continue
                _nm, _pp = _one.split("=", 1) if "=" in _one else (None, _one)
                _pp = _pp.strip()
                if not _st.GetPrimAtPath(_pp).IsValid():
                    say(f"  [레벨카메라] 없다 — 건너뛴다: {_pp}")
                    continue
                _found.append(((_nm or _pp.rsplit("/", 1)[-1]).strip(), _pp))
        if not _found:
            say(f"  [레벨카메라] 쓸 카메라를 못 찾았다 (SIM_REC_CAMS={_cam_spec})")
        for _nm, _pp in _found:
            _dirc = os.path.join(_out, _nm)
            os.makedirs(_dirc, exist_ok=True)
            _rpc = _rep.create.render_product(_pp, (_rw, _rh))
            _anc = _rep.AnnotatorRegistry.get_annotator("rgb")
            _anc.attach(_rpc)
            _cams.append({"name": _nm, "dir": _dirc, "ann": _anc})
            say(f"  [레벨카메라] {_nm}  ←  {_pp}   {_rw}x{_rh}  →  {_dirc}")

    # --- **뷰포트를 그대로 녹화한다** (`SIM_REC_VIEWPORT=1`)
    #
    # 카메라 키를 찍는 대신, 사람이 화면에서 돌리는 그 시점을 그대로 받는다.
    # 키는 시뮬 시간에 박히는데 판마다 길이가 ±1.5 % 달라서(416~428 s 실측) 키 잡은 판과
    # 촬영 판이 다르면 뒤로 갈수록 어긋난다. 한 판에 끝내려면 이쪽이 확실하다 (2026-09-30 도윤님).
    if os.environ.get("SIM_REC_VIEWPORT", "0") != "0":
        _vp_path = os.environ.get("SIM_REC_VIEWPORT_PRIM", "/OmniverseKit_Persp")
        if _st.GetPrimAtPath(_vp_path).IsValid():
            _dirv = os.path.join(_out, "viewport")
            os.makedirs(_dirv, exist_ok=True)
            _rpv = _rep.create.render_product(_vp_path, (1280, 720))
            _anv = _rep.AnnotatorRegistry.get_annotator("rgb")
            _anv.attach(_rpv)
            _cams.append({"name": "viewport", "dir": _dirv, "ann": _anv})
            say(f"  [viewport] 화면에 보이는 그대로 받는다 ({_vp_path})  →  {_dirv}")
        else:
            say(f"  [viewport] 뷰포트 카메라를 못 찾았다 ({_vp_path}) — 이 시점은 건너뛴다")
    # **파일 쓰기는 딴 실 loop 밖에서 한다.** 예전에는 시뮬 루프 안에서 1280x720 PNG 를
    # 그대로 압축했는데, 그것만으로 판이 **12.9배** 느려졌다 (2026-09-30 실측: 카메라 셋,
    # 서가 도착이 0분 29초 → 6분 14초). 그때 GPU 사용률은 **0 %** 였다 — 렌더가 아니라
    # 압축이 병목이었다. 큐에 넣고 일꾼 실이 쓰면 시뮬은 기다리지 않는다.
    # 형식도 골라 둔다: jpg 는 png 보다 몇 배 빠르고, 편집 소스로는 충분하다.
    import queue as _queue, threading as _threading
    _fmt = os.environ.get("SIM_REC_FORMAT", "jpg").lower()
    _q = _queue.Queue(maxsize=int(os.environ.get("SIM_REC_QUEUE", "256")))
    _drops = [0]

    def _writer():
        while True:
            _item = _q.get()
            if _item is None:
                _q.task_done()
                return
            _path, _arr = _item
            try:
                if _fmt in ("jpg", "jpeg"):
                    _Image.fromarray(_arr).save(_path, "JPEG", quality=92)
                else:
                    _Image.fromarray(_arr).save(_path)
            except Exception:      # noqa: BLE001 — 한 장 못 써도 판은 계속 간다
                pass
            _q.task_done()

    _nw = int(os.environ.get("SIM_REC_WORKERS", "4"))
    _threads = [_threading.Thread(target=_writer, daemon=True) for _ in range(_nw)]
    for _th in _threads:
        _th.start()
    say(f"녹화 저장: {_fmt} · 일꾼 {_nw} 실 · 큐 {_q.maxsize} (시뮬 루프는 기다리지 않는다)")
    rec = {"n": 0, "saved": 0, "cams": _cams, "img": _Image, "np": _np,
           "out": _out,       # 종료 줄에서 쓴다 (예전에는 이 키가 없어 종료 때 KeyError 였다)
           "q": _q, "fmt": ("jpg" if _fmt in ("jpg", "jpeg") else "png"),
           "drops": _drops, "threads": _threads}
    say(f"녹화 시작 → {_out} ({args.record_every} 스텝마다 1장, 1280x720)")

# --- 시작 문: 카메라를 놓고 **원할 때** 시작한다 (`SIM_START_GATE`)
#
# 왜 필요한가: Isaac 이 뜨자마자 트레이가 미끄러져 오고 팔이 홈으로 간다. 그 사이에
# 카메라를 놓을 수가 없다 (2026-09-30 도윤님). 여기서 멈추면 **물리는 한 걸음도 안 간다** —
# `app.update()` 만 부르므로 화면은 살아 있고 뷰포트·카메라는 마음대로 움직일 수 있다.
# 트레이 이송과 홈 이동은 둘 다 이 아래 루프에서 돌기 때문에 여기서 잡으면 둘 다 멈춘다.
#
#   SIM_START_GATE=1            → /tmp/b1_go 를 기다린다
#   SIM_START_GATE=/경로/파일    → 그 파일을 기다린다
#   준비되면 다른 터미널에서:  touch /tmp/b1_go
_gate_file = os.environ.get("SIM_START_GATE", "").strip()
if _gate_file:
    if _gate_file in ("1", "true", "on", "yes"):
        _gate_file = "/tmp/b1_go"
    if os.path.exists(_gate_file):
        os.remove(_gate_file)          # 지난 판의 신호가 남아 있으면 바로 지나가 버린다
    say(f"[시작 문] 기다린다 — 카메라를 놓고 준비되면 다른 터미널에서:  touch {_gate_file}")
    _last = time.time()
    while app.is_running() and not os.path.exists(_gate_file):
        app.update()
        if time.time() - _last > 30.0:
            say(f"[시작 문] 아직 기다리는 중 ({_gate_file})")
            _last = time.time()
    # **파일을 지우지 않는다.** `book_scene` 의 이송도 같은 파일을 보고 있어서, 여기서
    # 지우면 트레이 쪽 문이 다시 닫혀 버린다 (2026-09-30 실측: 팔만 움직이고 트레이는 제자리).
    # 판이 끝나면 다음 판 시작할 때 지운다.
    say("[시작 문] 열렸다 — 시작한다")


def _gate_hold():
    """문이 닫혀 있으면 **물리를 한 걸음도 안 돌리고** 화면만 살려 둔다.

    `world.step()` 을 안 부르므로 트레이·팔·주행이 전부 그 자리에 선다. `app.update()` 는
    부르니 뷰포트는 살아 있어서 카메라를 옮기고 키를 잡을 수 있다 (2026-09-30 도윤님).

        touch /tmp/b1_go   → 돈다
        rm    /tmp/b1_go   → 그 자리에서 멈춘다 (다시 touch 하면 이어서 간다)

    **주의**: 작업(파지·삽입)이 도는 중에 멈추면 ROS 쪽 제한 시간은 벽시계로 흐른다.
    오래 세워 두면 `404`·`411` 이 난다. 카메라 잡는 것은 작업 사이나 시작 전에 할 것.
    """
    if not _gate_file:
        return
    if os.path.exists(_gate_file):
        _gate_hold.said = False
        return
    if not getattr(_gate_hold, "said", False):
        _gate_hold.said = True
        say(f"[시작 문] 멈춘다 — 다시 돌리려면:  touch {_gate_file}")
    while app.is_running() and not os.path.exists(_gate_file):
        app.update()
    say("[시작 문] 다시 돈다")
    _gate_hold.said = False


# --- 카메라 키프레임 재생 (`SIM_TIMELINE=1`)
#
# 레벨에 찍어 둔 카메라 키(USD time-sample)를 **시뮬 시간에 맞춰** 돌린다.
# 2026-09-30 실측으로 알게 된 것 셋:
#   ① `world.step(render=False)` 로는 타임라인이 아예 안 흐른다 (0.033 s 에서 멈춘다).
#   ② `render=True` 면 저절로 흐르는데 **실시간 기준**이라, 시뮬이 느려지면 카메라만 앞서 간다.
#   ③ 매 스텝 `set_current_time()` 을 주면 그 값을 정확히 따른다 (키도 그 시각으로 평가된다).
# 그래서 ③ 으로 시뮬 시간을 그대로 먹인다. 촬영 판에서만 켠다 — 기본은 꺼짐이라 동작 불변.
#
# **주의**: 타임라인은 스테이지의 `endTimeCode` 에서 처음으로 되돌아간다(루프). 12분 판을
# 찍으려면 레벨의 end time code 를 그만큼 길게 잡아야 한다 (tps 24 기준 12분 = 17280).
_drive_timeline = os.environ.get("SIM_TIMELINE", "0") != "0"
_tl = None
if _drive_timeline:
    import omni.timeline
    _tl = omni.timeline.get_timeline_interface()
    say("[타임라인] 카메라 키를 시뮬 시간에 맞춰 돌린다 [SIM_TIMELINE=1] "
        "— 스테이지 end time code 를 넘으면 처음으로 돌아간다")

t0 = time.time()
while app.is_running():
    _gate_hold()
    if _tl is not None:
        try:
            _tl.set_current_time(float(world.current_time))
        except Exception:      # noqa: BLE001 — 카메라가 안 움직여도 판은 계속 돈다
            _tl = None
            say("[타임라인] 시각을 못 준다 — 카메라 키 재생을 끈다")
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
            _any = False
            for _cam in rec["cams"]:
                _d = _cam["ann"].get_data()
                if _d is None or not getattr(_d, "size", 0):
                    continue
                _a = rec["np"].asarray(_d)[:, :, :3].astype("uint8").copy()
                _fp = os.path.join(_cam["dir"], f"f{rec['saved']:06d}.{rec['fmt']}")
                try:
                    rec["q"].put_nowait((_fp, _a))
                except Exception:      # noqa: BLE001 — 큐가 차면 그 장은 버린다
                    rec["drops"][0] += 1
                    if rec["drops"][0] in (1, 100, 1000):
                        say(f"녹화: 쓰기가 못 따라와 {rec['drops'][0]}장 버렸다 "
                            f"— REC_EVERY 를 올리거나 카메라를 줄일 것")
                _any = True
            if _any:
                rec["saved"] += 1
    if args.max_seconds and time.time() - t0 > args.max_seconds:
        say("max-seconds 도달 — 종료")
        break

if rec is not None:
    for _ in rec["threads"]:
        rec["q"].put(None)
    rec["q"].join()
    say(f"녹화 종료 — {rec['saved']}장 저장 (버림 {rec['drops'][0]}장) ({rec['out']})")
ros_bridge.shutdown(node)
app.close()
