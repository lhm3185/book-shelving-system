"""게이트 1 자동화 — **기존 좌표에 팔이 닿는가**를 IK 로만 확인한다.

왜: 로봇을 M0609 로 바꾸면 "닿는지" 부터 막힐 수 있다. 팔을 실제로 움직여 보면 오래 걸리고
    실패 원인도 섞인다(충돌·그리퍼·계획). **IK 해가 있는지만** 먼저 갈라 본다.

검사 대상 (전부 `arm_base_link` 기준, `book_profiles.yaml` 과 같은 값)
    - 트레이 6칸: 책을 집는 자세 (위에서 내려다봄)
    - 서가 4칸: 책을 꽂는 자세 (yaw +90°, 책을 눕혀 끼움)

**서가 4칸이 진짜 관문이다.** 6축에서 "책 눕혀 끼우는 손목 자세" 해가 없으면 삽입 방식을
다시 설계해야 하고, 그건 하루 이틀에 못 한다.

실행
    ISAAC_ENTRY=simulation/isaac/tools/check_reach.py ./scripts/run_isaac_tool.sh
    ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/check_reach.py ./scripts/run_isaac_tool.sh
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="", help="비우면 run_isaac_sim.sh 와 같은 규칙(SIM_USD 등)")
ap.add_argument("--approach", type=float, default=0.10,
                help="파지 전 접근 높이 (m). 이 높이에서도 닿는지 같이 본다")
# 로봇이 바뀌면 **같은 선반 단이라도 팔 기준 높이가 달라진다** (M0609 는 베이스가 37cm 높다).
# 계약값(0.3399)만 보고 판단하지 않도록 단 높이를 바꿔가며 볼 수 있게 한다.
ap.add_argument("--shelf-z", type=float, default=0.3399,
                help="서가 칸의 팔 기준 높이 (m) = 선반판 윗면 + 책높이/2 - 팔베이스 z")
ap.add_argument("--shelf-x-shift", type=float, default=0.0,
                help="서가 칸 x 를 통째로 이 값만큼 옮겨 본다 (닿는 대역 안으로 밀어 넣을 때)")
ap.add_argument("--scan-x", action="store_true",
                help="서가 칸의 x 대역을 훑는다 (칸 좌표를 다시 잡을 때)")
args = ap.parse_args()

# tools/ → isaac/ → simulation/ → 저장소 루트 (네 번 올라간다)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac"))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac", "config"))

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, interface_config_loader  # noqa: E402

import world_loader  # noqa: E402
from robot_profiles import profile  # noqa: E402

BOT = profile()


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


usd = world_loader.resolve_usd(args.usd or None)
say(f"로봇 프로파일 '{BOT.name}' / USD {usd}")
open_stage(usd)
app.update()
while is_stage_loading():
    app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

base_path = f"{BOT.root}/{BOT.base_link}"
bp = SingleXFormPrim(base_path)
if not bp.is_valid():
    say(f"베이스 링크를 못 찾았다: {base_path} — 프로파일의 base_link 를 확인할 것")
    app.close()
    sys.exit(1)
l0p, l0q = bp.get_world_pose()

if BOT.lula[0] == "supported":
    cfg = interface_config_loader.load_supported_lula_kinematics_solver_config(BOT.lula[1])
else:
    for f in BOT.lula[1:]:
        if not os.path.exists(f):
            say(f"Lula 파일이 없다: {f}")
            app.close()
            sys.exit(1)
    cfg = {"robot_description_path": BOT.lula[1], "urdf_path": BOT.lula[2]}
lula = LulaKinematicsSolver(**cfg)
lula.set_robot_base_pose(np.asarray(l0p, float), np.asarray(l0q, float))
say(f"Lula 로드 완료, 엔드이펙터 프레임 '{BOT.ee_frame}'")

conf = yaml.safe_load(open(os.path.join(
    REPO, "ros2_ws", "src", "shelving_manipulation", "config", "book_profiles.yaml")))
tray = [(s["index"], np.array(s["center"], float)) for s in conf["tray"]["slots"]]
SHELF = [np.array([x + args.shelf_x_shift, 0.5495, args.shelf_z])
         for x in (-0.3497, -0.4297, -0.5097, -0.2697)]

# 자세 두 가지 (w, x, y, z)
#  - 내려다보기: 트레이에서 책등을 위에서 잡는다
#  - 눕혀 끼우기: 서가에 yaw +90° 로 밀어 넣는다
DOWN = np.array([0.0, 1.0, 0.0, 0.0])
INSERT = np.array([0.7071068, 0.0, 0.0, 0.7071068])


def quat_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    w1, x1, y1, z1 = [float(v) for v in a]
    w2, x2, y2, z2 = [float(v) for v in b]
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


BASE_P = np.asarray(l0p, float)
BASE_Q = np.asarray(l0q, float)
BASE_R = quat_to_R(BASE_Q)


def solve(pos, quat):
    """좌표는 **arm_base_link 기준**으로 받는다 (book_profiles.yaml 과 같은 계약).
    Lula 는 **월드 기준**을 받으므로 여기서 변환한다. 이 변환을 빼먹으면 로봇을 옮겼을 때
    전부 '해 없음' 으로 나와 잘못된 판단을 하게 된다 (2026-09-19 실제로 그럴 뻔했다)."""
    wp = BASE_P + BASE_R @ np.asarray(pos, float)
    wq = quat_mul(BASE_Q, np.asarray(quat, float))
    q, ok = lula.compute_inverse_kinematics(BOT.ee_frame, wp, wq)
    return bool(ok), q


rows = []
say("")
say("트레이 (집는 자세):")
for idx, c in tray:
    ok_at, _ = solve(c, DOWN)
    ok_up, _ = solve(c + np.array([0, 0, args.approach]), DOWN)
    rows.append(ok_at and ok_up)
    say(f"  칸 {idx}  {np.round(c, 4).tolist()}   파지 {'O' if ok_at else 'X'}"
        f"   접근({args.approach:.2f}m 위) {'O' if ok_up else 'X'}")

say("")
say(f"서가 (꽂는 자세, yaw +90°, 팔기준 z {args.shelf_z:+.4f}) ← **관문**:")
shelf_ok = []
for c in SHELF:
    ok_at, _ = solve(c, INSERT)
    ok_pre, _ = solve(c - np.array([0, args.approach, 0]), INSERT)
    shelf_ok.append(ok_at and ok_pre)
    say(f"  {np.round(c, 4).tolist()}   삽입 {'O' if ok_at else 'X'}"
        f"   직전({args.approach:.2f}m 앞) {'O' if ok_pre else 'X'}")

say("")
n_tray, n_shelf = sum(rows), sum(shelf_ok)
say(f"트레이 {n_tray}/{len(rows)}   서가 {n_shelf}/{len(shelf_ok)}")
if n_shelf == len(shelf_ok) and n_tray == len(rows):
    say("**게이트 1 통과** — 기존 좌표에 전부 해가 있다. 게이트 2(삽입 1권)로 넘어갈 것")
elif n_shelf == 0:
    say("**서가 자세에 해가 없다** — 6축에서 책을 눕혀 끼우는 손목 자세가 안 나온다.")
    say("  삽입 방식 재설계가 필요하고 하루 이틀에 못 한다 → 분리 시연 판단 (ASSET_RECEIPT_CHECKLIST 참조)")
else:
    say("**일부만 닿는다** — 좌표를 바꾸기 전에 **로봇 배치를 옮겨** 보는 쪽이 빠르다")

if args.scan_x:
    # 어느 x 대역이 닿는가. 칸 좌표는 **팔 기준 계약값**이라 로봇을 옮겨도 안 바뀐다.
    # 단 높이를 실제 선반으로 내리면 닿는 대역이 좁아지므로, 칸을 다시 잡으려면 이게 필요하다.
    say("")
    say(f"x 대역 훑기 (y 0.5495, z {args.shelf_z:+.4f}, 삽입+직전 둘 다 성립해야 O):")
    band = []
    x = -0.18
    while x > -0.70:
        ok_at, _ = solve([x, 0.5495, args.shelf_z], INSERT)
        ok_pre, _ = solve([x, 0.5495 - args.approach, args.shelf_z], INSERT)
        band.append((round(x, 3), ok_at and ok_pre))
        say(f"  x {x:+.3f}  {'O' if ok_at and ok_pre else 'X'}")
        x -= 0.02
    runs, cur = [], []
    for xv, g in band:
        if g:
            cur.append(xv)
        elif cur:
            runs.append(cur); cur = []
    if cur:
        runs.append(cur)
    if runs:
        b = max(runs, key=len)
        say(f"**연속 가능 대역 x {min(b):+.3f} ~ {max(b):+.3f} (폭 {max(b)-min(b):.3f} m)**")
    else:
        say("**연속 대역 없음**")

app.close()
