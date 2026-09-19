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
SHELF = [np.array([x, 0.5495, 0.3399]) for x in (-0.3497, -0.4297, -0.5097, -0.2697)]

# 자세 두 가지 (w, x, y, z)
#  - 내려다보기: 트레이에서 책등을 위에서 잡는다
#  - 눕혀 끼우기: 서가에 yaw +90° 로 밀어 넣는다
DOWN = np.array([0.0, 1.0, 0.0, 0.0])
INSERT = np.array([0.7071068, 0.0, 0.0, 0.7071068])


def solve(pos, quat):
    q, ok = lula.compute_inverse_kinematics(
        BOT.ee_frame, np.asarray(pos, float), np.asarray(quat, float))
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
say("서가 (꽂는 자세, yaw +90°) ← **관문**:")
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

app.close()
