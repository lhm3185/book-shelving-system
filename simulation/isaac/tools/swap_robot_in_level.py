"""레벨의 로봇을 새 결합 로봇으로 바꿔 넣는다 — **팔 베이스를 같은 자리에 맞춰서**.

왜 베이스를 맞추나: 트레이 6칸·서가 4칸 좌표가 전부 `arm_base_link` 기준이다.
새 로봇의 **팔 베이스를 기존 팔 베이스와 같은 월드 위치**에 놓으면 **기존 좌표가 그대로 유효**하다.
아무 데나 놓고 좌표를 다시 재는 것보다 훨씬 빠르고, 게이트 1(IK 확인)이 바로 의미를 갖는다.

하는 일
    1) 기존 로봇의 팔 베이스(`panda_link0`) 월드 위치를 읽는다
    2) 기존 로봇을 **비활성화**한다 (지우지 않는다 — 되돌리기 위해)
    3) 새 로봇을 참조로 넣고, 그 팔 베이스가 1)의 자리에 오도록 옮긴다
    4) 결과를 새 USD 로 저장한다 (원본 레벨은 건드리지 않는다)

실행
    ISAAC_ENTRY=simulation/isaac/tools/swap_robot_in_level.py ./scripts/run_isaac_tool.sh \\
        --robot-usd ~/carter_m0609_handoff/carter_m0609.usd \\
        --out ~/Desktop/ing_library_env_m0609.usd
    # 그 다음
    ARM_ROBOT=m0609 SIM_USD=~/Desktop/ing_library_env_m0609.usd \\
        ISAAC_ENTRY=simulation/isaac/tools/check_reach.py ./scripts/run_isaac_tool.sh
"""
import argparse
import os
import sys

# tools/ → isaac/ → simulation/ → 저장소 루트 (네 번 올라간다)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ap = argparse.ArgumentParser()
ap.add_argument("--level", default="", help="비우면 SIM_USD 등 기본 규칙")
ap.add_argument("--robot-usd", required=True, help="AMR 담당이 준 결합 로봇 USD")
ap.add_argument("--robot-prim", default="", help="그 USD 안에서 가져올 prim (비우면 기본 prim)")
ap.add_argument("--old-robot", default="/World/ridgeback_franka")
ap.add_argument("--old-base", default="panda_link0", help="기존 로봇의 팔 베이스 링크")
ap.add_argument("--new-root", default="/World/carter_m0609", help="레벨에 넣을 때 쓸 경로")
ap.add_argument("--new-base", default="", help="새 로봇의 팔 베이스 (비우면 프로파일 값)")
ap.add_argument("--out", required=True)
ap.add_argument("--keep-old", choices=["off", "on"], default="off",
                help="off(기본): 기존 로봇을 비활성화. on: 둘 다 남겨 비교")
args = ap.parse_args()

sys.path.insert(0, os.path.join(REPO, "simulation", "isaac"))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac", "config"))

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.utils.stage import (add_reference_to_stage, get_current_stage,  # noqa: E402
                                       is_stage_loading, open_stage)

import world_loader  # noqa: E402
from robot_profiles import profile  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


new_base_rel = args.new_base or profile("m0609").base_link
level = world_loader.resolve_usd(args.level or None)
say(f"레벨 {level}")
say(f"새 로봇 {args.robot_usd} → {args.new_root} (팔 베이스 {new_base_rel})")

open_stage(level)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()


def world_pos(path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return None
    return np.array(
        UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0).ExtractTranslation(), float)


old_base_path = f"{args.old_robot}/{args.old_base}"
target = world_pos(old_base_path)
if target is None:
    say(f"기존 팔 베이스를 못 찾았다: {old_base_path}")
    app.close()
    sys.exit(1)
say(f"기존 팔 베이스 월드 위치 {np.round(target, 4).tolist()} ← 여기에 맞춘다")

# 기존 로봇은 지우지 않고 끈다 (되돌릴 수 있게)
if args.keep_old == "off":
    old = stage.GetPrimAtPath(args.old_robot)
    if old.IsValid():
        old.SetActive(False)
        say(f"기존 로봇 비활성화 {args.old_robot} (지우지 않았다 — SetActive(True) 로 되돌린다)")

if not os.path.exists(os.path.expanduser(args.robot_usd)):
    say(f"로봇 USD 가 없다: {args.robot_usd}")
    app.close()
    sys.exit(1)
add_reference_to_stage(os.path.expanduser(args.robot_usd), args.new_root)
app.update()
while is_stage_loading():
    app.update()

new_base_path = f"{args.new_root}/{new_base_rel}"
here = world_pos(new_base_path)
if here is None:
    say(f"새 로봇의 팔 베이스를 못 찾았다: {new_base_path}")
    say("  → --new-base 로 알려주거나 robot_profiles.py 의 base_link 를 고칠 것")
    app.close()
    sys.exit(1)
say(f"새 팔 베이스 현재 위치 {np.round(here, 4).tolist()}")

shift = target - here
xf = UsdGeom.Xformable(stage.GetPrimAtPath(args.new_root))
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in shift]))
app.update()

after = world_pos(new_base_path)
err = float(np.linalg.norm(after - target))
say(f"이동 {np.round(shift, 4).tolist()} → 맞춘 뒤 오차 {err * 1000:.2f} mm")
if err > 0.001:
    say("  ** 오차가 크다 — 새 로봇 안에 추가 변환이 있는지 확인할 것 **")

out = os.path.expanduser(args.out)
stage.Export(out)
say(f"저장 {out}")
say("")
say("다음:")
say(f"  ARM_ROBOT=m0609 SIM_USD={out} \\")
say("    ISAAC_ENTRY=simulation/isaac/tools/check_reach.py ./scripts/run_isaac_tool.sh")

app.close()
