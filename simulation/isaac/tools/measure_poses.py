"""`home`·`stow` 관절값을 **실측**한다 — `arm_m0609.yaml` 의 자리값(전부 0)을 채우기 위해.

무엇을 재나
    home : 트레이를 내려다보는 자세. 매번 집으러 갈 때 크게 돌지 않고, 손목 카메라가 트레이를 본다
    stow : 주행 자세. 팔을 접어 **받침판 발자국 안**에 넣는다 (주행 중 서가에 안 닿게)

어떻게
    home  트레이 가운데 위 `--home-above` m 지점을 내려다보는 IK 해
    stow  관절값 후보를 훑어, **판 밖으로 안 나가고 가장 낮은** 자세를 고른다

자기검증 (웹 클로드 v19 회신 §1-2)
    ① 왕복 자기일치  ② 오답 주입  ③ 0 가정 깨기(로봇이 원점이 아님)

실행
    ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/measure_poses.py \\
        ./scripts/run_isaac_tool.sh --usd <레벨>.usd
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="")
ap.add_argument("--home-above", type=float, default=0.30, help="트레이 위 몇 m 를 볼 것인가")
args = ap.parse_args()

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac"))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac", "config"))

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from pxr import UsdGeom  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402
from isaacsim.robot_motion.motion_generation import (LulaKinematicsSolver,  # noqa: E402
                                                     interface_config_loader)

import world_loader  # noqa: E402
from robot_profiles import profile  # noqa: E402

BOT = profile()


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


usd = world_loader.resolve_usd(args.usd or None)
say(f"프로파일 '{BOT.name}' / {usd}")
open_stage(usd)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
World(stage_units_in_meters=1.0).reset()

bl = stage.GetPrimAtPath(f"{BOT.root}/{BOT.base_link}")
if not bl.IsValid():
    say(f"팔 베이스를 못 찾음: {BOT.root}/{BOT.base_link}")
    app.close()
    sys.exit(1)
M = UsdGeom.Xformable(bl).ComputeLocalToWorldTransform(0)
P = np.array(M.ExtractTranslation(), float)
_q = M.ExtractRotationQuat()
BQ = np.array([_q.GetReal(), *_q.GetImaginary()])


def qR(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


BR = qR(BQ)

say("=== 자기검증 ===")
say(f"③ 0 가정 깨기: 팔 베이스가 원점에서 {float(np.linalg.norm(P)):.2f} m 떨어져 있다")
_t = np.array([-0.45, 0.08, 0.11])
_err = float(np.linalg.norm(BR.T @ ((P + BR @ _t) - P) - _t))
say(f"① 왕복 자기일치 오차 {_err:.6f} m {'OK' if _err < 1e-6 else '**실패**'}")
if _err >= 1e-6:
    app.close()
    sys.exit(1)

if BOT.lula[0] == "supported":
    cfg = interface_config_loader.load_supported_lula_kinematics_solver_config(BOT.lula[1])
else:
    cfg = {"robot_description_path": BOT.lula[1], "urdf_path": BOT.lula[2]}
lula = LulaKinematicsSolver(**cfg)
lula.set_robot_base_pose(P, BQ)
DOWN = qmul(BQ, np.array([0.0, 1.0, 0.0, 0.0]))


def ik(rel, quat=None):
    q, ok = lula.compute_inverse_kinematics(
        BOT.ee_frame, P + BR @ np.asarray(rel, float), quat if quat is not None else DOWN)
    return (np.asarray(q, float) if ok else None)


if ik([-0.45, 2.0, 0.11]) is not None:
    say("② 오답 주입: 2 m 밖에 해가 나온다 — **도구를 믿을 수 없다**")
    app.close()
    sys.exit(1)
say("② 오답 주입: 2 m 밖 좌표는 해 없음 OK")
say("")

conf = yaml.safe_load(open(os.path.join(
    REPO, "ros2_ws", "src", "shelving_manipulation", "config", "book_profiles.yaml")))
slots = np.array([s["center"] for s in conf["tray"]["slots"]], float)
mid = slots.mean(axis=0)
say(f"트레이 중앙 (팔 기준) {np.round(mid, 4).tolist()}")

# --- home : 트레이 중앙 위를 내려다본다
target = mid + np.array([0.0, 0.0, args.home_above])
qh = ik(target)
if qh is None:
    say(f"home 해 없음 — --home-above 를 줄여 볼 것 (지금 {args.home_above})")
else:
    say(f"**home** (트레이 중앙 {args.home_above:.2f} m 위)")
    say(f"  {np.round(qh[:BOT.dof], 4).tolist()}")

# --- stow : 판 발자국 안에 들어가는 가장 낮은 자세
cache.Clear()
plate = np.array(compute_aabb(cache, f"{BOT.root}/Cube", include_children=True), float)
say("")
say(f"받침판 x {plate[0]:+.3f}~{plate[3]:+.3f}  y {plate[1]:+.3f}~{plate[4]:+.3f}")

art_root = f"{BOT.root}/chassis_link"
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
robot = SingleArticulation(art_root)
robot.initialize()
names = list(robot.dof_names)
idx = [names.index(n) for n in BOT.arm_joints]
world = World.instance()

best = None
for j2 in (-1.2, -1.4, -1.6):
    for j3 in (1.8, 2.0, 2.2):
        for j5 in (0.8, 1.0, 1.2):
            cand = np.zeros(BOT.dof)
            cand[1], cand[2], cand[4] = j2, j3, j5
            full = np.array(robot.get_joint_positions(), float)
            full[idx] = cand
            robot.apply_action(ArticulationAction(
                joint_positions=full, joint_indices=np.arange(len(names))))
            for _ in range(60):
                world.step(render=False)
            cache.Clear()
            arm_bb = np.array(compute_aabb(cache, f"{BOT.root}/m0609", include_children=True), float)
            inside = (plate[0] - 0.02 <= arm_bb[0] and arm_bb[3] <= plate[3] + 0.02
                      and plate[1] - 0.02 <= arm_bb[1] and arm_bb[4] <= plate[4] + 0.02)
            top = float(arm_bb[5])
            if inside and (best is None or top < best[0]):
                best = (top, cand.copy())

if best is None:
    say("**stow 후보를 못 찾음** — 판 발자국 안에 들어가는 자세가 없다. 범위를 넓혀 볼 것")
else:
    say(f"**stow** (판 안, 최고점 z {best[0]:.3f} = 판 위 {best[0]-plate[5]:.3f} m)")
    say(f"  {np.round(best[1], 4).tolist()}")

say("")
say("→ 위 두 줄을 simulation/isaac/config/arm_m0609.yaml 의 poses 에 넣는다")
app.close()
