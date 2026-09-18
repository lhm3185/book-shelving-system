"""레벨에서 팔 원점·선반 판 높이·책 상태를 정밀하게 뽑고, 선반 후보 지점 IK 도달 여부를 본다."""
import argparse, json, sys
ap = argparse.ArgumentParser(); ap.add_argument("--usd", required=True); ap.add_argument("--out", default="/tmp/probe2")
args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import os
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver

os.makedirs(args.out, exist_ok=True)
def say(m): sys.stderr.write(f"### {m}\n"); sys.stderr.flush()

open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
R = "/World/ridgeback_franka"
SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"

# 선반 판 높이: 메쉬 꼭짓점 z 를 월드로 옮겨 모인 높이를 찾는다
zs = []
xc = UsdGeom.XformCache()
for p in Usd.PrimRange(stage.GetPrimAtPath(SHELF)):
    if p.IsA(UsdGeom.Mesh):
        m = xc.GetLocalToWorldTransform(p)
        for v in UsdGeom.Mesh(p).GetPointsAttr().Get():
            zs.append(round(m.Transform(v)[2], 3))
vals, counts = np.unique(np.array(zs), return_counts=True)
levels = [(float(v), int(c)) for v, c in zip(vals, counts) if c >= 4]
say(f"선반 꼭짓점 z 레벨(개수>=4): {levels}")

# 책 물리 속성
bp = stage.GetPrimAtPath(BOOK)
for p in Usd.PrimRange(bp):
    if p.HasAPI(UsdPhysics.MassAPI):
        say(f"책 질량 {UsdPhysics.MassAPI(p).GetMassAttr().Get()} 밀도 {UsdPhysics.MassAPI(p).GetDensityAttr().Get()} @ {p.GetPath()}")
    if p.HasAPI(UsdPhysics.MeshCollisionAPI):
        say(f"책 충돌 근사 {UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()} @ {p.GetPath()}")

world = World(stage_units_in_meters=1.0)
robot = SingleArticulation(prim_path=R, name="rf")
world.reset()
robot.initialize()
for _ in range(30): world.step(render=False)

say(f"dof: {robot.dof_names}")
say(f"관절값: {np.round(robot.get_joint_positions(), 3).tolist()}")
link0 = SingleXFormPrim(R + "/panda_link0")
l0p, l0q = link0.get_world_pose()
say(f"panda_link0 월드 위치 {np.round(l0p,3).tolist()} 쿼터니언(wxyz) {np.round(l0q,3).tolist()}")
ee = SingleXFormPrim(R + "/panda_hand")
say(f"panda_hand 위치 {np.round(ee.get_world_pose()[0],3).tolist()}")
bk = SingleXFormPrim(BOOK)
say(f"책 원점 위치 {np.round(bk.get_world_pose()[0],3).tolist()}")

cfg = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
lula = LulaKinematicsSolver(**cfg)
lula.set_robot_base_pose(l0p, l0q)
ik = ArticulationKinematicsSolver(robot, lula, "right_gripper")
p, rot = ik.compute_end_effector_pose()
say(f"FK right_gripper 위치 {np.round(p,3).tolist()}")

# 후보 목표: 아래 방향 파지 자세 (그리퍼 z 가 -Z)
down = np.array([0.0, 1.0, 0.0, 0.0])  # Franka 예제의 하향 자세 (wxyz)
tests = {}
for name, tgt in {
    "책 위 10cm": [0, 0, 0],   # 아래에서 채움
}.items():
    pass
book_top = np.array(bk.get_world_pose()[0])
cands = {"book_above": [book_top[0], book_top[1], 0.60]}
for z in [lv for lv, _ in levels if 0.25 < lv < 1.3]:
    cands[f"shelf_front_z{z}"] = [2.57, -2.45, z + 0.22]
for name, tgt in cands.items():
    act, ok = ik.compute_inverse_kinematics(np.array(tgt, float), down, position_tolerance=0.01, orientation_tolerance=0.1)
    tests[name] = {"target": [round(v, 3) for v in tgt], "ok": bool(ok)}
    say(f"IK {name} {np.round(tgt,3).tolist()} -> {ok}")
json.dump({"levels": levels, "ik": tests, "link0": [l0p.tolist(), l0q.tolist()]}, open(f"{args.out}/reach.json", "w"), indent=1)
say("DONE")
app.close()
