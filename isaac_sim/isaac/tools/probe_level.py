"""팀 레벨(ing_library_env.usd)을 헤드리스로 열어 구조를 조사하고 캡처한다.

GPU PC 에서 실행:
    ~/isaacsim/python.sh ~/arm/isaac/probe_level.py --usd ~/Desktop/ing_library_env.usd --out /tmp/probe
"""
import argparse
import json
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", default="/tmp/probe")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import os
import cv2
import numpy as np
import omni
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera

os.makedirs(args.out, exist_ok=True)
def say(msg):
    sys.stderr.write(f"### {msg}\n"); sys.stderr.flush()

open_stage(args.usd)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
say(f"stage 열림 upAxis={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)}")

cache = create_bbox_cache()
report = {"articulations": [], "joints": [], "robot": {}, "shelves": [], "books": [], "physics_scene": []}

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if prim.IsA(UsdPhysics.Scene):
        report["physics_scene"].append(path)
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        report["articulations"].append(path)
    if path.startswith("/World/ridgeback_franka") and prim.IsA(UsdPhysics.Joint):
        report["joints"].append({"path": path, "type": prim.GetTypeName()})

def aabb(path):
    b = compute_aabb(cache, path, include_children=True)
    return [round(float(v), 3) for v in b]

robot = stage.GetPrimAtPath("/World/ridgeback_franka")
refs = []
for spec in robot.GetPrimStack():
    refs += [r.assetPath for r in spec.referenceList.GetAddedOrExplicitItems()]
    refs += [p.assetPath for p in spec.payloadList.GetAddedOrExplicitItems()]
report["robot"] = {"refs": refs, "aabb": aabb("/World/ridgeback_franka"),
                   "children": [str(c.GetPath()) for c in robot.GetChildren()][:40]}

def phys_flags(prim):
    flags = []
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI): flags.append("RIGID@" + str(p.GetPath()).split("/")[-1])
        if p.HasAPI(UsdPhysics.CollisionAPI): flags.append("COLL")
        if p.HasAPI(UsdPhysics.ArticulationRootAPI): flags.append("ARTIC")
    return sorted(set(flags))

for c in stage.GetPrimAtPath("/World/bookshelves").GetChildren():
    report["shelves"].append({"path": str(c.GetPath()), "aabb": aabb(str(c.GetPath())), "phys": phys_flags(c)})
for c in stage.GetPrimAtPath("/World/books").GetChildren():
    report["books"].append({"path": str(c.GetPath()), "aabb": aabb(str(c.GetPath())), "phys": phys_flags(c)})

json.dump(report, open(f"{args.out}/report.json", "w"), indent=1, ensure_ascii=False)
say(f"articulation 루트: {report['articulations']}")
say(f"robot aabb: {report['robot']['aabb']}  refs: {report['robot']['refs']}")
say(f"shelves {len(report['shelves'])} books {len(report['books'])} physics_scene {report['physics_scene']}")

world = World(stage_units_in_meters=1.0)
cams = {}
lo = np.array(report["robot"]["aabb"][:3]); hi = np.array(report["robot"]["aabb"][3:])
center = (lo + hi) / 2
views = {
    "robot": (center + np.array([-2.5, -2.5, 2.0]), center),
    "overview": (center + np.array([-6.0, -6.0, 7.0]), center),
}
for name, (eye, target) in views.items():
    path = f"/World/probe_cam_{name}"
    cams[name] = Camera(prim_path=path, resolution=(1280, 720))
    set_camera_view(eye=eye, target=target, camera_prim_path=path)

world.reset()
for cam in cams.values():
    cam.initialize()

# 물리 스텝을 조금 돌려 떨어지는 물체가 있는지도 본다
for _ in range(60):
    world.step(render=True)

for name, cam in cams.items():
    rgba = cam.get_rgba()
    if rgba is None or rgba.size == 0:
        say(f"{name}: 이미지 없음"); continue
    cv2.imwrite(f"{args.out}/{name}.png", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))
    say(f"{name}: 저장 {rgba.shape}")

# 스텝 후 책 위치가 바뀌었는지 (낙하 여부)
moved = []
cache2 = create_bbox_cache()
for b in report["books"][:80]:
    after = [round(float(v), 3) for v in compute_aabb(cache2, b["path"], include_children=True)]
    if abs(after[2] - b["aabb"][2]) > 0.02:
        moved.append((b["path"].split("/")[-1], b["aabb"][2], after[2]))
say(f"60스텝 후 높이가 바뀐 책: {moved[:6]} (총 {len(moved)})")
say("PROBE_DONE")
app.close()
