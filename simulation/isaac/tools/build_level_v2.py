"""파지·배치 테스트에서 검증한 레벨 변경을 반영해 새 레벨 파일로 저장한다.

원본은 수정하지 않는다. 시뮬레이션을 돌리지 않고 USD 만 편집해 저장한다
(물리 스텝을 돌린 뒤 저장하면 흔들린 상태가 저장된다).

GPU PC:
    ~/isaacsim/python.sh ~/arm/isaac/build_level_v2.py \
        --src ~/Desktop/ing_library_env.usd --dst ~/Desktop/ing_library_env_v2.usd
"""
import argparse
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True)
ap.add_argument("--dst", required=True)
ap.add_argument("--robot-shift", type=float, default=0.15)
ap.add_argument("--shelf-scale-z", type=float, default=1.4)
ap.add_argument("--book-mass", type=float, default=0.5)
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage, save_stage

def say(m):
    sys.stderr.write(f"### {m}\n"); sys.stderr.flush()

R = "/World/ridgeback_franka"
SHELVES = "/World/bookshelves"
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
TARGET_SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
DECK_Z = 0.286
DIV_H, DIV_T, GAP = 0.07, 0.01, 0.003
SLOT1_FLOOR = 0.355                       # 원래 1번 칸 바닥 높이 (메쉬 실측)

open_stage(args.src)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
def aabb(path):
    cache.Clear()
    return np.array(compute_aabb(cache, path, include_children=True), float)

# 1) 로봇 배치 +Y (주행이 아니라 초기 배치 좌표)
shift = np.array([0.0, args.robot_shift, 0.0])
xf = SingleXFormPrim(R); p, q = xf.get_world_pose(); xf.set_world_pose(np.array(p) + shift, q)
say(f"로봇 배치 {np.round(p,3).tolist()} → {np.round(np.array(p)+shift,3).tolist()}")

# 2) 데크 위 책: 로봇과 같이 옮기고 데크에 붙인다, 질량 0.5kg
bb = aabb(BOOK)
drop = (DECK_Z + 0.001) - bb[2]
xf = SingleXFormPrim(BOOK); p, q = xf.get_world_pose()
xf.set_world_pose(np.array(p) + shift + np.array([0, 0, drop]), q)
UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(BOOK)).CreateMassAttr().Set(args.book_mass)
bb = aabb(BOOK)
say(f"책 이동 z {drop:+.3f}, 질량 {args.book_mass}kg, AABB {np.round(bb,3).tolist()}")

# 3) 선반 전부: z 스케일 + 정적 충돌체
count = 0
for shelf in stage.GetPrimAtPath(SHELVES).GetChildren():
    sx = SingleXFormPrim(str(shelf.GetPath()))
    sc = sx.get_local_scale()
    sx.set_local_scale(np.array([sc[0], sc[1], sc[2] * args.shelf_scale_z]))
    for m in Usd.PrimRange(shelf):
        if m.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(m)
            UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr().Set("none")
    count += 1
say(f"선반 {count}개: z ×{args.shelf_scale_z}, 충돌체 추가")

# 4) 칸막이 (정적 큐브)
UsdGeom.Scope.Define(stage, "/World/fixtures")
def divider(name, center, size):
    cube = UsdGeom.Cube.Define(stage, f"/World/fixtures/{name}")
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    cube.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.25)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

half_t = (bb[3] - bb[0]) / 2
length = (bb[4] - bb[1]) + 0.02
cy = (bb[1] + bb[4]) / 2
for side, cx in (("L", bb[0] - GAP - DIV_T / 2), ("R", bb[3] + GAP + DIV_T / 2)):
    divider(f"tray_divider_{side}", (cx, cy, DECK_Z + DIV_H / 2), (DIV_T, length, DIV_H))

link0 = np.array(SingleXFormPrim(R + "/panda_link0").get_world_pose()[0])
shelf_bb = aabb(TARGET_SHELF)
place_x = link0[0] - 0.10
place_cy = shelf_bb[1] + 0.03 + (bb[4] - bb[1]) / 2
floor_z = SLOT1_FLOOR * args.shelf_scale_z
for side, cx in (("L", place_x - half_t - GAP - DIV_T / 2), ("R", place_x + half_t + GAP + DIV_T / 2)):
    divider(f"shelf01_bookend_{side}", (cx, place_cy, floor_z + DIV_H / 2), (DIV_T, length, DIV_H))
say(f"칸막이: 데크 2개, 선반01 북엔드 2개 (배치 x {place_x:.3f} y {place_cy:.3f} 바닥 {floor_z:.3f})")

ok = save_stage(args.dst, save_and_reload_in_place=False)
say(f"저장 {args.dst} -> {ok}")
app.close()
