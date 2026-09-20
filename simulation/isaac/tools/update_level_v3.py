"""v2 레벨에서 선반01 북엔드를 '책등이 로봇 쪽을 보도록 세워 꽂기'에 맞게 옮겨 v3 로 저장한다.

책을 도서관식으로 세우면 선반 안 깊이 방향으로 차지하는 길이가 책 폭(16.3cm)이 된다.
북엔드를 그 발자국에 맞추고, 책등 뒤에서 미는 손바닥이 걸리지 않게 앞쪽 2cm 를 비운다.
"""
import argparse, sys
ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True); ap.add_argument("--dst", required=True)
ap.add_argument("--place-dx", type=float, default=-0.35,
                help="넣는 위치 x = 팔 원점 x + 이 값. −10cm 는 팔꿈치가 한계까지 접혀 관절이 뒤집혔다(IK 비교로 −35cm 선택)")
args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from pxr import Gf, UsdGeom
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage, save_stage
say = lambda m: (sys.stderr.write(f"### {m}\n"), sys.stderr.flush())

open_stage(args.src); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
cache = create_bbox_cache()
bb = np.array(compute_aabb(cache, BOOK, include_children=True), float)
sh = np.array(compute_aabb(cache, SHELF, include_children=True), float)
width = bb[5] - bb[2]                      # 책 폭 (책등→앞마구리), 데크에서는 세로
y_front = sh[1]
spine_y = y_front + 0.02                   # 책등이 선반 앞면에서 2cm 안쪽
y0 = spine_y + 0.02                        # 북엔드 앞끝: 손바닥이 걸리지 않게 2cm 더 안쪽
y1 = spine_y + width
from isaacsim.core.prims import SingleXFormPrim
l0x = float(SingleXFormPrim("/World/ridgeback_franka/panda_link0").get_world_pose()[0][0])
place_x = l0x + args.place_dx
half_t = (bb[3] - bb[0]) / 2
xs = {"L": place_x - half_t - 0.003 - 0.005, "R": place_x + half_t + 0.003 + 0.005}
for side in ("L", "R"):
    prim = stage.GetPrimAtPath(f"/World/fixtures/shelf01_bookend_{side}")
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops["xformOp:translate"].Get(); s = ops["xformOp:scale"].Get()
    ops["xformOp:translate"].Set(Gf.Vec3d(xs[side], (y0 + y1) / 2, t[2]))
    ops["xformOp:scale"].Set(Gf.Vec3f(s[0], y1 - y0, s[2]))
    say(f"북엔드 {side}: x {t[0]:.3f} → {xs[side]:.3f}, y {t[1]:.3f} → {(y0+y1)/2:.3f}, 길이 {s[1]:.3f} → {y1-y0:.3f}")
say(f"저장 {args.dst} -> {save_stage(args.dst, save_and_reload_in_place=False)}")
app.close()
