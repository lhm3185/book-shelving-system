"""책 메쉬 꼭짓점을 월드로 옮겨 여섯 면 중 어디에 몰려 있는지(=둥근 책등) 센다."""
import argparse, sys
ap = argparse.ArgumentParser(); ap.add_argument("--usd", required=True); args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from pxr import Usd, UsdGeom
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
xc = UsdGeom.XformCache()
pts = []
for p in Usd.PrimRange(stage.GetPrimAtPath(BOOK)):
    if p.IsA(UsdGeom.Mesh):
        m = xc.GetLocalToWorldTransform(p)
        for v in UsdGeom.Mesh(p).GetPointsAttr().Get():
            w = m.Transform(v); pts.append([w[0], w[1], w[2]])
pts = np.array(pts); lo, hi = pts.min(0), pts.max(0); size = hi - lo
say = lambda s: (sys.stderr.write(f"### {s}\n"), sys.stderr.flush())
say(f"꼭짓점 {len(pts)}개, 크기 {np.round(size,3).tolist()}")
for ax, name in enumerate("xyz"):
    band = size[ax] * 0.12
    n_lo = int((pts[:, ax] < lo[ax] + band).sum()); n_hi = int((pts[:, ax] > hi[ax] - band).sum())
    say(f"{name}축: -{name}면 근처 {n_lo}개 / +{name}면 근처 {n_hi}개")
# 둥근 책등이면 그 면 근처 단면이 곡선 → 가장자리 쪽 다른 두 축 좌표가 넓게 퍼진다
app.close()
