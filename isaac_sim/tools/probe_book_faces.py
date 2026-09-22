"""데크 위 책을 여섯 방향에서 찍어 책등(제목 면)이 어느 쪽인지 확인한다."""
import argparse, sys
ap = argparse.ArgumentParser(); ap.add_argument("--usd", required=True); ap.add_argument("--out", default="/tmp/bookfaces")
args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import os, cv2, numpy as np
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
os.makedirs(args.out, exist_ok=True)
open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
bb = np.array(compute_aabb(create_bbox_cache(), BOOK, include_children=True), float)
c = (bb[:3] + bb[3:]) / 2
world = World(stage_units_in_meters=1.0)
views = {"from_+x": [0.45, 0, 0.05], "from_-x": [-0.45, 0, 0.05], "from_+y": [0, 0.45, 0.05],
         "from_-y": [0, -0.45, 0.05], "from_top": [0.02, 0.0, 0.5]}
cams = {}
for k, off in views.items():
    path = f"/World/bf_{k.replace('+','p').replace('-','m')}"
    cams[k] = Camera(prim_path=path, resolution=(480, 360))
    set_camera_view(eye=c + np.array(off), target=c, camera_prim_path=path)
world.reset()
for cam in cams.values(): cam.initialize()
for _ in range(20): world.step(render=True)
tiles = []
for k, cam in cams.items():
    im = cv2.cvtColor(cam.get_rgba()[:, :, :3], cv2.COLOR_RGB2BGR).copy()
    cv2.putText(im, k, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2); tiles.append(im)
tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(f"{args.out}/faces.jpg", np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])]))
sys.stderr.write(f"### book aabb {np.round(bb,3).tolist()}\n")
app.close()
