"""서가를 정면에서 찍어 **깊이 영상 + 서가 bbox + 정답 상태**를 저장한다 (Isaac Sim 5.1.0).

왜: 비전의 빈 칸 판정(`target_detector.TargetDetector`)은 "서가 bbox 를 5단으로 나눠 단마다 깊이 중앙값을 읽고,
가장 가까운 단보다 `depth_margin` 이상 먼 단을 비었다고 본다". 이 판정이 맞는지 보려면
**서가 bbox 와 실제 빈 칸**이 필요한데, Isaac 에서는 둘 다 정답으로 얻을 수 있다.

산출물 (`--out` 폴더)
    shelf_XX.npz   depth(m), bbox(x0,y0,x1,y1), fx fy cx cy, 서가 prim 경로
    shelf_XX.jpg   같은 시점 RGB (눈으로 확인용)
    truth.json     단별 실제 책 유무 (Isaac AABB 로 계산한 정답)

실행 (GPU PC)
    ISAAC_ENTRY=isaac_sim/isaac/tools/shelf_gap_probe.py ./scripts/run_isaac_tool.sh --views 6 --out ~/shelf_probe
"""
import argparse
import json
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="")
ap.add_argument("--shelf", default="/World/bookshelves",
                help="서가 묶음 prim. 아래 자식 하나를 골라 찍는다")
ap.add_argument("--shelf-index", type=int, default=-1, help="-1 이면 로봇 앞 서가를 자동 선택")
ap.add_argument("--views", type=int, default=6)
ap.add_argument("--dist", type=float, nargs=2, default=[0.9, 1.15],
                help="서가 앞면에서의 촬영 거리 범위 (m)")
ap.add_argument("--out", default=os.path.expanduser("~/shelf_probe"))
ap.add_argument("--res", type=int, default=640)
ap.add_argument("--hfov", type=float, default=90.5)
ap.add_argument("--rows", type=int, default=5, help="정답 계산에 쓸 단 개수 (비전 기본값과 같게)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.semantics import add_labels  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


usd = args.usd or _paths.default_usd()
open_stage(usd)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
os.makedirs(args.out, exist_ok=True)


def aabb(path):
    cache.Clear()
    return np.array(compute_aabb(cache, path, include_children=True), float)


shelves = [str(c.GetPath()) for c in stage.GetPrimAtPath(args.shelf).GetChildren()]
if not shelves:
    say(f"서가 prim 이 없다: {args.shelf}")
    app.close()
    sys.exit(1)
if args.shelf_index >= 0:
    target_shelf = shelves[args.shelf_index % len(shelves)]
else:
    robot = aabb("/World/ridgeback_franka")
    rc = (robot[:3] + robot[3:]) / 2
    target_shelf = min(shelves, key=lambda s: float(np.linalg.norm(((aabb(s)[:3] + aabb(s)[3:]) / 2)[:2] - rc[:2])))
say(f"서가 {len(shelves)}개 중 대상: {target_shelf}")

sb = aabb(target_shelf)
say(f"서가 AABB 크기 {np.round(sb[3:] - sb[:3], 3).tolist()}")

# 라벨: 서가와 책을 따로 붙여 bbox 를 정답으로 받는다
add_labels(stage.GetPrimAtPath(target_shelf), labels=["shelf"], instance_name="class")
books = []
lib = stage.GetPrimAtPath("/World/books")
if lib.IsValid():
    for c in lib.GetChildren():
        if not c.IsActive():
            continue
        b = aabb(str(c.GetPath()))
        mid = (b[:3] + b[3:]) / 2
        if (sb[0] - 0.05 <= mid[0] <= sb[3] + 0.05 and sb[1] - 0.05 <= mid[1] <= sb[4] + 0.05
                and sb[2] - 0.05 <= mid[2] <= sb[5] + 0.05):
            books.append((str(c.GetPath()), mid, b))

# 정답: 서가를 위에서부터 args.rows 단으로 나눠 각 단에 책이 있는지
z0, z1 = float(sb[2]), float(sb[5])
row_h = (z1 - z0) / args.rows
truth = []
for r in range(args.rows):                      # 0 = 맨 위 (비전 코드와 같은 순서)
    top = z1 - r * row_h
    bot = z1 - (r + 1) * row_h
    n = sum(1 for _, mid, _ in books if bot <= mid[2] < top)
    truth.append({"row": r, "z_top": round(top, 3), "z_bottom": round(bot, 3),
                  "books": n, "empty": n == 0})
say("정답 (위→아래): " + ", ".join(f"{t['row']}:{'빈칸' if t['empty'] else str(t['books']) + '권'}" for t in truth))

cam_path = "/World/shelf_probe_cam"
cam = UsdGeom.Camera.Define(stage, cam_path)
cam.CreateFocalLengthAttr(18.0)
ap_mm = 2.0 * 18.0 * np.tan(np.radians(args.hfov) / 2.0)
cam.CreateHorizontalApertureAttr(float(ap_mm))
cam.CreateVerticalApertureAttr(float(ap_mm))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
cam_prim = SingleXFormPrim(cam_path)
key = UsdLux.SphereLight.Define(stage, "/World/shelf_probe_key")
key.CreateRadiusAttr(0.4)
key.CreateIntensityAttr(40000.0)
key_prim = SingleXFormPrim("/World/shelf_probe_key")

rp = rep.create.render_product(cam_path, (args.res, args.res))
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
ann_depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
ann_box = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
for a in (ann_rgb, ann_depth, ann_box):
    a.attach(rp)

fx = (args.res / 2) / np.tan(np.radians(args.hfov) / 2)
center = (sb[:3] + sb[3:]) / 2
front_y = float(sb[1])          # 서가 앞면 (로봇이 있는 쪽)
say(f"카메라 초점거리 화소 {fx:.1f}, 서가 중심 {np.round(center, 3).tolist()}")

saved = []
for v in range(args.views):
    d0, d1 = args.dist
    dist = d0 + (d1 - d0) * v / max(1, args.views - 1)
    eye = np.array([center[0], front_y - dist, center[2] + (0.05 if v % 2 else -0.05)])
    look = center - eye
    look /= np.linalg.norm(look)
    right = np.cross(look, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, look)
    m = np.stack([right, up, -look], axis=1)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = np.zeros(4)
        q[0] = (m[k, j] - m[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (m[j, i] + m[i, j]) / s
        q[k + 1] = (m[k, i] + m[i, k]) / s
    q = q / np.linalg.norm(q)
    cam_prim.set_world_pose(eye, q)
    key_prim.set_world_pose(eye + np.array([0.3, 0.2, 0.6]), np.array([1.0, 0, 0, 0]))
    rep.orchestrator.step(rt_subframes=6, delta_time=0.0, pause_timeline=True)

    depth = np.asarray(ann_depth.get_data(), dtype=np.float32)
    rgb = np.asarray(ann_rgb.get_data())[:, :, :3]
    box = ann_box.get_data()
    data = box["data"] if isinstance(box, dict) else box
    info = box.get("info", {}) if isinstance(box, dict) else {}
    id2label = info.get("idToLabels", {})

    shelf_box = None
    for d in data:
        sid = int(d["semanticId"])
        label = str(id2label.get(sid, id2label.get(str(sid), ""))).lower()
        if "shelf" in label:
            cand = (float(d["x_min"]), float(d["y_min"]), float(d["x_max"]), float(d["y_max"]))
            if shelf_box is None or (cand[2] - cand[0]) * (cand[3] - cand[1]) > \
                    (shelf_box[2] - shelf_box[0]) * (shelf_box[3] - shelf_box[1]):
                shelf_box = cand
    if shelf_box is None:
        say(f"  {v}: 서가 상자를 못 얻었다 (라벨 {list(id2label.values())[:3]})")
        continue
    name = f"shelf_{v:02d}"
    np.savez(os.path.join(args.out, name + ".npz"), depth=depth, bbox=np.array(shelf_box),
             fx=fx, fy=fx, cx=args.res / 2, cy=args.res / 2, eye=eye, distance=dist)
    Image.fromarray(rgb.astype(np.uint8)).save(os.path.join(args.out, name + ".jpg"), quality=92)
    saved.append({"name": name, "bbox": [round(b, 1) for b in shelf_box], "distance": round(dist, 3)})
    say(f"  {v}: 서가 상자 {[round(b) for b in shelf_box]}, 거리 {dist:.2f} m, "
        f"깊이 {np.nanmin(depth):.2f}~{np.nanmax(depth[np.isfinite(depth)]):.2f} m")

with open(os.path.join(args.out, "truth.json"), "w") as f:
    json.dump({"shelf": target_shelf, "rows": truth, "books_on_shelf": len(books),
               "views": saved, "res": args.res, "hfov": args.hfov}, f, ensure_ascii=False, indent=1)
say(f"완료: {len(saved)} 장 → {args.out}")
app.close()
