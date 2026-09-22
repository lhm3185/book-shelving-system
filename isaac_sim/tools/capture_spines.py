"""트레이에 꽂힌 책의 **책등**을 여러 각도에서 찍어 YOLO 학습 데이터를 만든다 (Isaac Sim 5.1.0).

왜 이렇게 하나 (2026-09-18 실측, MIXED_BOOK_TEST.md)
    9/17 시연에서 트레이 책이 하나의 큰 박스로 묶여 인식됐다. 책 종류를 6종으로 바꿔도 같았다.
    원인은 "학습 데이터에 트레이 장면이 없다" 쪽이므로, **실제 트레이 장면**을 찍어 학습시킨다.
    손목 카메라가 보는 시점은 정해져 있으니 시점 종류를 늘리기보다 **책등이 보이는 각도**를 촘촘히 덮는다.

라벨
    Isaac 의 의미 라벨 + bounding_box_2d_tight 주석기를 쓴다. 가려진 부분은 자동으로 빠지고
    책마다 한 개의 상자가 나온다 (= 지금 문제인 "여러 권이 한 상자" 를 바로잡는 정답 라벨).

실행 (GPU PC)
    ISAAC_ENTRY=isaac_sim/isaac/tools/capture_spines.py ./scripts/run_isaac_tool.sh \
        --views 70 --rounds 8 --out ~/spine_ds
"""
import argparse
import json
import os
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
import _paths  # noqa: E402
import random
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=_paths.default_usd())
ap.add_argument("--tray", default=_paths.default_tray())
ap.add_argument("--tray-center", type=float, nargs=2, default=[2.36, -2.94])
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--out", default=os.path.expanduser("~/spine_ds"))
ap.add_argument("--views", type=int, default=60, help="한 배치(책 구성)당 시점 수")
ap.add_argument("--rounds", type=int, default=4, help="책 구성을 바꿔 가며 반복할 횟수")
ap.add_argument("--res", type=int, default=640)
ap.add_argument("--hfov", type=float, default=90.5, help="손목 RealSense 가로 화각")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--min-box", type=int, default=10, help="이보다 작은 상자(픽셀)는 버린다")
ap.add_argument("--max-occlusion", type=float, default=0.8, help="이보다 많이 가려지면 버린다")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.semantics import add_labels  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers"))
from book_scene import BookScene  # noqa: E402

MIXED_BOOKS = [
    "book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15",
    "decorative_book_set_01_2k__book_hardcover_01_cover02",
    "decorative_book_set_01_2k__book_softcover_01_cover14",
    "decorative_book_set_01_2k__book_hardcover_01_cover08",
    "oldbook__OldBook001",
    "book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book01",
]


def say(m):
    print(f"### {m}", flush=True)


rng = random.Random(args.seed)
np_rng = np.random.default_rng(args.seed)
out = args.out
for sub in ("images", "labels"):
    os.makedirs(os.path.join(out, sub), exist_ok=True)

scene = BookScene(app, args.usd, args.tray, args.tray_center, args.books,
                  [-0.51, -0.43, -0.35, -0.27], say, book_variants=MIXED_BOOKS)
stage = scene.stage

# 트레이 책 + 레벨 서가의 책에도 라벨을 붙인다.
# 배경 서가 책을 라벨 없이 두면 "책인데 책이 아니다" 로 배우게 된다.
# 트레이 칸막이(흰 빗살)는 일부러 라벨을 주지 않는다 — 지금 이것을 책으로 묶어 보는 것이 문제라서
# "이건 책이 아니다" 를 배워야 한다.
n_label = 0
for b in scene.books:
    add_labels(stage.GetPrimAtPath(b), labels=["book"], instance_name="class")
    n_label += 1
lib = stage.GetPrimAtPath("/World/books")
if lib.IsValid():
    for c in lib.GetChildren():
        if c.IsActive():
            add_labels(c, labels=["book"], instance_name="class")
            n_label += 1
say(f"라벨 붙인 책 prim {n_label}개 (트레이 {len(scene.books)} + 레벨 서가)")

cam_path = "/World/bs_capture_cam"
cam = UsdGeom.Camera.Define(stage, cam_path)
cam.CreateFocalLengthAttr(18.0)
aperture = 2.0 * 18.0 * np.tan(np.radians(args.hfov) / 2.0)
cam.CreateHorizontalApertureAttr(float(aperture))
cam.CreateVerticalApertureAttr(float(aperture))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
cam_prim = SingleXFormPrim(cam_path)

# 조명: 레벨 조명만으로는 트레이를 위에서 볼 때 너무 어둡다. 촬영용 조명을 더하고 시점마다 세기를 바꾼다
key = UsdLux.SphereLight.Define(stage, "/World/bs_capture_key")
key.CreateRadiusAttr(0.25)
key_prim = SingleXFormPrim("/World/bs_capture_key")
fill = UsdLux.DistantLight.Define(stage, "/World/bs_capture_fill")
fill.CreateAngleAttr(2.0)
fill_prim = SingleXFormPrim("/World/bs_capture_fill")

rp = rep.create.render_product(cam_path, (args.res, args.res))
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
ann_box = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
ann_rgb.attach(rp)
ann_box.attach(rp)


def look_at(eye, target, up=np.array([0.0, 0.0, 1.0])):
    """USD 카메라는 −Z 를 보고 +Y 가 위다"""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f = f / (np.linalg.norm(f) or 1.0)
    if abs(float(np.dot(f, up))) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up); r = r / (np.linalg.norm(r) or 1.0)
    u = np.cross(r, f)
    m = np.eye(3)
    m[:, 0], m[:, 1], m[:, 2] = r, u, -f
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
    return q / np.linalg.norm(q)


def spine_top_center():
    """트레이에 남아 있는 책들의 책등 윗면 중심"""
    pts = []
    for b in scene.books:
        c = scene.center(b)
        if c[2] > 0.1:
            pts.append(c)
    if not pts:
        return np.array([args.tray_center[0], args.tray_center[1], 0.40])
    p = np.mean(np.array(pts), axis=0)
    return np.array([p[0], p[1], p[2] + 0.06])


def settle(steps=60):
    for _ in range(steps):
        scene.world.step(render=False)


kept = 0
written = 0
stats = {"rounds": [], "boxes_per_image": []}
home = np.array([0.0, 0.0, -6.0])
for rnd in range(args.rounds):
    # 책 구성 바꾸기: 종류를 섞어 6칸 중 3~6권만 남긴다 (트레이가 비어 가는 상황도 학습)
    n_keep = rng.choice([6, 6, 5, 4, 3])
    keep = rng.sample(range(len(scene.books)), min(n_keep, len(scene.books)))
    slots = list(range(len(scene.slot_x)))
    rng.shuffle(slots)                       # 어느 칸에 어떤 책이 오는지도 바꾼다
    for n, i in enumerate(keep):
        b = scene.books[i]
        depth = scene.dims[b][2]
        SingleXFormPrim(b).set_world_pose(
            np.array([scene.slot_x[slots[n]], scene.tray_y, scene.tray_floor_z + depth / 2 + 0.002]),
            scene.upright_q[b])
    for i, b in enumerate(scene.books):
        if i in keep:
            continue
        SingleXFormPrim(b).set_world_pose(home + np.array([0.3 * i, 0, 0]),
                                          scene.upright_q.get(b, np.array([1.0, 0, 0, 0])))
    settle(40)
    stats["rounds"].append({"round": rnd, "books": len(keep)})
    say(f"배치 {rnd + 1}/{args.rounds}: 트레이 책 {len(keep)}권")

    for v in range(args.views):
        # 40% 는 한 권을 크게 잡는 근접 촬영 (책등 하나를 확실히 배우게), 나머지는 트레이 전체
        close = rng.random() < 0.4
        if close:
            b = scene.books[rng.choice(keep)]
            c = scene.center(b)
            target = np.array([c[0], c[1], c[2] + scene.dims[b][2] / 2]) + np_rng.normal(0, 0.006, 3)
            dist = rng.uniform(0.14, 0.32)
        else:
            target = spine_top_center() + np_rng.normal(0, 0.012, 3)
            dist = rng.uniform(0.28, 0.75)
        # 책등이 보이는 범위: 위에서 비스듬히 내려다보는 각도 (손목 카메라가 실제로 오는 방향 포함)
        az = np.radians(rng.uniform(-180, 180))
        el = np.radians(rng.uniform(35, 88))
        eye = target + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        # 조명도 각도·세기를 바꾼다 (그림자 방향이 늘 같으면 그 그림자까지 배우게 된다)
        laz = np.radians(rng.uniform(-180, 180))
        lel = np.radians(rng.uniform(35, 85))
        key_prim.set_world_pose(target + rng.uniform(0.8, 1.8) * np.array(
            [np.cos(lel) * np.cos(laz), np.cos(lel) * np.sin(laz), np.sin(lel)]), np.array([1.0, 0, 0, 0]))
        key.CreateIntensityAttr(float(rng.uniform(12000, 60000)))
        fill_prim.set_world_pose(target + np.array([0, 0, 2.0]), look_at(target + np.array([0, 0, 2.0]), target))
        fill.CreateIntensityAttr(float(rng.uniform(300, 1500)))
        cam_prim.set_world_pose(eye, look_at(eye, target))
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=True)
        rgb = ann_rgb.get_data()
        box = ann_box.get_data()
        data = box["data"] if isinstance(box, dict) else box
        rows = []
        for d in data:
            x0, y0, x1, y1 = float(d["x_min"]), float(d["y_min"]), float(d["x_max"]), float(d["y_max"])
            occ = float(d["occlusionRatio"]) if "occlusionRatio" in d.dtype.names else 0.0
            x0, y0 = max(0.0, x0), max(0.0, y0)
            x1, y1 = min(args.res - 1.0, x1), min(args.res - 1.0, y1)
            w, h = x1 - x0, y1 - y0
            if w < args.min_box or h < args.min_box or occ > args.max_occlusion:
                continue
            rows.append((0, (x0 + x1) / 2 / args.res, (y0 + y1) / 2 / args.res, w / args.res, h / args.res))
        if not rows:
            continue
        name = f"{written:05d}"
        img = np.asarray(rgb)[:, :, :3]
        Image.fromarray(img.astype(np.uint8)).save(os.path.join(out, "images", name + ".jpg"), quality=92)
        with open(os.path.join(out, "labels", name + ".txt"), "w") as f:
            for c, cx, cy, bw, bh in rows:
                f.write(f"{c} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
        written += 1
        kept += len(rows)
        stats["boxes_per_image"].append(len(rows))
        if written % 25 == 0:
            say(f"  {written} 장 (상자 {kept}개)")



stats["images"] = written
stats["boxes"] = kept
with open(os.path.join(out, "capture_stats.json"), "w") as f:
    json.dump(stats, f, ensure_ascii=False, indent=2)
say(f"완료: 사진 {written} 장, 상자 {kept} 개 → {out}")
app.close()
