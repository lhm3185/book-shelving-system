"""트레이 없이 **무작위로 놓이거나 쌓인 책**의 인식·파지 가능성을 잰다 (Isaac Sim 5.1.0).

범위 (웹 클로드 v12 회신 A3): **인식 측정 + 파지 계획까지. 실행하지 않는다.**
    시연 경로(`feature/robot_control`)를 건드리지 않으려고 `test/random-stack-grasp` 에서만 쓴다.

두 가지를 낸다
    1) 인식용 사진 + 정답 상자 (Isaac 주석기) — 학습한 모델이 이 배치에서도 잡는지 재려고
    2) 책마다 파지 계획 성공 여부 — IK 해가 있는지, 경로 관절 변화가 기준 안인지 (M401/M402 기준 그대로)

실행 (GPU PC)
    ISAAC_ENTRY=~/arm/isaac/random_stack_probe.py ~/arm/isaac/run_capture.sh \
        --layouts 4 --views 20 --out ~/stack_probe
"""
import argparse
import json
import math
import os
import random
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/Desktop/ing_library_env_v5.usd"))
ap.add_argument("--tray", default=os.path.expanduser("~/book_dataset/assets/tray/tray_v1.usdc"))
ap.add_argument("--tray-center", type=float, nargs=2, default=[2.36, -2.94])
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--layouts", type=int, default=4, help="배치를 몇 번 바꿔 시험할지")
ap.add_argument("--views", type=int, default=20, help="배치마다 찍을 사진 수")
ap.add_argument("--out", default=os.path.expanduser("~/stack_probe"))
ap.add_argument("--res", type=int, default=640)
ap.add_argument("--hfov", type=float, default=90.5)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--layout", choices=["mix", "flat", "stack", "lean"], default="mix")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.semantics import add_labels  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from book_scene import BookScene, DECK_Z, GRIP_CLEAR, MAX_STEP, TIP_DOWN  # noqa: E402
from arm_geometry import R_from_quat  # noqa: E402

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
for sub in ("images", "labels"):
    os.makedirs(os.path.join(args.out, sub), exist_ok=True)

scene = BookScene(app, args.usd, args.tray, args.tray_center, args.books,
                  [-0.51, -0.43, -0.35, -0.27], say, book_variants=MIXED_BOOKS)
stage = scene.stage

# 트레이는 치운다 — 이 시험의 전제가 "트레이 없이"다
SingleXFormPrim(scene.tray).set_world_pose(np.array([0.0, 0.0, -8.0]), np.array([1.0, 0, 0, 0]))

for b in scene.books:
    add_labels(stage.GetPrimAtPath(b), labels=["book"], instance_name="class")
lib = stage.GetPrimAtPath("/World/books")
if lib.IsValid():
    for c in lib.GetChildren():
        if c.IsActive():
            add_labels(c, labels=["book"], instance_name="class")

cam_path = "/World/bs_probe_cam"
cam = UsdGeom.Camera.Define(stage, cam_path)
cam.CreateFocalLengthAttr(18.0)
ap_mm = 2.0 * 18.0 * np.tan(np.radians(args.hfov) / 2.0)
cam.CreateHorizontalApertureAttr(float(ap_mm)); cam.CreateVerticalApertureAttr(float(ap_mm))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
cam_prim = SingleXFormPrim(cam_path)
key = UsdLux.SphereLight.Define(stage, "/World/bs_probe_key")
key.CreateRadiusAttr(0.25)
key_prim = SingleXFormPrim("/World/bs_probe_key")

rp = rep.create.render_product(cam_path, (args.res, args.res))
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb"); ann_rgb.attach(rp)
ann_box = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight"); ann_box.attach(rp)


def look_at(eye, target, up=np.array([0.0, 0.0, 1.0])):
    f = np.asarray(target, float) - np.asarray(eye, float)
    f = f / (np.linalg.norm(f) or 1.0)
    if abs(float(np.dot(f, up))) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up); r /= (np.linalg.norm(r) or 1.0)
    u = np.cross(r, f)
    m = np.eye(3); m[:, 0], m[:, 1], m[:, 2] = r, u, -f
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m))); j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = np.zeros(4)
        q[0] = (m[k, j] - m[j, k]) / s; q[i + 1] = 0.25 * s
        q[j + 1] = (m[j, i] + m[i, j]) / s; q[k + 1] = (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], float)


FLAT = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])   # x축 90° : 세운 책을 눕힌다
CENTER = np.array([args.tray_center[0], args.tray_center[1]])


def place_layout(kind):
    """데크 위에 책을 무작위로 놓는다. 눕힘·쌓임·기울임을 섞는다"""
    used = []
    info = []
    for i, b in enumerate(scene.books):
        t, ln, w = scene.dims[b]
        mode = kind if kind != "mix" else rng.choice(["flat", "flat", "stack", "lean"])
        yaw = rng.uniform(-math.pi, math.pi)
        if mode == "stack" and used:
            base = used[-1]
            pos = np.array([base["x"], base["y"], base["top"] + t / 2 + 0.002])
            pos[0] += rng.uniform(-0.02, 0.02); pos[1] += rng.uniform(-0.02, 0.02)
            q = quat_mul(yaw_quat(yaw), FLAT)
            top = pos[2] + t / 2
        elif mode == "lean":
            pos = np.array([CENTER[0] + rng.uniform(-0.12, 0.12), CENTER[1] + rng.uniform(-0.10, 0.10),
                            DECK_Z + w / 2 + 0.002])
            q = quat_mul(yaw_quat(yaw), scene.upright_q[b])
            top = pos[2] + w / 2
        else:   # flat
            pos = np.array([CENTER[0] + rng.uniform(-0.14, 0.14), CENTER[1] + rng.uniform(-0.12, 0.12),
                            DECK_Z + t / 2 + 0.002])
            q = quat_mul(yaw_quat(yaw), FLAT)
            top = pos[2] + t / 2
        SingleXFormPrim(b).set_world_pose(pos, q)
        used.append({"x": float(pos[0]), "y": float(pos[1]), "top": float(top)})
        info.append({"book": b.rsplit("/", 1)[1], "mode": mode, "dims": [round(v, 3) for v in (t, ln, w)]})
    for _ in range(180):
        scene.world.step(render=False)
    return info


def buried_by(book):
    """다른 책이 이 책 위를 덮고 있는지 (쌓인 더미에서 위에서 잡을 수 없는 책)"""
    bb = scene.aabb(book)
    for other in scene.books:
        if other == book:
            continue
        ob = scene.aabb(other)
        overlap_x = min(bb[3], ob[3]) - max(bb[0], ob[0])
        overlap_y = min(bb[4], ob[4]) - max(bb[1], ob[1])
        if overlap_x > 0.02 and overlap_y > 0.02 and ob[2] > bb[5] - 0.015:
            return other.rsplit("/", 1)[1]
    return None


def grasp_plan(book):
    """위에서 내려 잡는 파지 계획만 세운다 (실행하지 않는다).

    잡는 폭은 **책의 가장 얇은 변**, 손 방향은 그 변에 직각. 책이 어떻게 놓였든 AABB 로 판단한다.
    """
    bb = scene.aabb(book)
    c = (bb[:3] + bb[3:]) / 2
    ext = bb[3:] - bb[:3]
    top_z = float(bb[5])
    # 책이 회전해 있으면 AABB 는 대각선만큼 부풀어 실제 잡는 폭이 아니다.
    # 책 자신의 축으로 본다: 세로(월드 z)에 가장 가까운 축을 빼고, 남은 두 변 중 짧은 쪽을 집는다.
    _, q_w = SingleXFormPrim(book).get_world_pose()
    Rb = R_from_quat(np.asarray(q_w, float))
    local_ext = np.array(scene.dims[book])          # 책 좌표계 (두께 x, 높이 y, 깊이 z)
    up_axis = int(np.argmax(np.abs(Rb.T @ np.array([0.0, 0.0, 1.0]))))
    horiz = [i for i in range(3) if i != up_axis]
    width = float(min(local_ext[horiz[0]], local_ext[horiz[1]]))
    grip_axis = horiz[0] if local_ext[horiz[0]] <= local_ext[horiz[1]] else horiz[1]
    closing_w = Rb[:, grip_axis]                     # 손가락이 닫히는 방향 (월드)
    closing_w[2] = 0.0
    if np.linalg.norm(closing_w) < 1e-6:
        closing_w = np.array([1.0, 0.0, 0.0])
    closing_w /= np.linalg.norm(closing_w)
    ori = scene.orientation([0, 0, -1], closing_w)
    blocker = buried_by(book)
    tip = np.array([c[0], c[1], top_z - min(TIP_DOWN, 0.4 * float(ext[2]))])
    pre = tip + np.array([0, 0, 0.13])
    rec = {"book": book.rsplit("/", 1)[1], "center": np.round(c, 3).tolist(),
           "aabb_size": np.round(ext, 3).tolist(), "grip_width": round(width, 3),
           "buried_by": blocker,
           "gripper_ok": bool(width / 2 + GRIP_CLEAR <= 0.04)}
    if blocker:
        rec.update({"plan_ok": False, "reason": f"위에 {blocker} 이 덮고 있음 (쌓임)"})
        return rec
    if not rec["gripper_ok"]:
        rec.update({"plan_ok": False,
                    "reason": f"잡을 변이 {width * 100:.1f} cm — 그리퍼 최대 8 cm 초과 (눕힌 책은 위에서 못 잡는다)"})
        return rec
    qs, worst, err = scene.plan_path([(scene.home_tip, scene.DOWN), (pre, ori), (tip, ori)], scene.q_home)
    if qs is None:
        rec.update({"plan_ok": False, "reason": f"IK 실패 (M401): {err}"})
    elif worst > MAX_STEP:
        rec.update({"plan_ok": False, "reason": f"경로 관절변화 {worst:.3f} rad > {MAX_STEP} (M402)"})
    else:
        rec.update({"plan_ok": True, "worst_joint_step": round(float(worst), 3)})
    return rec


written = 0
summary = {"layouts": [], "seed": args.seed}
for li in range(args.layouts):
    info = place_layout(args.layout)
    say(f"배치 {li + 1}/{args.layouts}: " + ", ".join(f"{d['book']}={d['mode']}" for d in info))

    # 1) 인식용 사진
    target = np.array([CENTER[0], CENTER[1], DECK_Z + 0.06])
    for v in range(args.views):
        az = np.radians(rng.uniform(-180, 180)); el = np.radians(rng.uniform(40, 88))
        dist = rng.uniform(0.3, 0.8)
        eye = target + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        cam_prim.set_world_pose(eye, look_at(eye, target + np_rng.normal(0, 0.01, 3)))
        laz = np.radians(rng.uniform(-180, 180)); lel = np.radians(rng.uniform(40, 85))
        key_prim.set_world_pose(target + rng.uniform(0.9, 1.8) * np.array(
            [np.cos(lel) * np.cos(laz), np.cos(lel) * np.sin(laz), np.sin(lel)]), np.array([1.0, 0, 0, 0]))
        key.CreateIntensityAttr(float(rng.uniform(15000, 60000)))
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=True)
        box = ann_box.get_data()
        data = box["data"] if isinstance(box, dict) else box
        rows = []
        for d in data:
            x0, y0 = max(0.0, float(d["x_min"])), max(0.0, float(d["y_min"]))
            x1, y1 = min(args.res - 1.0, float(d["x_max"])), min(args.res - 1.0, float(d["y_max"]))
            if x1 - x0 < 10 or y1 - y0 < 10:
                continue
            rows.append((0, (x0 + x1) / 2 / args.res, (y0 + y1) / 2 / args.res,
                         (x1 - x0) / args.res, (y1 - y0) / args.res))
        if not rows:
            continue
        name = f"{written:05d}"
        Image.fromarray(np.asarray(ann_rgb.get_data())[:, :, :3].astype(np.uint8)).save(
            os.path.join(args.out, "images", name + ".jpg"), quality=92)
        with open(os.path.join(args.out, "labels", name + ".txt"), "w") as f:
            for c_, cx, cy, bw, bh in rows:
                f.write(f"{c_} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
        written += 1

    # 2) 파지 계획 (실행 없음)
    plans = [grasp_plan(b) for b in scene.books]
    ok = sum(1 for p in plans if p.get("plan_ok"))
    say(f"  파지 계획 {ok}/{len(plans)} 성공" +
        ("" if ok == len(plans) else " — " + "; ".join(
            f"{p['book']}: {p.get('reason', '')}" for p in plans if not p.get("plan_ok"))))
    summary["layouts"].append({"layout": li, "books": info, "plans": plans, "images": written})
    # 배치마다 저장한다 (끝에서 한 번만 쓰면 중간에 끊겼을 때 전부 잃는다 — 2026-09-18 실측)
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

summary["images"] = written
with open(os.path.join(args.out, "summary.json"), "w") as f:
    json.dump(summary, f, ensure_ascii=False, indent=1)
tot = sum(len(l["plans"]) for l in summary["layouts"])
okc = sum(1 for l in summary["layouts"] for p in l["plans"] if p.get("plan_ok"))
say(f"완료: 사진 {written} 장, 파지 계획 {okc}/{tot} 성공 → {args.out}")
app.close()
