"""트레이 맞춤 시험: v3 레벨 데크에 트레이를 놓고 책을 칸마다 세운 뒤 안착 상태를 잰다.

측정: 책별 기울기·높이·이동량, 손가락-옆책 여유(기하), 캡처.
레벨 파일은 수정하지 않는다 (기존 데크 책·칸막이는 메모리에서 비활성).

GPU PC:
    ~/isaacsim/python.sh ~/arm/isaac/tray_fit.py --tray ~/book_dataset/assets/tray/tray_v1.usdc --out /tmp/tray_fit
"""
import argparse, json, math, os, sys
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=_paths.default_usd())
ap.add_argument("--tray", required=True)
ap.add_argument("--out", default="/tmp/tray_fit")
ap.add_argument("--center", type=float, nargs=2, default=[2.36, -2.94], help="트레이 중심 x y (월드)")
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--settle", type=float, default=4.0)
args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import cv2, numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera

os.makedirs(args.out, exist_ok=True)
def say(m): sys.stderr.write(f"### {m}\n"); sys.stderr.flush()
open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
cache = create_bbox_cache()
def aabb(p):
    cache.Clear(); return np.array(compute_aabb(cache, p, include_children=True), float)

DECK_Z = 0.286
BOOK_SRC = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
FINGER_T = 0.0264          # 손가락 두께(벌림축) 실측
GRIP_CLEAR = 0.005         # 책 표면과 손가락 사이 여유 (한쪽)

# 기존 데크 책 원본 자세·참조 경로를 가져오고, 데크 위 물체는 비활성
src_prim = stage.GetPrimAtPath(BOOK_SRC)
book_ref = None
for spec in src_prim.GetPrimStack():
    for r in list(spec.referenceList.GetAddedOrExplicitItems()) + list(spec.payloadList.GetAddedOrExplicitItems()):
        book_ref = r.assetPath
src_q = SingleXFormPrim(BOOK_SRC).get_world_pose()[1]
layer_dir = os.path.dirname(stage.GetRootLayer().realPath)
book_ref_abs = os.path.normpath(os.path.join(layer_dir, book_ref)) if book_ref and not os.path.isabs(book_ref) else book_ref
say(f"책 원본 참조: {book_ref_abs}")
for p in (BOOK_SRC, "/World/fixtures/tray_divider_L", "/World/fixtures/tray_divider_R"):
    if stage.GetPrimAtPath(p).IsValid():
        stage.GetPrimAtPath(p).SetActive(False)

# 트레이 (정적 충돌체)
TRAY = "/World/test_tray"
add_reference_to_stage(args.tray, TRAY)
tray_xf = SingleXFormPrim(TRAY)
tray_xf.set_world_pose(np.array([args.center[0], args.center[1], DECK_Z]), np.array([1.0, 0, 0, 0]))
for p in Usd.PrimRange(stage.GetPrimAtPath(TRAY)):
    if p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
tb = aabb(TRAY)
say(f"트레이 AABB {np.round(tb,3).tolist()}  크기 {np.round(tb[3:]-tb[:3],3).tolist()}")

# 트레이 USD 의 칸 정보 (make_tray.py 가 커스텀 속성으로 남김)
pitch = None; nslots = None; gap = None; floor_top = None
for p in Usd.PrimRange(stage.GetPrimAtPath(TRAY)):
    for a in p.GetAttributes():
        n = a.GetName()
        if n.endswith("tray_pitch"): pitch = float(a.Get())
        if n.endswith("tray_slots"): nslots = int(a.Get())
        if n.endswith("slot_gap"): gap = float(a.Get())
        if n.endswith("floor_top_z"): floor_top = float(a.Get())
slot_x = [(i + 0.5 - nslots / 2) * pitch for i in range(nslots)]
say(f"칸 {nslots}개 간격 {pitch} 폭 {gap}  바닥판 윗면 {floor_top}  칸 중심 x {[round(v,4) for v in slot_x]}")

# 책 배치
books = []
for i in range(min(args.books, len(slot_x))):
    path = f"/World/test_books/book_{i}"
    add_reference_to_stage(book_ref_abs, path)
    prim = stage.GetPrimAtPath(path)
    UsdPhysics.RigidBodyAPI.Apply(prim); UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(0.5)
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("convexHull")
    xf = SingleXFormPrim(path)
    xf.set_world_pose(np.array([0.0, 0.0, 5.0 + i]), src_q)
    b = aabb(path); c = (b[:3] + b[3:]) / 2
    target = np.array([args.center[0] + slot_x[i], args.center[1], DECK_Z + floor_top + (b[5] - b[2]) / 2 + 0.002])
    pos, q = xf.get_world_pose()
    xf.set_world_pose(np.array(pos) + (target - c), q)
    b0 = aabb(path)
    books.append({"path": path, "start": b0})
say(f"책 {len(books)}권 배치. 시작 AABB 크기 {np.round(books[0]['start'][3:]-books[0]['start'][:3],3).tolist()}")

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
cams = {}
for name, off in {"iso": (-0.75, -0.75, 0.65), "top": (0.001, 0.0, 1.1), "side": (0.0, -0.95, 0.25)}.items():
    cp = f"/World/fit_cam_{name}"
    cams[name] = Camera(prim_path=cp, resolution=(960, 540))
    ctr = np.array([args.center[0], args.center[1], DECK_Z + 0.09])
    set_camera_view(eye=ctr + np.array(off), target=ctr, camera_prim_path=cp)
world.reset()
for c in cams.values(): c.initialize()
for _ in range(int(args.settle * 60)):
    world.step(render=False)
for _ in range(40): world.step(render=True)   # 카메라 버퍼가 차려면 렌더 프레임이 여러 장 필요하다 (5장은 빈 이미지)
for name, c in cams.items():
    rgba = c.get_rgba()
    if rgba is not None and rgba.size:
        cv2.imwrite(f"{args.out}/{name}.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))

report = []
for i, bk in enumerate(books):
    b = aabb(bk["path"]); s = bk["start"]
    h = b[5] - b[2]; wx = b[3] - b[0]
    tilt = math.degrees(math.asin(min(1.0, max(0.0, (wx - (s[3] - s[0])) / max(1e-6, h)))))
    moved = float(np.linalg.norm(((b[:3] + b[3:]) - (s[:3] + s[3:])) / 2))
    upright = abs(h - (s[5] - s[2])) < 0.01
    report.append({"slot": i, "height": round(h, 4), "x_extent": round(wx, 4), "tilt_deg_est": round(tilt, 1),
                   "moved_m": round(moved, 4), "upright": bool(upright), "aabb": np.round(b, 4).tolist()})
    say(f"칸 {i}: 높이 {h:.3f} (시작 {s[5]-s[2]:.3f})  x폭 {wx:.3f} (시작 {s[3]-s[0]:.3f})  기울기 약 {tilt:.1f}°  이동 {moved*1000:.1f}mm  서있음 {upright}")

# 손가락-옆책 여유: 책 i 를 잡으려고 벌렸을 때 손가락 바깥면과 이웃 책 표면 사이
clear = []
for i, r in enumerate(report):
    b = np.array(r["aabb"]); cx = (b[0] + b[3]) / 2; half_t = (b[3] - b[0]) / 2
    finger_outer = half_t + GRIP_CLEAR + FINGER_T
    for j in (i - 1, i + 1):
        if 0 <= j < len(report):
            nb = np.array(report[j]["aabb"])
            gap_ij = (nb[0] - cx) if j > i else (cx - nb[3])
            clear.append(gap_ij - finger_outer)
min_clear = min(clear) if clear else float("nan")
say(f"손가락-옆책 최소 여유 {min_clear*1000:.1f} mm (0 이하면 파지 시 옆 책을 친다)")
ok = all(r["upright"] and r["moved_m"] < 0.01 for r in report) and min_clear > 0.003
say(f"트레이 맞춤 판정: {'통과' if ok else '불통과'}")
json.dump({"tray": args.tray, "books": report, "min_finger_clearance_m": float(min_clear), "pass": bool(ok)}, open(f"{args.out}/fit.json", "w"), indent=1)
app.close()
