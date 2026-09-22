"""결합 로봇을 옆·앞·위에서 찍고, 팔이 카터 어디에 얹혔는지 수치로 확인한다.

왜: 팔이 카터 위에 **떠 있어** 보인다는 보고. 장착 높이를 카터 AABB 윗면(0.555)으로 잡았는데,
그 높이는 **센서 마스트 꼭대기**일 수 있다. 실제 상판(데크) 높이를 찾아야 한다.

산출물
    <out>/side.png  front.png  top.png  iso.png
    로그: 카터 하위 prim 별 높이, 팔 밑면 z, 상판 후보 z

실행
    ISAAC_ENTRY=isaac_sim/isaac/tools/capture_carter_views.py ./scripts/run_isaac_tool.sh \\
        --usd ~/carter_m0609_handoff/carter_m0609.usd --out ~/carter_views
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/carter_m0609_handoff/carter_m0609.usd"))
ap.add_argument("--robot", default="/World/carter_m0609")
ap.add_argument("--out", default=os.path.expanduser("~/carter_views"))
ap.add_argument("--res", type=int, default=900)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


open_stage(args.usd)
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
os.makedirs(args.out, exist_ok=True)


def aabb(path):
    cache.Clear()
    b = np.array(compute_aabb(cache, path, include_children=True), float)
    return b if np.all(np.isfinite(b)) else None


carter = aabb(args.robot + "/carter")
arm = aabb(args.robot + "/arm")
if carter is None or arm is None:
    say(f"카터 또는 팔을 찾지 못했다 (carter={carter is not None}, arm={arm is not None})")
    app.close()
    sys.exit(1)
say(f"카터 x {carter[0]:+.3f}~{carter[3]:+.3f} y {carter[1]:+.3f}~{carter[4]:+.3f} z {carter[2]:+.3f}~{carter[5]:+.3f}")
say(f"팔   x {arm[0]:+.3f}~{arm[3]:+.3f} y {arm[1]:+.3f}~{arm[4]:+.3f} z {arm[2]:+.3f}~{arm[5]:+.3f}")
say(f"**팔 밑면 z {arm[2]:+.3f}**  (카터 윗면 {carter[5]:+.3f})")

# 카터 하위 prim 을 높이순으로 — 어느 것이 마스트이고 어느 것이 상판인지 본다
rows = []
for p in Usd.PrimRange(stage.GetPrimAtPath(args.robot + "/carter")):
    if p.GetPath().pathString.count("/") > 5:
        continue
    b = aabb(p.GetPath().pathString)
    if b is None:
        continue
    size = b[3:] - b[:3]
    if size.min() <= 0:
        continue
    rows.append((float(b[5]), p.GetName(), b, size))
# 튜플에 numpy 배열이 있어 그냥 정렬하면 비교가 모호해진다 → 키를 지정한다
rows.sort(key=lambda r: r[0], reverse=True)
say("카터 하위 prim (윗면 높은 순):")
for top, name, b, size in rows[:12]:
    say(f"  {name:26s} 윗면 z {top:+.3f}  크기 {np.round(size, 3).tolist()}")

# 상판 후보: 가로로 넓고(>0.2 m) 얇은(<0.12 m) 것 중 가장 높은 것
flat = [(t, n, s) for t, n, b, s in rows if s[0] > 0.2 and s[1] > 0.2 and s[2] < 0.12]
if flat:
    say(f"상판 후보: {[(n, round(t, 3)) for t, n, s in flat[:3]]}")
else:
    say("상판 후보를 못 찾음 — 옆면 그림으로 판단할 것")

# 조명 + 카메라
key = UsdLux.DistantLight.Define(stage, "/World/view_key")
key.CreateIntensityAttr(3000.0)
UsdLux.DomeLight.Define(stage, "/World/view_dome").CreateIntensityAttr(800.0)

cam_path = "/World/view_cam"
cam = UsdGeom.Camera.Define(stage, cam_path)
cam.CreateFocalLengthAttr(35.0)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(36.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
cam_prim = SingleXFormPrim(cam_path)
rp = rep.create.render_product(cam_path, (args.res, args.res))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach(rp)

both = np.array([min(carter[0], arm[0]), min(carter[1], arm[1]), min(carter[2], arm[2]),
                 max(carter[3], arm[3]), max(carter[4], arm[4]), max(carter[5], arm[5])])
center = (both[:3] + both[3:]) / 2
radius = float(np.linalg.norm(both[3:] - both[:3])) * 1.1


def look_at(eye, target, up=np.array([0.0, 0.0, 1.0])):
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= (np.linalg.norm(f) or 1.0)
    if abs(float(np.dot(f, up))) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up); r /= (np.linalg.norm(r) or 1.0)
    u = np.cross(r, f)
    m = np.eye(3); m[:, 0], m[:, 1], m[:, 2] = r, u, -f
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = int(np.argmax(np.diag(m))); j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)


views = {
    "side":  center + np.array([0.0, -radius, 0.05]),      # 옆에서 (y −)
    "front": center + np.array([radius, 0.0, 0.05]),       # 앞에서 (x +)
    "top":   center + np.array([0.0, 0.0, radius]),        # 위에서
    "iso":   center + np.array([radius * 0.7, -radius * 0.7, radius * 0.5]),
}
for name, eye in views.items():
    cam_prim.set_world_pose(eye, look_at(eye, center))
    rep.orchestrator.step(rt_subframes=8, delta_time=0.0, pause_timeline=True)
    img = np.asarray(ann.get_data())[:, :, :3].astype(np.uint8)
    path = os.path.join(args.out, name + ".png")
    Image.fromarray(img).save(path)
    say(f"저장 {path}")

app.close()
