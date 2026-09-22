"""팔을 얹을 **실제 상판**을 찾는다 — 높이뿐 아니라 "어디에" 얹을지까지.

왜: 장착 높이를 카터 AABB 윗면(0.555)으로 잡았는데 그 높이는 **라이다 마스트 꼭대기**였고,
    장착 xy (0,0) 아래에 있는 것은 상판이 아니라 **front_RPLidar** 였다. (2026-09-18 실측)
    AMR 담당이 라이다로 주행 시험 중이라 센서 위에 얹으면 안 된다.

방법
    1) `/arm` 을 끈 상태에서 카터 발자국 전체에 **격자로 아래로 광선**을 쏴 높이 지도를 만든다
    2) 센서 prim 들의 발자국을 따로 뽑아 지도에 겹쳐 **피해야 할 칸**을 표시한다
    3) 센서를 피하면서 평평하고(높이 편차 작음) 넓은 칸을 골라 장착 후보로 제안한다

실행
    ISAAC_ENTRY=isaac_sim/isaac/tools/probe_carter_deck.py ./scripts/run_isaac_tool.sh \\
        --usd ~/carter_m0609_handoff/carter_m0609.usd
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/carter_m0609_handoff/carter_m0609.usd"))
ap.add_argument("--robot", default="/World/carter_m0609")
ap.add_argument("--step", type=float, default=0.02, help="격자 간격 (m)")
ap.add_argument("--pad", type=float, default=0.075, help="팔 받침판 반경 (m) — 이만큼 평평해야 한다")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
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


def aabb(path):
    cache.Clear()
    b = np.array(compute_aabb(cache, path, include_children=True), float)
    return b if np.all(np.isfinite(b)) else None


arm = aabb(args.robot + "/arm")
cur_z = 0.0
_armx = UsdGeom.Xformable(stage.GetPrimAtPath(args.robot + "/arm"))
for op in (_armx.GetOrderedXformOps() if _armx else []):
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        cur_z = float(op.Get()[2])
below = cur_z - arm[2] if arm is not None else 0.0
say(f"팔 원점 z {cur_z:+.4f}, 팔 밑면 {arm[2]:+.4f} → 원점 아래 기하 {below:+.4f} m")

# --- 센서 발자국 (피해야 할 곳)
SENSOR = ("lidar", "camera", "imu", "hawk", "owl", "realsense", "stereo")
blocked = []
for p in Usd.PrimRange(stage.GetPrimAtPath(args.robot + "/carter")):
    n = p.GetName().lower()
    if not any(k in n for k in SENSOR):
        continue
    if any(k in p.GetParent().GetName().lower() for k in SENSOR):
        continue          # 이미 부모를 담았으면 건너뛴다
    b = aabb(p.GetPath().pathString)
    if b is None:
        continue
    blocked.append((p.GetName(), b))
say(f"센서 발자국 {len(blocked)}개 (여기에는 얹지 않는다):")
for n, b in blocked:
    say(f"  {n:24s} x {b[0]:+.3f}~{b[3]:+.3f}  y {b[1]:+.3f}~{b[4]:+.3f}  윗면 z {b[5]:+.3f}")

# --- 높이 지도
armp = stage.GetPrimAtPath(args.robot + "/arm")
if armp.IsValid():
    armp.SetActive(False)
world = World(stage_units_in_meters=1.0)
world.reset()
world.step(render=False)   # 베이스가 고정돼 있지 않아 오래 돌리면 가라앉는다 — 한 스텝만
from omni.physx import get_physx_scene_query_interface  # noqa: E402
sq = get_physx_scene_query_interface()

cb = aabb(args.robot + "/carter")
xs = np.arange(cb[0], cb[3] + 1e-9, args.step)
ys = np.arange(cb[1], cb[4] + 1e-9, args.step)
H = np.full((len(xs), len(ys)), np.nan)
for i, x in enumerate(xs):
    for j, y in enumerate(ys):
        h = sq.raycast_closest([float(x), float(y), 1.5], [0.0, 0.0, -1.0], 3.0)
        if h.get("hit"):
            H[i, j] = float(h["position"][2])
if armp.IsValid():
    armp.SetActive(True)


# 센서가 **상판보다 낮으면** 얹는 데 방해가 되지 않는다. 옆구리에 박힌 카메라를 가리면
# 상판 전체가 못 쓰는 곳으로 보인다 (첫 지도의 오판). 높이로 거른다.
DECK_GUESS = float(np.nanmax(H)) - 0.02


def is_blocked(x, y, m=0.03):
    return any(b[5] >= DECK_GUESS and b[0] - m <= x <= b[3] + m and b[1] - m <= y <= b[4] + m
               for _, b in blocked)


say(f"상판 추정 {DECK_GUESS + 0.02:+.3f} — 이보다 낮은 센서는 방해가 아니다")
say("높이 지도 (행=x 앞→뒤, 열=y 좌→우, cm / 뒤 * = 상판보다 높은 센서 / .=빔 없음)")
say("      " + "".join(f"{y*100:+5.0f}" for y in ys))
for i in range(len(xs) - 1, -1, -1):
    row = f"x{xs[i]*100:+6.0f} "
    for j, y in enumerate(ys):
        if np.isnan(H[i, j]):
            row += "    ."
        else:
            row += f"{H[i, j]*100:4.0f}" + ("*" if is_blocked(xs[i], y) else " ")
    say(row)

# --- 후보: 받침판 반경 안이 평평하고 센서를 피하는 칸
r = int(round(args.pad / args.step))
cands = []
for i in range(len(xs)):
    for j in range(len(ys)):
        if np.isnan(H[i, j]) or is_blocked(xs[i], ys[j]):
            continue
        patch = H[max(0, i - r):i + r + 1, max(0, j - r):j + r + 1]
        if np.isnan(patch).any() or patch.size < (2 * r + 1) ** 2:
            continue
        if any(is_blocked(xs[a], ys[b])
               for a in range(max(0, i - r), min(len(xs), i + r + 1))
               for b in range(max(0, j - r), min(len(ys), j + r + 1))):
            continue
        flat = float(patch.max() - patch.min())
        if flat > 0.006:          # 6 mm 넘게 기울면 받침판이 안 닿는다
            continue
        cands.append((float(H[i, j]), flat, float(xs[i]), float(ys[j])))

if not cands:
    say("평평한 후보를 못 찾음 — --pad 를 줄이거나 지도를 보고 직접 고를 것")
else:
    cands.sort(key=lambda c: (-c[0], c[1]))       # 높고 평평한 순
    say("장착 후보 (높은 순):")
    for z, flat, x, y in cands[:8]:
        say(f"  상판 z {z:+.4f} (편차 {flat*1000:.1f} mm)  xy ({x:+.3f}, {y:+.3f})"
            f"  → --mount-xyz {x:g} {y:g} {z + below:.4f}")
    z, flat, x, y = cands[0]
    say(f"**추천: --mount-xyz {x:g} {y:g} {z + below:.4f}**")

app.close()
