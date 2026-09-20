"""받침판(Cube)의 **서가 쪽 한 변**을 줄인다 — 서가에 붙을 수 있게.

왜 (2026-09-20 실측):
    검증된 Franka 시연은 팔 베이스 → 서가 앞면 **0.444 m**, 몸체 앞끝 여유 **4.8 cm** 였다.
    그리고 **물러설 수 없다** — 5 cm 만 물러서도 삽입 IK 가 4칸 중 3칸, 20 cm 면 0칸이다.
    현재 판은 서가 쪽 반폭이 0.482 m 라 그 자리에서 **서가를 5.6 cm 파고든다.**
    판에는 충돌체가 있고 서가에도 있으므로 실제로 부딪힌다.

    → 서가 쪽 반폭을 **0.396 m 이하**로. 나머지 세 변은 그대로 둬도 된다 (통로 1.82 m).

이 도구는 **원본을 고치지 않고** 새 USD 로 저장한다. AMR 담당이 그대로 써도 되고,
우리는 담당자 회신 전까지 시험용 사본으로 쓴다.

실행
    ISAAC_ENTRY=simulation/isaac/tools/resize_plate.py ./scripts/run_isaac_tool.sh \\
        --usd <레벨>.usd --out <출력>.usd
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--robot", default="/World/Nova_Carter_ROS")
ap.add_argument("--plate", default="Cube", help="받침판 prim 이름 (robot 기준)")
ap.add_argument("--arm-base", default="m0609/base_link")
ap.add_argument("--max-half", type=float, default=0.396,
                help="팔 베이스에서 **서가 쪽** 판 끝까지 허용 거리 (m). 실측 기준 0.396")
ap.add_argument("--shelf-dir", default="-x", choices=["+x", "-x", "+y", "-y"],
                help="서가가 있는 방향 (월드 기준). 이 레벨은 -x")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


open_stage(os.path.expanduser(args.usd))
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()

plate_path = f"{args.robot}/{args.plate}"
plate = stage.GetPrimAtPath(plate_path)
if not plate.IsValid():
    say(f"받침판을 못 찾았다: {plate_path}")
    app.close()
    sys.exit(1)

cache.Clear()
b = np.array(compute_aabb(cache, plate_path, include_children=True), float)
arm = stage.GetPrimAtPath(f"{args.robot}/{args.arm_base}")
P = np.array(UsdGeom.Xformable(arm).ComputeLocalToWorldTransform(0).ExtractTranslation(), float)
say(f"판 x {b[0]:+.3f}~{b[3]:+.3f} ({b[3]-b[0]:.3f} m)  y {b[1]:+.3f}~{b[4]:+.3f} ({b[4]-b[1]:.3f} m)")
say(f"팔 베이스 {np.round(P, 3).tolist()}")

axis = 0 if "x" in args.shelf_dir else 1
sign = -1 if args.shelf_dir.startswith("-") else +1
edge = b[axis] if sign < 0 else b[axis + 3]
half = abs(edge - P[axis])
say(f"서가 쪽({args.shelf_dir}) 판 끝 {edge:+.3f}, 팔에서 {half:.3f} m")

if half <= args.max_half + 1e-6:
    say(f"이미 {args.max_half} m 이하다 — 바꿀 것 없음")
    app.close()
    sys.exit(0)

trim = half - args.max_half
say(f"**{trim*1000:.0f} mm 트림한다** → 그 변 반폭 {args.max_half:.3f} m")

# Cube 는 size·scale·translate 로 크기가 정해진다. 한 변만 줄이려면
# 스케일을 줄이고 중심을 반대쪽으로 옮겨 **반대편 끝을 그대로 둔다**.
xf = UsdGeom.Xformable(plate)
ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
sc_op = next((o for n, o in ops.items() if "scale" in n), None)
tr_op = next((o for n, o in ops.items() if "translate" in n), None)
if sc_op is None or tr_op is None:
    say("판에 scale/translate op 가 없다 — 수동 조정 필요")
    app.close()
    sys.exit(1)

sc = np.array(sc_op.Get(), float)
tr = np.array(tr_op.Get(), float)
span = b[axis + 3] - b[axis]
new_span = span - trim
sc[axis] *= new_span / span
sc_op.Set(Gf.Vec3f(*[float(v) for v in sc]))
app.update()

# 스케일은 prim 자기 원점 기준으로 줄어든다. 줄인 뒤 실제 위치를 재서 보정한다.
#
# **주의**: translate 는 부모 좌표계 값이고, 이 판은 부모·자신 모두 180° yaw 가 걸려 있다.
# 월드 보정량을 그대로 더하면 **부호가 뒤집힌다** (2026-09-20 실제로 당함).
# 부모의 회전을 풀어서 로컬 증분으로 바꾼다.
cache.Clear()
mid = np.array(compute_aabb(cache, plate_path, include_children=True), float)
want_edge = P[axis] + sign * args.max_half
have_edge = mid[axis] if sign < 0 else mid[axis + 3]
d_world = np.zeros(3)
d_world[axis] = want_edge - have_edge

parent = plate.GetParent()
pw = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(0)
Rp = np.array([[pw[0][0], pw[1][0], pw[2][0]],
               [pw[0][1], pw[1][1], pw[2][1]],
               [pw[0][2], pw[1][2], pw[2][2]]], float)
d_local = Rp.T @ d_world          # 회전만 푼다 (평행이동은 증분이라 무관)
say(f"보정: 월드 {np.round(d_world, 4).tolist()} → 로컬 {np.round(d_local, 4).tolist()}")
tr = tr + d_local
tr_op.Set(Gf.Vec3d(*[float(v) for v in tr]))
app.update()

cache.Clear()
b2 = np.array(compute_aabb(cache, plate_path, include_children=True), float)
edge2 = b2[axis] if sign < 0 else b2[axis + 3]
say(f"바뀐 판 x {b2[0]:+.3f}~{b2[3]:+.3f} ({b2[3]-b2[0]:.3f} m)  "
    f"y {b2[1]:+.3f}~{b2[4]:+.3f} ({b2[4]-b2[1]:.3f} m)")
say(f"서가 쪽 끝 {edge2:+.3f}, 팔에서 {abs(edge2 - P[axis]):.3f} m")
say(f"반대편 끝은 {b[axis+3] if sign < 0 else b[axis]:+.3f} → "
    f"{b2[axis+3] if sign < 0 else b2[axis]:+.3f} (유지되어야 한다)")

out = os.path.expanduser(args.out)
stage.Export(out)
say(f"저장 {out}")
app.close()
