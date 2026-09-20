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
ap.add_argument("--edge", action="append", default=None, metavar="방향:거리",
                help="변마다 '팔 베이스에서 판 끝까지의 거리'를 지정한다. 여러 번 줄 수 있다. "
                     "예: --edge -x:0.396 (서가 쪽) --edge -y:0.70 (트레이 쪽). "
                     "안 주면 서가 쪽 -x:0.396 만 적용")
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

specs = []
for spec in (args.edge or ["-x:0.396"]):
    d, v = spec.split(":")
    specs.append((0 if "x" in d else 1, -1 if d.startswith("-") else +1, float(v), d))

# Cube 는 size·scale·translate 로 크기가 정해진다. 한 변만 바꾸려면
# 스케일을 바꾸고 **반대편 끝이 그대로 있도록** 옮긴다.
xf = UsdGeom.Xformable(plate)
ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
sc_op = next((o for n, o in ops.items() if "scale" in n), None)
tr_op = next((o for n, o in ops.items() if "translate" in n), None)
if sc_op is None or tr_op is None:
    say("판에 scale/translate op 가 없다 — 수동 조정 필요")
    app.close()
    sys.exit(1)

parent = plate.GetParent()
pw = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(0)
Rp = np.array([[pw[0][0], pw[1][0], pw[2][0]],
               [pw[0][1], pw[1][1], pw[2][1]],
               [pw[0][2], pw[1][2], pw[2][2]]], float)

for axis, sign, want_half, label in specs:
    cache.Clear()
    b = np.array(compute_aabb(cache, plate_path, include_children=True), float)
    edge = b[axis] if sign < 0 else b[axis + 3]
    keep = b[axis + 3] if sign < 0 else b[axis]        # 반대편 끝 — 그대로 둬야 한다
    half = abs(edge - P[axis])
    if abs(half - want_half) < 1e-4:
        say(f"{label}: 이미 {want_half:.3f} m — 그대로 둔다")
        continue
    say(f"{label}: 팔에서 {half:.3f} m → {want_half:.3f} m "
        f"({'트림' if want_half < half else '확장'} {abs(want_half-half)*1000:.0f} mm)")

    sc = np.array(sc_op.Get(), float)
    tr = np.array(tr_op.Get(), float)
    span = b[axis + 3] - b[axis]
    new_span = span + (want_half - half)
    sc[axis] *= new_span / span
    sc_op.Set(Gf.Vec3f(*[float(v) for v in sc]))
    app.update()

    # 스케일은 prim 자기 원점 기준으로 바뀐다. 바꾼 뒤 실제 위치를 재서 보정한다.
    #
    # **주의**: translate 는 부모 좌표계 값이고, 이 판은 부모·자신 모두 180° yaw 가 걸려 있다.
    # 월드 보정량을 그대로 더하면 **부호가 뒤집힌다** (2026-09-20 실제로 당함).
    cache.Clear()
    mid = np.array(compute_aabb(cache, plate_path, include_children=True), float)
    have_keep = mid[axis + 3] if sign < 0 else mid[axis]
    d_world = np.zeros(3)
    d_world[axis] = keep - have_keep       # 반대편 끝을 원래 자리로 되돌린다
    tr = tr + Rp.T @ d_world
    tr_op.Set(Gf.Vec3d(*[float(v) for v in tr]))
    app.update()

    cache.Clear()
    b2 = np.array(compute_aabb(cache, plate_path, include_children=True), float)
    e2 = b2[axis] if sign < 0 else b2[axis + 3]
    k2 = b2[axis + 3] if sign < 0 else b2[axis]
    say(f"  → 팔에서 {abs(e2 - P[axis]):.3f} m, 반대편 {keep:+.3f}→{k2:+.3f} "
        f"{'OK' if abs(k2-keep) < 1e-3 else '**유지 실패**'}")

cache.Clear()
b2 = np.array(compute_aabb(cache, plate_path, include_children=True), float)
say("")
say(f"최종 판 x {b2[0]:+.3f}~{b2[3]:+.3f} ({b2[3]-b2[0]:.3f} m)  "
    f"y {b2[1]:+.3f}~{b2[4]:+.3f} ({b2[4]-b2[1]:.3f} m)")

out = os.path.expanduser(args.out)
stage.Export(out)
say(f"저장 {out}")
app.close()
