"""레벨에서 로봇의 **yaw 를 맞춘다** (기본 0°).

왜 (2026-09-20):
    파지·삽입 경로 계산이 **월드 축 기준**으로 짜여 있다 — 칸은 월드 x 를 따라 늘어서고
    서가는 월드 +y 에 있다고 본다. 받은 에셋은 팔이 yaw 90° 로 서 있어 그 가정이 깨지고,
    `carry_rotate` 단계에서 IK 가 풀리지 않는다.

    제대로 된 해법은 경로 계산을 전부 **팔 기준**으로 바꾸는 것이지만 손대는 곳이 많다.
    시연 전날이라, 레벨에서 로봇을 돌려 **기존 수식이 성립하게** 한다.
    (배치는 우리 담당이고, 로봇이 어느 방향을 보는지는 시연 구성의 문제다)

**임시 조치다.** 경로 계산의 팔 기준 전환은 그대로 남아 있는 과제다.

실행
    ISAAC_ENTRY=simulation/isaac/tools/set_robot_yaw.py ./scripts/run_isaac_tool.sh \\
        --usd <레벨>.usd --out <출력>.usd --yaw 0
"""
import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--robot", default="/World/Nova_Carter_ROS")
ap.add_argument("--arm-base", default="m0609/base_link")
ap.add_argument("--yaw", type=float, default=0.0, help="팔 베이스의 목표 yaw (도)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


open_stage(os.path.expanduser(args.usd))
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()


def yaw_of(path):
    p = stage.GetPrimAtPath(path)
    m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
    R = np.array([[m[0][0], m[1][0], m[2][0]],
                  [m[0][1], m[1][1], m[2][1]],
                  [m[0][2], m[1][2], m[2][2]]], float)
    return math.atan2(float(R[1, 0]), float(R[0, 0])), np.array(m.ExtractTranslation(), float)


arm_path = f"{args.robot}/{args.arm_base}"
if not stage.GetPrimAtPath(arm_path).IsValid():
    say(f"팔 베이스를 못 찾음: {arm_path}")
    app.close()
    sys.exit(1)

y0, p0 = yaw_of(arm_path)
say(f"현재 팔 베이스 yaw {math.degrees(y0):+.1f}°, 위치 {np.round(p0, 3).tolist()}")
want = math.radians(args.yaw)
delta = want - y0
say(f"목표 {args.yaw:+.1f}° → {math.degrees(delta):+.1f}° 회전한다")

root = stage.GetPrimAtPath(args.robot)
xf = UsdGeom.Xformable(root)
ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
or_op = next((o for n, o in ops.items() if "orient" in n), None)
tr_op = next((o for n, o in ops.items() if "translate" in n), None)
if or_op is None or tr_op is None:
    say("로봇 루트에 orient/translate op 가 없다")
    app.close()
    sys.exit(1)

q = or_op.Get()
qw, qi = float(q.GetReal()), np.array(q.GetImaginary(), float)
dq = np.array([math.cos(delta / 2), 0.0, 0.0, math.sin(delta / 2)])
w1, x1, y1, z1 = dq
w2, x2, y2, z2 = qw, qi[0], qi[1], qi[2]
nq = np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
               w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])
or_op.Set(Gf.Quatd(float(nq[0]), Gf.Vec3d(*[float(v) for v in nq[1:]])))
app.update()

# 회전하면 팔 베이스가 옮겨진다 — 원래 자리로 되돌린다 (가정하지 않고 재서 보정)
y1n, p1 = yaw_of(arm_path)
tr = np.array(tr_op.Get(), float)
tr_op.Set(Gf.Vec3d(*[float(v) for v in (tr + (p0 - p1))]))
app.update()
y2n, p2 = yaw_of(arm_path)
say(f"결과: yaw {math.degrees(y2n):+.1f}°, 위치 {np.round(p2, 3).tolist()} "
    f"(오차 {float(np.linalg.norm(p2 - p0)) * 1000:.1f} mm)")

out = os.path.expanduser(args.out)
stage.Export(out)
say(f"저장 {out}")
app.close()
