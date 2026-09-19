"""결합 로봇을 **실제로 재생해** 넘어지는지·튀어 나가는지 본다.

왜: 정지 상태 검사(DOF·IK)만으로는 "재생하면 넘어진다"를 못 잡는다. 실제 증상은
    고정 조인트가 어긋나 있을 때 물리가 시작 순간 팔을 끌어당기면서 나온다 (2026-09-18).

무엇을 보나
    - 베이스가 처음 자리에서 얼마나 밀렸는가 (수 cm 이상이면 이상)
    - 로봇이 얼마나 기울었는가 (기울기 각도)
    - 첫 스텝에서 속도가 튀지 않는가

실행
    ISAAC_ENTRY=simulation/isaac/tools/settle_test_carter.py ./scripts/run_isaac_tool.sh \\
        --usd ~/carter_m0609_handoff/carter_m0609.usd --seconds 5
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/carter_m0609_handoff/carter_m0609.usd"))
ap.add_argument("--robot", default="/World/carter_m0609")
ap.add_argument("--seconds", type=float, default=5.0)
ap.add_argument("--ground", choices=["on", "off"], default="on", help="바닥을 깔고 시험한다")
# 에셋마다 구조가 다르다 (우리 결합체는 carter/chassis_link, AMR 담당 것은 chassis_link)
ap.add_argument("--base-link", default="carter/chassis_link", help="베이스 링크 (robot 기준 상대경로)")
ap.add_argument("--arm-link", default="arm/m0609/base_link", help="팔 베이스 링크 (robot 기준 상대경로)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
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

# 바닥은 **로봇 바닥 높이에** 깐다. z=0 에 깔면 바퀴가 14 cm 파묻힌 채 시작해
# 물리가 밀어내면서 로봇이 튀어 나간다 — 로봇 탓으로 오해하기 쉽다 (2026-09-18 실측)
cache = create_bbox_cache()
cache.Clear()
bb = np.array(compute_aabb(cache, args.robot, include_children=True), float)
floor = float(bb[2])
say(f"로봇 밑면 z {floor:+.4f} → 바닥을 그 높이에 깐다")

world = World(stage_units_in_meters=1.0)
if args.ground == "on":
    GroundPlane(prim_path="/World/settle_ground", size=20.0,
                position=np.array([0.0, 0.0, floor]))
world.reset()

base = SingleXFormPrim(args.robot + "/carter/chassis_link")
arm = SingleXFormPrim(args.robot + "/arm/m0609/base_link")


def state():
    bp, bq = base.get_world_pose()
    ap_, _ = arm.get_world_pose()
    return np.array(bp, float), np.array(bq, float), np.array(ap_, float)


b0, q0, a0 = state()
say(f"시작  베이스 {np.round(b0, 4).tolist()}  팔 {np.round(a0, 4).tolist()}")
say(f"      팔이 베이스 위로 {np.round(a0 - b0, 4).tolist()}")

steps = int(args.seconds * 60)
worst = 0.0
for i in range(steps):
    world.step(render=False)
    if i in (1, 5, 30) or (i + 1) % 120 == 0:
        b, q, a = state()
        move = float(np.linalg.norm(b - b0))
        worst = max(worst, move)
        # 기울기: 몸통의 z 축이 월드 z 에서 얼마나 벗어났는가
        w, x, y, z = q
        up_z = 1.0 - 2.0 * (x * x + y * y)
        tilt = float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))
        gap = float(np.linalg.norm((a - b) - (a0 - b0)))
        say(f"  {(i + 1) / 60:5.2f}s  베이스 이동 {move * 100:7.2f} cm  기울기 {tilt:6.2f}°  "
            f"팔-베이스 어긋남 {gap * 1000:7.2f} mm")

b, q, a = state()
move = float(np.linalg.norm(b - b0))
w, x, y, z = q
tilt = float(np.degrees(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))
gap = float(np.linalg.norm((a - b) - (a0 - b0)))
say("")
if move < 0.05 and tilt < 5.0 and gap < 0.005:
    say(f"**정상** — 이동 {move * 100:.2f} cm, 기울기 {tilt:.2f}°, 팔 어긋남 {gap * 1000:.2f} mm")
else:
    say(f"**이상** — 이동 {move * 100:.2f} cm, 기울기 {tilt:.2f}°, 팔 어긋남 {gap * 1000:.2f} mm")
    say("  이동·어긋남이 크면 고정 조인트 로컬 좌표를, 기울기만 크면 무게중심을 본다")

app.close()
