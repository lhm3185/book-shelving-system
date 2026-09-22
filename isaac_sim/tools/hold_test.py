"""팔을 **목표 관절각에 직접 놓고** 버티는지 본다 — 경로와 무관하게 자세만 검사한다.

## 무엇을 가르는 시험인가

경유점을 전부 보냈는데 관절이 남을 때, 원인은 둘 중 하나다.

| 결과 | 뜻 |
| --- | --- |
| 놓은 자리에 **머무른다** | 자세는 가능하다 → **경로·추종 문제** (가는 길에 뭔가 걸린다) |
| 놓자마자 **밀려난다** | 그 자세가 **애초에 불가능** (무언가와 겹쳐 있다) |

순간이동으로 놓고 목표도 같은 값으로 준 뒤 스텝을 돌린다. 밀려나면 그 방향과
**무엇과 겹치는지**를 같이 남긴다 — 관절값만 보면 어느 물체인지 영영 모른다.

    ARM_ROBOT=m0609 ISAAC_ENTRY=isaac_sim/isaac/tools/hold_test.py \\
      ./scripts/run_isaac_tool.sh --usd <레벨> \\
      --q -0.257 -0.968 -0.935 1.487 -1.814 -0.342 \\
      --also -0.257 -0.968 -0.935 -1.654 -4.469 2.800

`--also` 로 **같은 손 자세의 다른 손목 해**를 이어서 시험할 수 있다. 둘을 비교하면
"손목을 그쪽으로 돌리는 것 자체가 막히는가"를 가를 수 있다.
"""
import argparse
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.environ.get("SIM_USD", ""))
ap.add_argument("--q", nargs="+", type=float, required=True, help="검사할 관절각")
ap.add_argument("--also", nargs="+", type=float, action="append", default=[],
                help="이어서 검사할 다른 자세 (여러 번 줄 수 있다)")
ap.add_argument("--steps", type=int, default=240)
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--book-variants", default="")
ap.add_argument("--tray-center", type=float, nargs=2, default=[-0.4748, 0.0788])
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27])
a = ap.parse_args()

from isaacsim import SimulationApp                                    # noqa: E402
app = SimulationApp({"headless": os.environ.get("SIM_GUI", "0") == "0"})
# 경로는 앱을 띄운 뒤에 넣는다 — SimulationApp 이 sys.path 를 갈아엎는다
for _p in ("isaac_sim/isaac", "isaac_sim/isaac/controllers", "isaac_sim/isaac/config"):
    sys.path.insert(0, os.path.join(REPO, _p))

import numpy as np                                                    # noqa: E402

import world_loader                                                   # noqa: E402
from book_scene import BookScene                                      # noqa: E402


def say(m):
    sys.stderr.write("### " + str(m) + "\n")
    sys.stderr.flush()


usd = os.path.expanduser(a.usd) if a.usd else world_loader.resolve_usd(None)
tray = os.path.join(REPO, "isaac_sim/assets/book_dataset/assets/tray/tray_v1.usdc")
variants = [v for v in a.book_variants.split(",") if v] or None
scene = BookScene(app, usd, tray, list(a.tray_center), a.books, list(a.place_dx), say,
                  **({"book_variants": variants} if variants else {}))
world = scene.world
robot = scene.robot


def arm_now():
    return robot.get_joint_positions()[scene.idx_arm].copy()


def hold(q_t, label):
    q_t = np.asarray(q_t, float)
    say("=" * 74)
    say(f"[{label}] {np.round(q_t, 4).tolist()}  가지 {scene.elbow_branch(q_t)}")
    full = robot.get_joint_positions().copy()
    full[scene.idx_arm] = q_t
    robot.set_joint_positions(full)
    robot.set_joint_velocities(np.zeros_like(full))
    # 목표도 같은 값으로 준다 — 안 주면 이전 목표로 끌려가 시험이 무의미해진다
    try:
        robot.set_joint_position_targets(full)
    except Exception:      # noqa: BLE001 — 버전에 따라 이름이 다르다
        scene.set_arm_targets(q_t) if hasattr(scene, "set_arm_targets") else None
    say(f"{'스텝':>5s}  {'최대오차':>9s}   현재 관절각")
    steps = 0
    for mark in (1, 10, 30, 60, 120, a.steps):
        if mark < steps:
            continue
        while steps < mark:
            world.step(render=False)
            steps += 1
        n = arm_now()
        say(f"{steps:5d}  {float(np.max(np.abs(n - q_t))):9.4f}   {np.round(n, 3).tolist()}")
    n = arm_now()
    d = n - q_t
    j = int(np.argmax(np.abs(d)))
    say(f"  → 가장 많이 밀린 관절: joint_{j+1} {d[j]:+.4f} rad")
    rep = getattr(scene, "clash_report", None)
    if rep:
        try:
            say("  겹치는 것: " + str(rep()))
        except Exception as exc:      # noqa: BLE001
            say(f"  겹침 보고 실패: {exc}")
    return float(np.max(np.abs(d)))


err0 = hold(a.q, "기본")
for k, q in enumerate(a.also):
    hold(q, f"대안 {k + 1}")

say("")
say("판정: 오차가 0 근처로 유지되면 **자세는 가능**하다 → 경로·추종 문제.")
say("      계속 커지면 그 자세가 **불가능**하다 → 무엇과 겹치는지 위 보고를 볼 것.")
app.close()
