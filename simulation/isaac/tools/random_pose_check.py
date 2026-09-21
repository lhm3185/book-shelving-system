"""로봇을 **아무 데나 세워도 파지·반납이 똑같이 되는지** 잰다.

## 무엇을 묻는 시험인가

우리 IK·FK·경로는 오랫동안 **월드 좌표**로 짜여 있었다. 칸은 월드 x 로 늘어서고
서가는 월드 +y 에 있다고 **가정**했다. 그래서 로봇이 다른 자리·다른 방향에 서면
같은 명령이 다른 결과를 냈다. AMR 이 붙으면 그 가정은 무너진다.

이 시험은 한 가지만 묻는다:

> **팔 기준으로 똑같은 명령을 주면, 로봇이 어디에 어떤 각도로 서 있든
> 팔 기준 경로가 같은가?**

같으면 좌표계가 팔 기준으로 제대로 돌아선 것이다. 다르면 월드 가정이 남아 있다.

## 어떻게 재나

Isaac 을 **한 번만** 띄우고, 로봇 루트를 무작위 자세로 옮겨 가며 잰다
(매번 다시 띄우면 한 번에 4분씩 든다).

각 시도마다
  1. 로봇을 (x, y, yaw) 무작위로 옮기고 잠깐 정착시킨다
  2. `scene.refresh_base()` — 팔 베이스 자세를 다시 읽는다
  3. **팔 기준** 파지·반납 좌표(계약값)를 월드로 바꿔 `plan_job` 을 부른다
  4. 나온 경로를 **다시 팔 기준으로** 되돌려, 0번 시도와 비교한다

3~4 가 핵심이다. 팔 기준으로 넣고 팔 기준으로 꺼내므로, 월드 자세가 어떻든
**결과가 같아야 한다.** 이것이 합격 판정이다.

    ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/random_pose_check.py \\
      ./scripts/run_isaac_tool.sh --usd <레벨> --trials 8 --span 1.5 --yaw-span 180

`--yaw-span 0` 으로 먼저 돌려 **위치만** 바꿔 보고, 그게 통과하면 각도를 넣는다.
둘을 한꺼번에 돌리면 어느 쪽이 깨뜨렸는지 못 가른다.
"""
import argparse
import math
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.environ.get("SIM_USD", ""))
ap.add_argument("--trials", type=int, default=6)
ap.add_argument("--span", type=float, default=1.0, help="위치를 흔드는 폭 (m, ±)")
ap.add_argument("--yaw-span", type=float, default=0.0, help="방향을 흔드는 폭 (도, ±)")
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--settle", type=float, nargs="+", default=[1.5],
                help="로봇을 옮긴 뒤 refresh_base 까지 기다릴 시간(초). 여러 개면 비교한다")
ap.add_argument("--tol-rad", type=float, default=0.01,
                help="관절각이 이만큼까지 같으면 같은 경로로 본다 (0.01 rad ≈ 0.57°)")
ap.add_argument("--books", type=int, default=6)
ap.add_argument("--book-variants", default="")
ap.add_argument("--tray-center", type=float, nargs=2, default=[-0.4748, 0.0788])
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27])
a = ap.parse_args()

from isaacsim import SimulationApp                                    # noqa: E402
app = SimulationApp({"headless": os.environ.get("SIM_GUI", "0") == "0"})
# **경로는 앱을 띄운 뒤에 넣는다.** SimulationApp 이 sys.path 를 갈아엎어서
# 앞에 넣어 두면 `No module named 'world_loader'` 로 죽는다 (2026-09-21 실측).
for _p in ("simulation/isaac", "simulation/isaac/controllers", "simulation/isaac/config"):
    sys.path.insert(0, os.path.join(REPO, _p))

import numpy as np                                                    # noqa: E402
from isaacsim.core.prims import SingleXFormPrim                       # noqa: E402

import world_loader                                                   # noqa: E402
from book_scene import BookScene                                      # noqa: E402
from robot_profiles import profile                                    # noqa: E402

BOT = profile()


def say(m):
    sys.stderr.write("### " + str(m) + "\n")
    sys.stderr.flush()


# 계약값 — book_profiles.yaml 과 test_contract_coords.py 가 쓰는 바로 그 좌표다.
# **팔 기준(arm_base_link)** 이라 로봇이 어디 서 있든 같은 값이어야 한다.
PICK_ARM = np.array([-0.6623, 0.0788, 0.1119])          # 트레이 0번 칸
PLACE_ARM = {"m0609": np.array([-0.4697, 0.5495, 0.5097]),
             "franka": np.array([-0.5097, 0.5495, 0.3400])}.get(BOT.name,
                                                                np.array([-0.4697, 0.5495, 0.5097]))

usd = os.path.expanduser(a.usd) if a.usd else world_loader.resolve_usd(None)
tray = os.path.join(REPO, "simulation/assets/book_dataset/assets/tray/tray_v1.usdc")
variants = [v for v in a.book_variants.split(",") if v] or None
scene = BookScene(app, usd, tray, list(a.tray_center), a.books, list(a.place_dx), say,
                  **({"book_variants": variants} if variants else {}))
world = scene.world
root = SingleXFormPrim(BOT.root)
p0, q0 = root.get_world_pose()
p0 = np.asarray(p0, float)
q0 = np.asarray(q0, float)

rng = np.random.default_rng(a.seed)


def set_pose(dx, dy, dyaw_deg, settle_s=1.5):
    """로봇 루트를 원래 자리에서 (dx, dy) 옮기고 dyaw 만큼 돌린 뒤 **정착시킨다**.

    왜 기다리나: Nav2 가 멈춘 직후에는 베이스가 아직 흔들린다. 그 순간 `refresh_base()` 를
    읽으면 **흔들린 자세가 기준**이 된다 — "상태를 언제 읽느냐" 계열의 함정이다.
    """
    h = math.radians(dyaw_deg) / 2.0
    dq = np.array([math.cos(h), 0.0, 0.0, math.sin(h)])
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = dq
    q = np.array([w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
                  w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
                  w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
                  w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0])
    root.set_world_pose(p0 + np.array([dx, dy, 0.0]), q)
    for _ in range(max(1, int(settle_s * 60))):
        world.step(render=False)


def arm_frame_path(plan):
    """경로의 관절각을 그대로 쓴다 — 관절각은 **이미 팔 기준**이다.

    월드 가정이 남아 있으면 같은 팔 기준 명령이라도 관절각이 달라진다.
    """
    out = []
    for name in ("approach", "down", "lift", "carry_rotate", "wedge", "push"):
        seg = plan["segs"].get(name)
        if seg:
            out.append((name, np.asarray(seg[-1], float)))
    return out


def branch_of(q):
    """팔꿈치 가지. 6축은 해가 8개뿐이고 **가지가 전부를 정한다** (웹 클로드 v21 회신 §1)"""
    fn = getattr(scene, "elbow_branch", None)
    return fn(np.asarray(q, float)) if fn else None


def trial(dx, dy, dyaw, settle_s=1.5):
    set_pose(dx, dy, dyaw, settle_s)
    moved = scene.refresh_base()
    pick_w = scene.to_world(PICK_ARM)
    place_w = scene.to_world(PLACE_ARM)
    book, dist = scene.book_on_tray_near(pick_w)
    if book is None:
        near = sorted(float(np.linalg.norm(scene.center(b) - pick_w)) for b in scene.books)
        return None, f"트레이에서 책을 못 찾음 (가장 가까운 것 {near[0]*100:.1f}cm)"
    # **팔 기준으로 되돌려 찍는다.** 시도마다 같아야 한다 — 다르면 월드 값이 새고 있다
    say(f"      팔기준  home_tip {np.round(scene.to_arm(scene.home_tip), 4).tolist()}  "
        f"책중심 {np.round(scene.to_arm(scene.center(book)), 4).tolist()}  "
        f"q_home[0] {scene.q_home[0]:+.4f}  yaw {math.degrees(scene.tray_yaw):+.1f}°")
    plan, code, err = scene.plan_job(book, place_w)
    if plan is None:
        return None, f"계획 실패 M{code} {err}"
    # **가지 불변 단언** — 모든 경유점이 팔꿈치↑ 여야 한다. 하나라도 ↓ 면 받침판에 닿는다
    bad_br = []
    for _n, _qs in plan["segs"].items():
        for _i, _q in enumerate(_qs):
            if branch_of(_q) == "down":
                bad_br.append(f"{_n}[{_i}]")
                break
    if bad_br:
        return None, f"팔꿈치↓ 가 섞였다: {', '.join(bad_br[:4])}"
    return (arm_frame_path(plan), book, dist, moved), None


say("=" * 78)
say(f"무작위 배치 시험 — {a.trials}회, 위치 ±{a.span}m, 방향 ±{a.yaw_span}°, 허용 {a.tol_rad} rad")
say(f"팔 기준 명령 (고정): 파지 {PICK_ARM.tolist()} → 반납 {PLACE_ARM.tolist()}")
say("=" * 78)

base = None
rows = []
for i in range(a.trials):
    if i == 0:
        dx = dy = dyaw = 0.0                 # 0번은 기준점 — 원래 자리 그대로
    else:
        dx = float(rng.uniform(-a.span, a.span))
        dy = float(rng.uniform(-a.span, a.span))
        dyaw = float(rng.uniform(-a.yaw_span, a.yaw_span))
    res, err = trial(dx, dy, dyaw, a.settle[0])
    tag = f"{i:2d}  dx{dx:+.2f} dy{dy:+.2f} yaw{dyaw:+6.1f}°"
    if err:
        rows.append((tag, None, err))
        say(f"{tag}  **{err}**")
        continue
    path, book, dist, moved = res
    if base is None:
        base = path
        rows.append((tag, 0.0, f"기준점 ({book.rsplit('/', 1)[-1]}, 칸에서 {dist*100:.1f}cm)"))
        say(f"{tag}  기준점 — 관절각을 이후 시도와 견준다")
        continue
    worst, worst_name = 0.0, ""
    for (n0, q_base), (n1, q_now) in zip(base, path):
        d = float(np.max(np.abs(q_now - q_base)))
        if d > worst:
            worst, worst_name = d, n0
    rows.append((tag, worst, f"{worst_name} 에서 최대"))
    ok = worst <= a.tol_rad
    say(f"{tag}  관절각 최대차 {worst:.5f} rad ({worst_name})  "
        f"{'같다' if ok else '**다르다**'}")

say("=" * 78)
bad = [r for r in rows if r[1] is None or r[1] > a.tol_rad]
if not bad:
    say(f"**{a.trials}회 전부 같은 경로가 나왔다.** 좌표계가 팔 기준으로 돌아섰다")
else:
    say(f"**{len(bad)}/{a.trials} 회가 다르다** — 월드 가정이 남아 있다")
    for tag, w, note in bad:
        say(f"    {tag}  {note}")
if len(a.settle) > 1 and base is not None:
    say("")
    say("=" * 78)
    say("정착 시간별 비교 — **언제부터 같아지는가**가 필요한 대기 시간이다")
    say("=" * 78)
    ref = None
    for t in a.settle:
        r2, e2 = trial(0.0, 0.0, 0.0, t)
        if e2:
            say(f"  {t:4.1f}초  **{e2}**")
            continue
        path2 = r2[0]
        if ref is None:
            ref = path2
            say(f"  {t:4.1f}초  기준")
            continue
        w = max(float(np.max(np.abs(b - c))) for (_, b), (_, c) in zip(ref, path2))
        say(f"  {t:4.1f}초  관절각 최대차 {w:.5f} rad")

say("주의: 이 시험은 **계획**만 본다. 실제로 꽂히는지는 따로 봐야 한다")
app.close()
sys.exit(1 if bad else 0)
