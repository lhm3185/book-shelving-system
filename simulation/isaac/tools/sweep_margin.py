"""관절 여유 지도 — 물리 실행 없이 IK 로만 훑는다 (계획 v5 §6-1 드라이런 스윕).

## 무엇을 묻나

> **베이스를 어디에 세우고 어느 선반 판에 꽂을 때, 관절 여유가 얼마나 남는가?**

야간 F-5 는 이 곡선의 **점 다섯 개**(GOAL_Y ±10 mm)만 봤고, 거기서 얻은 기울기
0.0015 rad/mm 로 "±30 mm 면 0.115" 를 외삽했다. 그 외삽은 믿을 수 없다 —
v09→v11 에서 "8 cm 필요" 예측이 실제 3 cm 였고, 원인은 **IK 가지가 j6→j1 로
바뀐 것**이었다. 가지가 바뀌면 곡선이 점프하므로 외삽이 원리적으로 불가능하다.

그래서 점이 아니라 **곡선**을 그린다.

## 왜 물리가 필요 없나

짝 규칙 `GOAL_Y = 0.5495 + (−3.019 − PICK_Y)` 의 뜻은 "베이스를 뒤로 Δ 옮기고
팔 기준 목표를 +Δ 옮기면 월드 삽입점이 그대로" 다. 뒤집으면:

> **팔에게는 '베이스를 Δ 뒤로 빼는 것' 과 '팔 기준 goal_y 를 +Δ 하는 것' 이 같은 일이다.**

팔은 자기가 월드 어디 있는지 모르고 팔 기준 목표만 본다. 그러니 베이스 오프셋
스윕은 **goal_y 스윕**이고, 차체를 움직일 필요도 물리를 돌릴 필요도 없다.
(레벨의 로봇 자세는 건드리지 않는다.)

## 어떻게

Isaac 을 **한 번만** 띄우고, 트레이 이송·정착까지만 물리를 돌린 뒤
(파지 기하가 실제와 같아야 approach·carry 여유가 의미 있다),
그 다음부터는 `plan_job` 만 반복 호출한다. 실행하지 않는다.

    SIM_USD=~/Desktop/ing_library_env_v5.usd \
    SIM_USE_LEVEL_TRAY=1 SIM_GRIP_ROT90=1 SIM_GRASP_KINEMATIC=1 \
    SIM_TRAY_SETTLE_S=2.0 SIM_TRAY_DELIVERY_S=6.0 \
    ~/isaacsim/python.sh sweep_margin.py --out /tmp/sweep.csv

`--gy` 는 **팔 기준** 목표 y (m). 기준은 demo_best 의 0.5795.
`--board` 는 **월드** 선반 판 z. 0.498 이 지금 판, 1.042 가 옮기려는 판.
"""
import argparse
import math
import os
import sys

REPO = os.path.expanduser(os.environ.get("SWEEP_REPO", "~/b1_arm"))

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.environ.get("SIM_USD", ""))
ap.add_argument("--out", default="/tmp/sweep.csv")
ap.add_argument("--gy", type=float, nargs=3, default=[0.5195, 0.6395, 0.005],
                help="팔 기준 목표 y: 시작 끝 간격 (m). 기본 = demo_best 0.5795 ±60mm @5mm")
ap.add_argument("--board", type=float, nargs="+",
                default=[0.498, 0.548, 0.598, 0.648, 0.698, 0.748,
                         0.798, 0.848, 0.898, 0.948, 0.998, 1.042],
                help="월드 선반 판 z. 0.498=지금 판, 1.042=옮기려는 판")
ap.add_argument("--goal-x", type=float, default=-0.3497, help="팔 기준 칸 x (FRANKA_SLOTS[0])")
ap.add_argument("--books", type=int, default=5)
ap.add_argument("--tray-center", type=float, nargs=2, default=[-0.4748, 0.0788])
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.51, -0.43, -0.35, -0.27])
ap.add_argument("--all-books", action="store_true",
                help="책 5권 각각으로 같은 스윕을 돌린다 (어느 책이 검증 실행과 같은 가지인지 가린다)")
ap.add_argument("--book-index", type=int, default=None,
                help="집을 책을 팔기준 x 순서로 고른다 (기본: 계약 0번 칸에 가장 가까운 것)")
ap.add_argument("--pick-book", default="",
                help="집을 책을 **이름 조각**으로 지정한다. 실제 판과 스윕이 어긋날 때 "
                     "가장 먼저 맞춰야 하는 것이 '어느 책인가' 다 — 책이 다르면 파지 해의 "
                     "IK 가지가 달라지고, 이 팔은 1 mm 에도 가지가 갈린다")
ap.add_argument("--pick-spot", type=float, nargs=3, default=[2.535, -3.049, 0.0],
                metavar=("X", "Y", "YAW_DEG"),
                help="이송 뒤 로봇을 여기로 통째로 옮긴다 (검증 실행의 주행 도착 지점). "
                     "빈 값 대신 --no-move 로 끌 것")
ap.add_argument("--no-move", action="store_true", help="로봇을 옮기지 않고 레벨 시작 자세 그대로 잰다")
ap.add_argument("--gy-ref", type=float, default=0.600,
                help="도착 오차 0 으로 볼 기준 gy. 검증 조합(SIM_PICK_Y=-3.0695)의 짝 규칙 값. "
                     "결과의 base_dy_mm 은 이 기준에서의 **상대** 도착 오차다 — 절대 월드 y 는 "
                     "베이스가 요청대로 안 서면 틀리므로 상대값을 본다")
ap.add_argument("--settle", type=float, default=12.0, help="이송+정착에 줄 시간(초)")
a = ap.parse_args()

# demo_best 기준값 — 판 0.498 일 때 팔 기준 goal_z. 판을 옮기면 같은 양만큼 더한다.
GZ_AT_498 = 0.3399
BASE_BOARD = 0.498
REF_GY = 0.5795          # demo_best
REF_PICK_Y = -3.049      # 그때 베이스 월드 y

from isaacsim import SimulationApp                                    # noqa: E402
app = SimulationApp({"headless": os.environ.get("SIM_GUI", "0") == "0"})
# 경로는 앱을 띄운 뒤에 넣는다 (SimulationApp 이 sys.path 를 갈아엎는다)
for _p in ("simulation/isaac", "simulation/isaac/controllers", "simulation/isaac/config"):
    sys.path.insert(0, os.path.join(REPO, _p))

import numpy as np                                                    # noqa: E402
import world_loader                                                   # noqa: E402
from book_scene import BookScene                                      # noqa: E402
from robot_profiles import profile                                    # noqa: E402

BOT = profile()
LOG = open(a.out + ".log", "w")


def say(m):
    LOG.write(str(m) + "\n")


def tell(m):
    sys.stderr.write("### " + str(m) + "\n")
    sys.stderr.flush()


usd = os.path.expanduser(a.usd) if a.usd else world_loader.resolve_usd(None)
tray = os.path.join(REPO, "simulation/assets/book_dataset/assets/tray/tray_v1.usdc")
tell(f"레벨 {usd}")
scene = BookScene(app, usd, tray, list(a.tray_center), a.books, list(a.place_dx), say)
world = scene.world

# --- 트레이 이송 + 정착까지만 물리를 돌린다 (여기까지가 유일한 물리) ---
tell(f"트레이 이송·정착 {a.settle:.0f}초…")
for _ in range(int(a.settle * 60)):
    world.step(render=False)
# --- 검증 실행과 같은 자세로 통째로 옮긴다 ---------------------------------
# 왜 필요한가: 레벨 시작 yaw 는 +90° 인데 검증 실행(v11~v15)은 **주행해서 yaw 0** 으로
# 도착한 뒤 집는다. 이 90° 차이가 IK 가지를 바꿔(제약 관절 1 → 6) 여유가 0.160 에서
# 0.004 로 떨어진다 — 팔 기준 명령이 같아도 그렇다. 그래서 자세를 맞춰야 잰 값이
# 검증 실행과 견줄 수 있다. 레벨 파일은 건드리지 않는다(런타임 이동일 뿐이다).
#
# 로봇만 옮기면 책이 제자리에 남으므로 **루트·트레이·책을 같은 강체 변환으로** 옮긴다.
# 강체라서 팔 기준 기하는 한 치도 변하지 않는다 — 바뀌는 것은 월드 자세뿐이다.
def _qmul(q1, q2):
    w1, x1, y1, z1 = q1; w2, x2, y2, z2 = q2
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def move_rig(target_xy, target_yaw_deg):
    from isaacsim.core.prims import SingleXFormPrim
    root = SingleXFormPrim(BOT.root)
    p0, q0 = root.get_world_pose()
    p0 = np.asarray(p0, float); q0 = np.asarray(q0, float)
    yaw0 = math.atan2(2*(q0[0]*q0[3] + q0[1]*q0[2]), 1 - 2*(q0[2]**2 + q0[3]**2))
    dyaw = math.radians(target_yaw_deg) - yaw0
    h = dyaw / 2.0
    dq = np.array([math.cos(h), 0.0, 0.0, math.sin(h)])
    c, sn = math.cos(dyaw), math.sin(dyaw)
    pivot_new = np.array([target_xy[0], target_xy[1], p0[2]])

    def xform(p, q):
        d = np.asarray(p, float) - p0
        return (pivot_new + np.array([c*d[0] - sn*d[1], sn*d[0] + c*d[1], d[2]]),
                _qmul(dq, np.asarray(q, float)))

    movers = [BOT.root] + list(scene.books)
    tray_path = getattr(scene, "tray_path", None) or globals().get("KIOSK_TRAY")
    for extra in (tray_path, getattr(scene, "tray", None)):
        if isinstance(extra, str) and extra and extra not in movers:
            movers.append(extra)
    n = 0
    for path in movers:
        try:
            pr = SingleXFormPrim(path)
            pp, qq = pr.get_world_pose()
            np_, nq_ = xform(pp, qq)
            pr.set_world_pose(np_, nq_)
            n += 1
        except Exception as exc:                       # noqa: BLE001
            tell(f"  옮기기 실패 {path}: {type(exc).__name__}")
    tell(f"통째 이동: yaw {math.degrees(yaw0):+.2f}° → {target_yaw_deg:+.2f}° · "
         f"루트 [{p0[0]:.3f},{p0[1]:.3f}] → [{target_xy[0]:.3f},{target_xy[1]:.3f}] · 프림 {n}개")


# **옮기기 전후로 팔 베이스를 잰다.** 루트 Xform 을 옮긴다고 팔 베이스가 그만큼
# 따라오지 않는다 — 관절이 달린 물리 바디라서 솔버가 되돌린다. 2026-09-24 에 스윕이
# 요청보다 **21 mm 앞에 선 채로** 도착 오차를 재고 있었다. ±30 mm 를 묻는 도구가
# 21 mm 오차를 갖고 있었던 것이다. 그래서 믿지 않고 잰다.
scene.refresh_base()
_base_before = np.asarray(scene.l0p, float).copy()
if not a.no_move:
    from isaacsim.core.prims import SingleXFormPrim as _SXP
    _root_before = np.asarray(_SXP(BOT.root).get_world_pose()[0], float).copy()
    move_rig(a.pick_spot[:2], a.pick_spot[2])
    for _ in range(120):                    # 짧게 정착 (강체 이동이라 크게 흔들릴 일은 없다)
        world.step(render=False)
    scene.refresh_base()
    _want = np.asarray([a.pick_spot[0], a.pick_spot[1]], float) - _root_before[:2]
    _got = np.asarray(scene.l0p, float)[:2] - _base_before[:2]
    _err = _got - _want
    tell(f"팔 베이스 이동: 요청 {np.round(_want*1000, 1).tolist()} mm  "
         f"실제 {np.round(_got*1000, 1).tolist()} mm  차이 {np.round(_err*1000, 1).tolist()} mm")
    if float(np.max(np.abs(_err))) > 0.002:
        tell(f"[경고] **루트를 옮겼는데 팔 베이스가 그만큼 안 따라왔다** "
             f"({float(np.max(np.abs(_err)))*1000:.1f} mm). 이 판의 절대 base_y 는 믿지 말 것 — "
             f"base_dy_mm(기준 gy {a.gy_ref:.4f} 에서의 상대 오차)로 읽어라. "
             f"--no-move 로 돌리면 이 오차가 없다")

scene.refresh_base()
tell(f"이송 상태 _deliver={bool(getattr(scene, '_deliver', None))} "
     f"(None 이어야 이송 끝)")
tell(f"팔 베이스 월드 {np.round(scene.l0p, 4).tolist()}  "
     f"시작관절 {np.round(np.asarray(scene.q_home, float), 3).tolist()}")
# **스윕과 실제 판을 견주려면 이 줄부터 같아야 한다.** 2026-09-24 에 같은 조합이라
# 믿고 견주다가, 실제로 같았는지 확인할 방법이 없어 한 시간을 썼다.
_SWING_ENV = ("SIM_CARRY_MODE", "SIM_RETURN_MODE", "SIM_SWING_CLEAR_M", "SIM_HOME_Q",
              "SIM_GRIP_ROT90", "SIM_GRASP_KINEMATIC", "SIM_MAX_STEP", "SIM_SPEED_SCALE",
              "SIM_JOINT_SEGS", "SIM_PICK_Y", "SIM_GOAL_Y")
tell("동선 스위치 (실제 판과 한 줄씩 맞춰 볼 것):")
for _k in _SWING_ENV:
    tell(f"    {_k:<22} {os.environ.get(_k, '(없음)')}")

# 트레이·책이 어디 있는지 먼저 찍는다. **팔 기준**이 같으면 베이스가 어디 있든
# 경로가 같다(random_pose_check 가 검증한 성질) — 그래서 로봇을 옮기지 않는다.
tell("트레이·책 위치 (팔 기준):")
rows = []
for b in scene.books:
    c_arm = scene.to_arm(scene.center(b))
    rows.append((float(c_arm[0]), b, c_arm))
# v11~v15 비전 입력 (팔기준 윗면 중심). **옛 레벨의 값이다** — 레벨이 바뀌면 이 값에
# 가까운 책이 실제 판이 집는 책과 달라진다. 아래에서 거리를 찍고, 멀면 경고한다.
VERIFIED_TOP = np.array([-0.3622, 0.0054, 0.1953])
tops = []
for x, b, c in sorted(rows, key=lambda r: -r[2][1]):
    bb = scene.aabb(b)
    top = scene.to_arm([(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, bb[5]])
    d = float(np.linalg.norm(top - VERIFIED_TOP))
    tops.append((d, b, top))
    tell(f"    {b.rsplit('/', 1)[-1][-26:]:<26} 중심 [{c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}] "
         f"윗면 [{top[0]:+.4f},{top[1]:+.4f},{top[2]:+.4f}] 검증값과 {d*100:5.1f}cm")
tops.sort()
tell(f"검증 실행(v11~v15)의 비전 입력 팔기준 {VERIFIED_TOP.tolist()} 에 "
     f"가장 가까운 책: {tops[0][1].rsplit('/', 1)[-1][-26:]} ({tops[0][0]*100:.1f}cm)")

# 집을 책: 기본은 계약 0번 칸에 가장 가까운 것 (없으면 그냥 가장 가까운 것을 쓴다)
pick_arm = np.array([-0.6623, 0.0788, 0.1119])
pick_w = scene.to_world(pick_arm)
cands = sorted(((float(np.linalg.norm(scene.center(b) - pick_w)), b) for b in scene.books))
if a.all_books:
    picks = [b for _d, b, _t in tops]          # 검증값에 가까운 순
elif a.pick_book:
    picks = [b for b in scene.books if a.pick_book in b]
    if not picks:
        tell(f"--pick-book '{a.pick_book}' 에 맞는 책이 없다. 있는 책: "
             f"{[b.rsplit('/', 1)[-1] for b in scene.books]}")
        sys.exit(2)
    if len(picks) > 1:
        tell(f"--pick-book '{a.pick_book}' 가 {len(picks)}권에 걸린다 — 더 좁혀라: "
             f"{[b.rsplit('/', 1)[-1] for b in picks]}")
        sys.exit(2)
elif a.book_index is not None:
    picks = [sorted(rows)[a.book_index][1]]
else:
    picks = [tops[0][1]]                        # 검증 실행과 가장 비슷한 책
tell(f"집을 책 {len(picks)}권: {[b.rsplit('/', 1)[-1][-18:] for b in picks]}")
# **어느 책을 집는가가 결과를 가른다.** 실제 판과 스윕이 어긋나면 여기부터 맞춘다.
if not a.pick_book and not a.all_books and tops and tops[0][0] > 0.03:
    tell(f"[경고] 고른 책이 검증값(VERIFIED_TOP, 옛 레벨)에서 {tops[0][0]*100:.1f} cm 떨어져 있다 — "
         f"레벨이 바뀌어 **실제 판이 집는 책과 다를 수 있다.** "
         f"실제 판과 견주려면 --pick-book 으로 같은 책을 지정할 것")

lo, hi = scene._arm_limits()
if lo is None:
    tell("관절 한계를 못 읽었다 — 여유를 잴 수 없다")
    sys.exit(2)
tell(f"관절 한계 lo {np.round(lo, 4).tolist()}")
tell(f"          hi {np.round(hi, 4).tolist()}")

SEGS = ["approach", "down", "lift", "carry_rotate", "wedge", "back",
        "touch", "push", "retreat", "return"]
INSERT = {"wedge", "touch", "push", "retreat"}


def margins(segs):
    """구간별 (최소여유, 관절번호). _audit_plan 과 같은 계산."""
    out = {}
    for name in SEGS:
        qs = segs.get(name)
        if not qs or len(qs) < 2:
            continue
        arr = np.asarray(qs, float)
        n = min(arr.shape[1], len(lo))
        m = np.minimum(arr[:, :n] - lo[:n], hi[:n] - arr[:, :n])
        mi = np.unravel_index(int(np.argmin(m)), m.shape)
        out[name] = (float(m[mi]), int(mi[1]) + 1)
    return out


def branch_of(q):
    fn = getattr(scene, "elbow_branch", None)
    return fn(np.asarray(q, float)) if fn else None


gys = [round(a.gy[0] + i * a.gy[2], 6)
       for i in range(int(round((a.gy[1] - a.gy[0]) / a.gy[2])) + 1)]
tell(f"스윕 {len(a.board)} 판 × {len(gys)} 점 = {len(a.board) * len(gys)} 계획")
tell(f"  판(월드 z) {a.board}")
tell(f"  팔기준 goal_y {gys[0]:.4f} … {gys[-1]:.4f} @{a.gy[2]:.3f}")

out = open(a.out, "w")
# q1_lift·q1_end 를 넣는 이유: 스윙(`SIM_CARRY_MODE=swing`)의 성패는 **들어올린
# 순간의 관절1 값 하나**로 갈린다 — 거기서 Δφ 를 더해 한계를 넘으면 401 이다.
# 이 값이 없으면 "스윕은 401 인데 실제 판은 됐다" 를 볼 방법이 없다 (2026-09-24).
# base_dy_mm: **기준 gy 에서의 상대 도착 오차** (양수 = 서가에서 멀어짐).
# base_y_equiv_world 는 상수에서 나오므로 베이스가 요청대로 안 서면 틀린다 —
# 도착 오차를 읽을 때는 **상대값** 쪽을 본다 (2026-09-24).
out.write("book,board_z_world,gz_arm,gy_arm,base_y_equiv_world,code,err,"
          "min_all,min_all_seg,min_all_joint,min_insert,min_insert_seg,min_insert_joint,"
          "q1_lift,q1_end,base_dy_mm,"
          + ",".join(f"m_{s}" for s in SEGS) + ",branch_down\n")
out.flush()

done = 0
total = len(picks) * len(a.board) * len(gys)
for book in picks:
  bname = book.rsplit("/", 1)[-1][-18:]
  for board in a.board:
      gz = GZ_AT_498 + (board - BASE_BOARD)
      for gy in gys:
          done += 1
          # 이 gy 가 대응하는 베이스 월드 y (짝 규칙의 역)
          base_y = REF_PICK_Y - (gy - REF_GY)
          place_w = scene.to_world(np.array([a.goal_x, gy, gz]))
          if done == 1 and (os.environ.get("SIM_PLAN_INPUTS", "")
                            or os.environ.get("SIM_SAY_PLAN_INPUTS", "0") != "0"):
              # **첫 점에서 한 번만.** 실제 판의 같은 블록과 한 줄씩 견주라고 있는 것이다
              try:
                  scene.say_plan_inputs(book, place_w, tag="스윕")
              except Exception as _exc:                  # noqa: BLE001
                  tell(f"[계획입력] 실패 {type(_exc).__name__}: {_exc}")
          try:
              plan, code, err = scene.plan_job(book, place_w)
          except Exception as exc:                       # noqa: BLE001
              plan, code, err = None, -1, f"{type(exc).__name__}: {exc}"
          if plan is None:
              out.write(f"{bname},{board:.3f},{gz:.4f},{gy:.4f},{base_y:.4f},{code},"
                        f"\"{str(err)[:120]}\",,,,,,,," + f"{-(gy - a.gy_ref)*1000:+.1f},"
                        + ",".join([""] * (len(SEGS) + 1)) + "\n")
              out.flush()
              if done % 10 == 0 or code != 401:
                  tell(f"[{done}] {bname} 판 {board:.3f} gy {gy:.4f} → 실패 {code} {str(err)[:70]}")
              continue
          m = margins(plan["segs"])
          allm = min(m.items(), key=lambda kv: kv[1][0])
          ins = {k: v for k, v in m.items() if k in INSERT}
          insm = min(ins.items(), key=lambda kv: kv[1][0]) if ins else ("", (float("nan"), 0))
          bd = any(branch_of(q) == "down" for qs in plan["segs"].values() for q in qs)
          _lift = plan["segs"].get("lift") or []
          _rot = plan["segs"].get("carry_rotate") or []
          _q1l = f"{float(_lift[-1][0]):+.4f}" if len(_lift) else ""
          _q1e = f"{float(_rot[-1][0]):+.4f}" if len(_rot) else ""
          row = [bname, f"{board:.3f}", f"{gz:.4f}", f"{gy:.4f}", f"{base_y:.4f}", "0", "\"\"",
                 f"{allm[1][0]:.4f}", allm[0], str(allm[1][1]),
                 f"{insm[1][0]:.4f}", insm[0], str(insm[1][1]), _q1l, _q1e,
                 f"{-(gy - a.gy_ref)*1000:+.1f}"]
          row += [f"{m[s][0]:.4f}" if s in m else "" for s in SEGS]
          row.append("1" if bd else "0")
          out.write(",".join(row) + "\n")
          out.flush()
          if done % 10 == 0:
              tell(f"[{done}/{total}] {bname} 판 {board:.3f} gy {gy:.4f} "
                   f"→ 전체 {allm[1][0]:.3f}(관절{allm[1][1]},{allm[0]}) "
                   f"삽입 {insm[1][0]:.3f}(관절{insm[1][1]})")

out.close()
LOG.close()
tell(f"끝. {a.out}")
app.close()
