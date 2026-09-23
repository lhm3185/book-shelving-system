"""서가 실측 → 비전팀 정답지. **레벨을 읽기만 한다** (아무것도 고치지 않는다).

## 무엇을 재나

빈칸을 검출하려면 "어디가 비었는가" 의 정답이 있어야 한다. 그 정답은 레벨에서
직접 재야 하고, 레벨은 바뀐다. 그래서 **레벨이 바뀔 때마다 다시 돌리는 도구**다.

재는 것 (전부 world 기준, m):

  ① 선반 판 z 와 **옆판 사이 안쪽 x 범위** — 책이 들어갈 수 있는 진짜 폭
  ② 판마다 책 목록 — x 범위, **자기 좌표계 두께**, 기울기
  ③ **진짜 틈** — 책 사이 + **양 끝 옆판과 책 사이**

## 앞서 틀렸던 것 (2026-09-23)

처음에는 이웃 책 AABB 간격만 보고 "빈칸은 원래 없다" 고 했다. **틀렸다.**
옆판과 책 사이를 안 봤고, 층을 하나만 봤다. 실제로는

  - 판 z 1.581: 책 사이에 81.1 / 59.9 / 36.8 / 36.1 mm 틈이 있다
  - 판 z 1.042: 책 사이는 붙어 있지만 **오른쪽 옆판과 96.0 mm** 틈이 있다

그래서 이 도구는 옆판을 먼저 찾고, 틈을 **옆판까지 포함해서** 센다.

## 난이도

틈 폭에서 우리가 꽂을 책 두께를 빼면 좌우 여유가 나오고, 그게 곧 **비전 검출
정확도 요구치**다. 좌우 여유가 5 mm 인 칸에 검출 오차 5 mm 면 실패가 나기 시작한다.

## 쓰기

    ISAAC_ENTRY=simulation/isaac/tools/measure_shelf.py ./scripts/run_isaac_tool.sh

    ~/isaacsim/python.sh simulation/isaac/tools/measure_shelf.py \\
        --usd ~/Desktop/ing_library_env_v5.usd \\
        --report docs/doyoon-kim/measurements/<날짜>_shelf_gaps.txt \\
        --yaml simulation/isaac/config/ground_truth_slots.yaml
"""
import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.environ.get("SIM_USD", ""))
ap.add_argument("--shelf", default="/World/bookshelves/shelf_brown__book_shelf_01",
                help="낱권 책이 있는 서가. 나머지 15개는 책이 통짜 메시 하나라 잴 것이 없다")
ap.add_argument("--floors", nargs="+", default=["thirdFloor", "secondFloor"],
                help="/World/books 아래 층 이름")
ap.add_argument("--report", default="", help="사람이 읽을 보고서 경로 (.txt)")
ap.add_argument("--yaml", default="", help="정답지 경로 (.yaml)")
ap.add_argument("--book-thick-mm", type=float, default=35.3, help="우리가 꽂을 책 두께")
ap.add_argument("--min-gap-mm", type=float, default=5.0, help="이보다 넓어야 '틈' 으로 센다")
a = ap.parse_args()

from isaacsim import SimulationApp                                    # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np                                                    # noqa: E402
from pxr import Usd, UsdGeom                                          # noqa: E402
from isaacsim.core.utils.stage import (open_stage, get_current_stage,  # noqa: E402
                                       is_stage_loading)

open_stage(os.path.expanduser(a.usd))
while is_stage_loading():
    app.update()
st = get_current_stage()
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

_rep = open(a.report, "w", encoding="utf-8") if a.report else None


def w(m=""):
    print(m)
    if _rep:
        _rep.write(str(m) + "\n")


def aabb(prim):
    cache.Clear()
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return np.array([mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]], float)


def local_size(prim):
    """책 **자기 좌표계** 치수. 돌아앉은 책은 AABB 가 부풀어 두께를 못 믿는다."""
    lo = hi = None
    inv = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).GetInverse()
    for p in Usd.PrimRange.AllPrims(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for q in pts:
            v = inv.Transform(m.Transform(q))
            arr = np.array([v[0], v[1], v[2]], float)
            lo = arr if lo is None else np.minimum(lo, arr)
            hi = arr if hi is None else np.maximum(hi, arr)
    return None if lo is None else (hi - lo)


def upright_tilt_deg(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    up = np.array([m[0][2], m[1][2], m[2][2]], float)
    n = float(np.linalg.norm(up))
    return float("nan") if n < 1e-9 else math.degrees(math.acos(min(1.0, abs(up[2] / n))))


# ------------------------------------------------------------------ ① 선반 뼈대
shelf = st.GetPrimAtPath(a.shelf)
mesh = None
for p in Usd.PrimRange.AllPrims(shelf):
    if p.IsA(UsdGeom.Mesh):
        mesh = p
        break
if mesh is None:
    sys.exit(f"서가 메시를 못 찾았다: {a.shelf}")
xf = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
wp = [xf.Transform(q) for q in UsdGeom.Mesh(mesh).GetPointsAttr().Get()]
xs = sorted({round(float(q[0]), 4) for q in wp})
zs = sorted({round(float(q[2]), 4) for q in wp})
# 옆판은 양 끝의 얇은 판이다 — 고유 x 값 바깥 두 개씩이 그 두께다
inner = (xs[1], xs[-2]) if len(xs) >= 4 else (xs[0], xs[-1])
# 선반판 윗면: 연달아 붙은 z 쌍(아랫면·윗면) 중 **윗값**
boards = []
i = 0
while i < len(zs) - 1:
    if 0.02 < zs[i + 1] - zs[i] < 0.08:
        boards.append(zs[i + 1])
        i += 2
    else:
        i += 1

w("=" * 78)
w(f"① 서가 뼈대  {a.shelf}")
w("=" * 78)
w(f"  서가 전체 x {min(float(q[0]) for q in wp):.4f} ~ {max(float(q[0]) for q in wp):.4f}")
w(f"  **안쪽 x 범위 {inner[0]:.4f} ~ {inner[1]:.4f}  (폭 {inner[1]-inner[0]:.4f} m)**")
w(f"  선반판 윗면 z {boards}")
w(f"  (고유 z 전부: {zs})")

# ------------------------------------------------------------------ ②③ 층별
floors = {}
for name in a.floors:
    sc = st.GetPrimAtPath(f"/World/books/{name}")
    if not sc.IsValid():
        w(f"\n[건너뜀] /World/books/{name} 없음")
        continue
    rows = []
    for c in sc.GetChildren():
        b = aabb(c)
        if b is None or np.any(b[3:] - b[:3] <= 0):
            continue
        ls = local_size(c)
        rows.append({"name": c.GetName(), "bb": b,
                     "thick": float(min(ls)) if ls is not None else float("nan"),
                     "tilt": upright_tilt_deg(c)})
    if not rows:
        continue
    rows.sort(key=lambda r: r["bb"][0])
    board = min(boards, key=lambda z: abs(z - float(rows[0]["bb"][2]))) if boards else None

    w("")
    w("=" * 78)
    w(f"② {name}  선반판 z {board}  — {len(rows)}권")
    w("=" * 78)
    w("  %-3s %-26s %8s %8s %7s %7s %6s  %s"
      % ("#", "이름", "x최소", "x최대", "AABB폭", "실두께", "기울기", "앞 책과"))
    gaps = []
    for i, r in enumerate(rows):
        b = r["bb"]
        g = None if i == 0 else float(b[0] - rows[i - 1]["bb"][3])
        if g is not None:
            gaps.append({"before_index": i, "width": g,
                         "x_min": float(rows[i - 1]["bb"][3]), "x_max": float(b[0])})
        w("  %-3d %-26s %8.4f %8.4f %7.4f %7.4f %5.1f°  %s"
          % (i, r["name"][-26:], b[0], b[3], b[3] - b[0], r["thick"], r["tilt"],
             "-" if g is None else f"{g*1000:+8.1f} mm"))
    # 양 끝 옆판과의 틈 — **이것을 빠뜨려서 "빈칸이 없다" 고 잘못 봤다**
    edge_l = {"before_index": 0, "width": float(rows[0]["bb"][0] - inner[0]),
              "x_min": float(inner[0]), "x_max": float(rows[0]["bb"][0]), "edge": "left"}
    edge_r = {"before_index": len(rows), "width": float(inner[1] - rows[-1]["bb"][3]),
              "x_min": float(rows[-1]["bb"][3]), "x_max": float(inner[1]), "edge": "right"}
    w("")
    w(f"  왼쪽 옆판 ~ 첫 책    **{edge_l['width']*1000:+.1f} mm**")
    w(f"  마지막 책 ~ 오른쪽 옆판 **{edge_r['width']*1000:+.1f} mm**")
    allg = [edge_l] + gaps + [edge_r]
    real = sorted([g for g in allg if g["width"] * 1000 >= a.min_gap_mm],
                  key=lambda g: -g["width"])
    t = a.book_thick_mm / 1000.0
    w(f"  {a.min_gap_mm:.0f} mm 넘는 틈 {len(real)}개 "
      f"(우리 책 {a.book_thick_mm:.1f} mm 기준 좌우 여유):")
    for g in real:
        if "edge" in g:
            where = "왼쪽 옆판" if g["edge"] == "left" else "오른쪽 옆판"
        else:
            where = f"책 #{g['before_index'] - 1} 와 #{g['before_index']} 사이"
        w(f"    {g['width']*1000:7.1f} mm  x {g['x_min']:.4f}~{g['x_max']:.4f}  "
          f"좌우 여유 {(g['width']-t)/2*1000:+6.1f} mm  {where}")
    floors[name] = {"board": board, "books": rows, "gaps": real, "inner": inner}

# ------------------------------------------------------------------ 정답지
if a.yaml:
    t = a.book_thick_mm / 1000.0
    os.makedirs(os.path.dirname(a.yaml) or ".", exist_ok=True)
    with open(a.yaml, "w", encoding="utf-8") as f:
        f.write("# 비전팀 정답지 — 서가 빈칸 검출용. measure_shelf.py 가 만든다\n")
        f.write("# **레벨이 바뀌면 다시 돌릴 것.** 좌표는 전부 world 기준, 단위 m\n")
        f.write(f"# 만든 레벨: {os.path.basename(os.path.expanduser(a.usd))}\n\n")
        f.write("shelf:\n")
        f.write(f"  prim: {a.shelf}\n")
        f.write(f"  inner_x_world: [{inner[0]:.4f}, {inner[1]:.4f}]\n")
        f.write(f"  board_z_world: {[round(b, 4) for b in boards]}\n")
        f.write("  note: >-\n")
        f.write("    낱권 책이 있는 서가는 이것 하나뿐이다. 나머지 15개는 책이 book_cube\n")
        f.write("    통짜 메시 하나라 빈칸 검출 시험이 되지 않는다. 낱권 책에는\n")
        f.write("    RigidBodyAPI 도 CollisionAPI 도 없어서 검출 결과를 물리로 확인할 수 없다.\n\n")
        f.write(f"inserted_book_thickness_m: {t:.4f}\n\n")
        f.write("floors:\n")
        for name, d in floors.items():
            f.write(f"  {name}:\n")
            f.write(f"    board_z_world: {d['board']}\n")
            f.write(f"    book_count: {len(d['books'])}\n")
            f.write("    gaps:            # 이미 **레벨에 있는** 빈칸. 우리가 만든 것이 아니다\n")
            for g in d["gaps"]:
                f.write(f"      - width_m: {g['width']:.4f}\n")
                f.write(f"        x_world: [{g['x_min']:.4f}, {g['x_max']:.4f}]\n")
                f.write(f"        center_x_world: {(g['x_min']+g['x_max'])/2:.4f}\n")
                f.write(f"        side_clearance_m: {(g['width']-t)/2:.4f}\n")
                f.write(f"        where: {g.get('edge', 'between books #' + str(g['before_index']-1) + ' and #' + str(g['before_index']))}\n")
            f.write("    books:\n")
            for i, r in enumerate(d["books"]):
                b = r["bb"]
                f.write(f"      - {{index: {i:2d}, x: [{b[0]:.4f}, {b[3]:.4f}], "
                        f"thickness: {r['thick']:.4f}, name: {r['name']}}}\n")
    w("")
    w(f"정답지를 썼다: {a.yaml}")

if _rep:
    _rep.close()
app.close()
