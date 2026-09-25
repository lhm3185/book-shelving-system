#!/usr/bin/env python3
"""레벨을 주면 **각 선반 판에 꽂을 수 있는지** 시뮬 없이 답한다.

왜: 2026-09-24 에 "3·4번 선반에 한 권씩" 이 위 판 401 로 막혔다. 그걸 알아내는 데
GPU 판 여러 개를 썼는데, **빈칸 위치와 팔 도달은 둘 다 시뮬 없이 잴 수 있다.**

    python3 reach_check.py <레벨.usd>
    python3 reach_check.py <레벨.usd> --thickness 0.0352 --insert-y 0.5439

레벨이 바뀌면 **이것부터 다시 돌린다.** 판 좌표·책 배치가 달라지면 답도 달라진다.

**서가 prim 이름을 믿지 말고 좌표로 확인한다.** 2026-09-25 새 레벨에서 `shelf_01`/`shelf_11` 이름이
서로 맞바뀌었다 — 작업 서가(x +1.87~+3.28)가 `_11`, 새 서가(x -1.37~+0.04)가 `_01`. `--shelf` 로 준 prim
의 안쪽 x 범위가 출력 둘째 줄에 찍히니 그걸로 어느 서가인지 본다.

`pxr`(USD)·`numpy` 가 시스템 파이썬에 없으면 그게 있는 가상환경으로 돈다 — 데스크탑은
`/tmp/usdenv/bin/python3 simulation/isaac/tools/reach_check.py <레벨.usd>` (2026-09-25 확인).

한계: 팔 기구학은 `arm_kinematics`(Franka 전용 DH)를 쓴다. 다른 로봇은 못 잰다.
      그리고 여기서 "풀린다" 는 **끝점**이지 경로가 아니다 — 경로는 중간에 더
      좁은 데를 지날 수 있다 (위 판이 실제로 그랬다: 끝점 0.217, 경로 중간 0.091).
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak          # noqa: E402
from shelf_gap import BOOKS_ROOTS, boards_from_zs, book_in_shelf, is_floor_group, gaps as find_gaps   # noqa: E402

#: 꽂을 때 손 자세 — 서가(+Y)를 향하고 물림축이 +X (`SIM_GRIP_ROT90=1` 조합)
R_INSERT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
#: 검증된 삽입 x (팔 기준). 차체가 옆으로 움직여 빈칸을 **늘 이 x 로** 가져온다
PLACE_X = -0.35
#: 팔 베이스 월드 z (2026-09-24 실측 유도). 레벨이 바뀌면 다시 재야 한다
ARM_BASE_Z = 0.330
#: 선반판 윗면 → 꽂힌 책 중심까지 (m). 실측 두 값의 차이로 낸다:
#: 아래 판 윗면이 팔기준 0.168 이고 검증된 삽입 z 가 0.3399 다.
BOARD_TO_SLOT_Z = 0.3399 - 0.168
#: 손을 아래로 향한 자세 (파지·운반 첫 구간). 물림축 +X — R_INSERT 와 같은 조합
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
#: 운반 경유점 — book_scene 의 정의를 그대로 옮긴다 (그쪽이 바뀌면 여기도 바꾼다):
#:   y_front  = 꽂힌 책 중심 y − W/2 − MEASURED_INSET(0.024)
#:   y_pre    = y_front − (W − TIP_DOWN(0.035)) − 0.03
#:   transfer = (place_x, y_pre − 0.02, 칸중심 z + 0.06)   손 아래(DOWN)로 들른다
#:   pre_ins  = (place_x, y_pre,        칸중심 z + 0.01)   HORIZ
#: 위 판 401 의 정체가 이 transfer 였다 (2026-09-25 00:00): 칸 자체는 풀리는데 경유점이 안 풀렸다.
MEASURED_INSET, TIP_DOWN = 0.024, 0.035
TRANSFER_DZ, PRE_INS_DZ = 0.06, 0.01
#: 씨앗 자세 둘 — 어느 쪽에서든 풀리면 풀리는 것으로 본다
SEEDS = (
    np.array([2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033]),        # yaml 홈
    np.array([-2.3634, -0.73, -0.6933, -2.1884, -0.4396, 1.5845, -0.554]),  # SIM_HOME_Q
)


def read_shelf(usd_path, shelf_prim):
    """레벨에서 `(선반판 윗면 z 목록, 안쪽 x 범위, 판별 책 목록)`."""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise SystemExit(f"레벨을 못 연다: {usd_path}")
    shelf = stage.GetPrimAtPath(shelf_prim)
    if not shelf or not shelf.IsValid():
        raise SystemExit(f"서가 prim 이 없다: {shelf_prim}")
    mesh = next((p for p in Usd.PrimRange.AllPrims(shelf) if p.IsA(UsdGeom.Mesh)), None)
    if mesh is None:
        raise SystemExit("서가 메시를 못 찾았다")
    xf = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    wp = [xf.Transform(q) for q in UsdGeom.Mesh(mesh).GetPointsAttr().Get()]
    xs = sorted({round(float(q[0]), 4) for q in wp})
    inner = (xs[1], xs[-2]) if len(xs) >= 4 else (xs[0], xs[-1])
    boards = boards_from_zs(float(q[2]) for q in wp)
    # **이 서가의 상자** — 책을 판 높이로만 거르면 다른 서가의 책이 섞인다. 2026-09-25 새 레벨
    # (서가 둘, 같은 판 높이)에서 두 서가 사이 3.28 m 가 "빈칸" 으로 나왔다(데스크탑 실측).
    shelf_bb = (min(float(q[0]) for q in wp), min(float(q[1]) for q in wp), min(float(q[2]) for q in wp),
                max(float(q[0]) for q in wp), max(float(q[1]) for q in wp), max(float(q[2]) for q in wp))
    books = {}
    root = next((stage.GetPrimAtPath(r) for r in BOOKS_ROOTS if stage.GetPrimAtPath(r).IsValid()), None)
    if root and root.IsValid():
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=True)
        # 층 그룹(`…Floor…`)의 자식이 낱권 책. 깊이는 레벨마다 다르다(옛: 루트/층, 새: 루트/서가/층)
        floors = [p for p in Usd.PrimRange(root) if is_floor_group(p.GetName()) and p != root]
        for floor in floors:
            for c in floor.GetChildren():
                r = cache.ComputeWorldBound(c).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                lo, hi = r.GetMin(), r.GetMax()
                bb = (float(lo[0]), float(lo[1]), float(lo[2]), float(hi[0]), float(hi[1]), float(hi[2]))
                if not book_in_shelf(bb, shelf_bb):
                    continue                        # 다른 서가의 책
                books.setdefault(str(floor.GetName()), []).append(
                    (str(c.GetName()), float(lo[0]), float(hi[0]), float(lo[2])))
    return boards, inner, books


def margin_at(z_arm, y, R=R_INSERT):
    """그 점·그 손 자세가 **풀리는가** → 한계 여유 (못 풀면 `None`)."""
    best = None
    for q0 in SEEDS:
        r = ak.ik_best(np.array([PLACE_X, y, z_arm]), R, ak.seeds_around(q0), prefer=q0)
        if r.ok and (best is None or r.limit_margin > best.limit_margin):
            best = r
    return None if best is None else best.limit_margin


def carry_waypoints(insert_y, width):
    """꽂힌 책 중심 y 와 책 폭에서 **운반 경유점의 y** 둘 → `(transfer_y, pre_ins_y)`."""
    y_front = insert_y - width / 2.0 - MEASURED_INSET
    y_pre = y_front - (width - TIP_DOWN) - 0.03
    return y_pre - 0.02, y_pre


def carry_check(z_slot, insert_y, width):
    """운반 경유점이 풀리는가 — `(transfer@DOWN, transfer@HORIZ, pre_ins@HORIZ)` 여유.

    첫 값이 풀리면 지금 경로 그대로다. 안 풀리고 둘째가 풀리면 자세 사다리 2번째
    (`arm_planning.carry_ladder`)로 간다. 셋째가 안 풀리면 꽂는 자세 앞에서 막힌다.
    """
    y_t, y_p = carry_waypoints(insert_y, width)
    return (margin_at(z_slot + TRANSFER_DZ, y_t, R_DOWN),
            margin_at(z_slot + TRANSFER_DZ, y_t, R_INSERT),
            margin_at(z_slot + PRE_INS_DZ, y_p, R_INSERT))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("--shelf", default="/World/bookshelves/shelf_brown__book_shelf_01")
    ap.add_argument("--thickness", type=float, default=0.0352, help="책 두께 (m)")
    ap.add_argument("--clearance", type=float, default=0.005, help="한쪽 여유 (m)")
    ap.add_argument("--insert-y", type=float, default=0.5439, help="꽂힌 책 중심의 팔 기준 y")
    ap.add_argument("--width", type=float, default=0.1517, help="책 폭 = 꽂는 깊이 방향 치수 (m)")
    ap.add_argument("--arm-base-z", type=float, default=ARM_BASE_Z)
    a = ap.parse_args()

    boards, inner, books = read_shelf(a.level, a.shelf)
    need = a.thickness + 2 * a.clearance
    print(f"레벨  {a.level}")
    print(f"서가  안쪽 x {inner[0]:.4f} ~ {inner[1]:.4f}  (폭 {inner[1] - inner[0]:.4f} m)")
    print(f"책    두께 {a.thickness * 1000:.1f} mm · 한쪽 여유 {a.clearance * 1000:.1f} mm "
          f"→ 필요 폭 **{need * 1000:.1f} mm**")
    print()
    y_t, y_p = carry_waypoints(a.insert_y, a.width)
    print(f"운반  경유점 y  transfer {y_t:.4f} · pre_ins {y_p:.4f}  (팔 기준, 꽂힌 책 y {a.insert_y} 에서 유도)")
    print()
    print(f"{'판(월드 z)':>12}{'팔기준 z':>10}{'책':>5}{'들어가는 빈칸':>26}{'꽂기':>8}"
          f"{'경유 DOWN':>10}{'경유 HORIZ':>11}{'pre_ins':>9}  판정")
    print("-" * 100)
    usable = 0
    for bz in boards:
        z_arm = bz - a.arm_base_z
        rows = []
        for floor, items in books.items():
            if items and abs(min(i[3] for i in items) - bz) < 0.10:
                rows = [(n, lo, hi) for n, lo, hi, _z in items]
                break
        if not rows:
            print(f"{bz:>12.4f}{z_arm:>10.4f}{0:>5}{'(책 없음 — 못 잼)':>26}{'—':>10}  ?")
            continue
        fits = [g for g in find_gaps(rows, *inner) if g.width >= need]
        widest = max((g.width for g in fits), default=0.0)
        # **판 윗면이 아니라 꽂힌 책 중심 높이**로 푼다. 둘은 172 mm 다르다
        # (실측: 판 윗면 팔기준 0.168 ↔ 검증된 삽입 z 0.3399).
        z_slot = z_arm + BOARD_TO_SLOT_Z
        m = margin_at(z_slot, a.insert_y)
        t_down, t_horiz, p_ins = carry_check(z_slot, a.insert_y, a.width)
        f = lambda v: f"{v:.3f}" if v is not None else "✗"      # noqa: E731
        # 꽂는 자세가 풀리고, 운반 경유점이 사다리 어느 단에서든 풀리고, pre_ins 가 풀려야 쓴다
        carry_ok = (t_down is not None) or (t_horiz is not None)
        ok = bool(fits) and m is not None and carry_ok and p_ins is not None
        usable += int(ok)
        if ok:
            verdict = "**쓸 수 있다**" + ("" if t_down is not None else " (자세 사다리 2번째)")
        elif not fits:
            verdict = "못 쓴다 — 빈칸 없음"
        elif m is None:
            verdict = "못 쓴다 — 꽂는 자세 안 풀림"
        elif p_ins is None:
            verdict = "못 쓴다 — pre_ins 안 풀림"
        else:
            verdict = "못 쓴다 — 운반 경유점 안 풀림"
        print(f"{bz:>12.4f}{z_arm:>10.4f}{len(rows):>5}"
              f"{(f'{len(fits)}개 · 최대 {widest * 1000:.0f} mm' if fits else '없음'):>26}"
              f"{f(m):>8}{f(t_down):>10}{f(t_horiz):>11}{f(p_ins):>9}  {verdict}")
    print()
    print(f"→ **쓸 수 있는 판 {usable}개.** 두 권을 다른 판에 꽂으려면 2 이상이어야 한다.")
    print("  주의: 여기 '풀린다' 는 **점**이다. 가는 경로는 더 좁은 데를 지날 수 있다")
    print("        (2026-09-24 위 판: 끝점 0.217 인데 경로 중간 0.091 에서 401 — 그게 경유 DOWN 이었다).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
