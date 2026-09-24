#!/usr/bin/env python3
"""레벨을 주면 **각 선반 판에 꽂을 수 있는지** 시뮬 없이 답한다.

왜: 2026-09-24 에 "3·4번 선반에 한 권씩" 이 위 판 401 로 막혔다. 그걸 알아내는 데
GPU 판 여러 개를 썼는데, **빈칸 위치와 팔 도달은 둘 다 시뮬 없이 잴 수 있다.**

    python3 reach_check.py <레벨.usd>
    python3 reach_check.py <레벨.usd> --thickness 0.0352 --insert-y 0.5439

레벨이 바뀌면 **이것부터 다시 돌린다.** 판 좌표·책 배치가 달라지면 답도 달라진다.

한계: 팔 기구학은 `arm_kinematics`(Franka 전용 DH)를 쓴다. 다른 로봇은 못 잰다.
      그리고 여기서 "풀린다" 는 **끝점**이지 경로가 아니다 — 경로는 중간에 더
      좁은 데를 지날 수 있다 (위 판이 실제로 그랬다: 끝점 0.217, 경로 중간 0.091).
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak          # noqa: E402
from shelf_gap import gaps as find_gaps   # noqa: E402

#: 꽂을 때 손 자세 — 서가(+Y)를 향하고 물림축이 +X (`SIM_GRIP_ROT90=1` 조합)
R_INSERT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
#: 검증된 삽입 x (팔 기준). 차체가 옆으로 움직여 빈칸을 **늘 이 x 로** 가져온다
PLACE_X = -0.35
#: 팔 베이스 월드 z (2026-09-24 실측 유도). 레벨이 바뀌면 다시 재야 한다
ARM_BASE_Z = 0.330
#: 선반판 윗면 → 꽂힌 책 중심까지 (m). 실측 두 값의 차이로 낸다:
#: 아래 판 윗면이 팔기준 0.168 이고 검증된 삽입 z 가 0.3399 다.
BOARD_TO_SLOT_Z = 0.3399 - 0.168
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
    zs = sorted({round(float(q[2]), 4) for q in wp})
    inner = (xs[1], xs[-2]) if len(xs) >= 4 else (xs[0], xs[-1])
    boards, i = [], 0
    while i < len(zs) - 1:
        if 0.02 < zs[i + 1] - zs[i] < 0.08:
            boards.append(zs[i + 1])
            i += 2
        else:
            i += 1
    books = {}
    root = stage.GetPrimAtPath("/World/books")
    if root and root.IsValid():
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=True)
        for floor in root.GetChildren():
            for c in floor.GetChildren():
                r = cache.ComputeWorldBound(c).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                lo, hi = r.GetMin(), r.GetMax()
                books.setdefault(str(floor.GetName()), []).append(
                    (str(c.GetName()), float(lo[0]), float(hi[0]), float(lo[2])))
    return boards, inner, books


def margin_at(z_arm, insert_y):
    """그 높이의 꽂기 직전 자세가 **풀리는가** → 한계 여유 (못 풀면 `None`)."""
    best = None
    for q0 in SEEDS:
        r = ak.ik_best(np.array([PLACE_X, insert_y, z_arm]), R_INSERT,
                       ak.seeds_around(q0), prefer=q0)
        if r.ok and (best is None or r.limit_margin > best.limit_margin):
            best = r
    return None if best is None else best.limit_margin


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("--shelf", default="/World/bookshelves/shelf_brown__book_shelf_01")
    ap.add_argument("--thickness", type=float, default=0.0352, help="책 두께 (m)")
    ap.add_argument("--clearance", type=float, default=0.005, help="한쪽 여유 (m)")
    ap.add_argument("--insert-y", type=float, default=0.5439, help="꽂힌 책 중심의 팔 기준 y")
    ap.add_argument("--arm-base-z", type=float, default=ARM_BASE_Z)
    a = ap.parse_args()

    boards, inner, books = read_shelf(a.level, a.shelf)
    need = a.thickness + 2 * a.clearance
    print(f"레벨  {a.level}")
    print(f"서가  안쪽 x {inner[0]:.4f} ~ {inner[1]:.4f}  (폭 {inner[1] - inner[0]:.4f} m)")
    print(f"책    두께 {a.thickness * 1000:.1f} mm · 한쪽 여유 {a.clearance * 1000:.1f} mm "
          f"→ 필요 폭 **{need * 1000:.1f} mm**")
    print()
    print(f"{'판(월드 z)':>12}{'팔기준 z':>10}{'책':>5}{'들어가는 빈칸':>26}{'한계여유':>10}  판정")
    print("-" * 78)
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
        m = margin_at(z_arm + BOARD_TO_SLOT_Z, a.insert_y)
        ms = f"{m:.3f}" if m is not None else "안 풀림"
        ok = bool(fits) and m is not None
        usable += int(ok)
        print(f"{bz:>12.4f}{z_arm:>10.4f}{len(rows):>5}"
              f"{(f'{len(fits)}개 · 최대 {widest * 1000:.0f} mm' if fits else '없음'):>26}"
              f"{ms:>10}  {'**쓸 수 있다**' if ok else '못 쓴다'}")
    print()
    print(f"→ **쓸 수 있는 판 {usable}개.** 두 권을 다른 판에 꽂으려면 2 이상이어야 한다.")
    print("  주의: 여기 '풀린다' 는 **끝점**이다. 가는 경로는 더 좁은 데를 지날 수 있다")
    print("        (2026-09-24 위 판: 끝점 0.217 인데 경로 중간 0.091 에서 401).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
