#!/usr/bin/env python3
"""콜라이더가 **실제로 어디 있는지** 잰다 — Isaac 없이 돈다 (usd-core 만 있으면).

왜 필요한가 (2026-09-24): 꽂은 책이 놓는 순간 **16 mm 떠오른다.** 세 판이 모두
같은 높이(밑면 z 0.5155~0.5160)에 앉는데, 눈에 보이는 선반 판 윗면은 0.4976 이다.
운반 동안 책 충돌을 꺼 두므로 콜라이더를 통과해 내려갔다가, 놓으며 켜는 순간
밀려 올라간다는 그림이다. 그렇다면 **콜라이더 윗면이 시각 형상보다 18 mm 위**다.

눈에 보이는 것과 물리가 쓰는 것이 다를 수 있다. 시각 메시를 재고 물리를 논하면
안 된다 — 이 밤에 그 실수를 여러 번 했다.

    python3 measure_collider.py <레벨.usdc> --root /World/bookshelves
    python3 measure_collider.py <레벨.usdc> --root /World/bookshelves --near-z 0.51

`--near-z` 를 주면 그 높이 근처의 **윗면**만 골라 보여 준다 (선반 한 단을 찾을 때).
"""
import argparse
import sys

from pxr import Usd, UsdGeom, UsdPhysics


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("usd")
    ap.add_argument("--root", default="/World/bookshelves")
    ap.add_argument("--near-z", type=float, default=None,
                    help="이 높이 ±tol 안에 윗면이 있는 콜라이더만")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--near-x", type=float, default=None, help="이 x 를 품는 것만")
    a = ap.parse_args()

    st = Usd.Stage.Open(a.usd)
    if st is None:
        print(f"열 수 없다: {a.usd}")
        return 2
    # **시각 purpose 로만 잰다.** render/proxy 를 섞으면 물리가 안 쓰는 형상이 들어온다
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    root = st.GetPrimAtPath(a.root)
    if not root.IsValid():
        print(f"프림이 없다: {a.root}")
        return 2

    rows, n_prim, n_coll = [], 0, 0
    for p in Usd.PrimRange(root):
        n_prim += 1
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        n_coll += 1
        try:
            r = cache.ComputeWorldBound(p).ComputeAlignedRange()
            lo, hi = r.GetMin(), r.GetMax()
        except Exception as exc:      # noqa: BLE001
            print(f"  못 쟀다 {p.GetPath()}: {type(exc).__name__}")
            continue
        if a.near_z is not None and abs(hi[2] - a.near_z) > a.tol:
            continue
        if a.near_x is not None and not (lo[0] <= a.near_x <= hi[0]):
            continue
        approx = ""
        if p.HasAPI(UsdPhysics.MeshCollisionAPI):
            _at = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr()
            approx = str(_at.Get()) if _at and _at.HasAuthoredValue() else "(기본)"
        rows.append((float(hi[2]), str(p.GetPath()), lo, hi, approx))

    _filt = f" · 걸러서 {len(rows)}개" if len(rows) != n_coll else ""
    print(f"{a.root} 아래 프림 {n_prim}개 · CollisionAPI {n_coll}개{_filt}")
    if n_coll == 0:
        print("  **콜리전이 하나도 없다** — 물리로는 아무것도 막지 못한다")
        return 0
    rows.sort(reverse=True)
    print(f"  {'윗면 z':>9}  {'아랫면 z':>9}  {'x 범위':>21}  근사  경로")
    for top, path, lo, hi, approx in rows[:40]:
        print(f"  {top:9.4f}  {lo[2]:9.4f}  "
              f"[{lo[0]:8.4f},{hi[0]:8.4f}]  {approx:12s}  {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
