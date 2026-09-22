"""책 에셋의 **원점을 형상 중심(AABB 중심)으로 다시 잡는다.**

## 왜 필요한가

블렌더에서 월드 좌표계를 유지한 채 내보내서, 책 6권 모두 **prim 원점과 실제 형상이
따로 논다** (2026-09-21 실측).

| 파일 | 형상 중심(로컬) | 원점에서 |
| --- | --- | --- |
| book01 | `[0.000, -0.019, 0.119]` | 0.120 m |
| book06 | `[0.131, -0.019, 0.119]` | 0.178 m |

x 가 책마다 커지는 것이 "원래 나란히 꽂혀 있던 줄의 자리"를 그대로 들고 나온 증거다.

**무엇이 깨지나**: `book_scene` 의 배치는 AABB 로 보정하므로 놓는 위치는 맞다. 그러나
`attach()` 가 만드는 파지 고정 조인트는 **책 prim 원점**에 걸린다 — 손이 잡고 있는
자리에서 12~18 cm 떨어진 곳이다. 지렛대가 길면 운반 가속에서 구속이 밀리고,
그것이 `406 운반 중 손 안에서 책 어긋남` 의 모양과 맞는다.

좌표 계약(`book_profiles.yaml`)도 "**AABB 중심**(에셋 원점 아님)"이라고 적혀 있다.
원점을 중심으로 옮기면 **계약과 에셋이 같은 것을 가리키게 된다.**

## 어떻게 하나

원본을 건드리지 않는다. 원본을 참조하는 Xform 을 하나 씌우고 `-중심` 만큼 옮긴 뒤
**평탄화(flatten)해서 독립 파일**로 내보낸다. 원본이 그대로 있으므로 되돌릴 수 있다.

    ~/isaacsim/python.sh recenter_book_assets.py \
        --in  'isaac_sim/assets/book_dataset/usd_v2/*book0[1-6].usdc' \
        --out isaac_sim/assets/book_dataset/usd_v3

내보낸 뒤 **같은 방식으로 다시 재서** 중심이 원점에 왔는지 확인하고 출력한다.
0.5 mm 를 넘으면 실패로 본다 — 검사의 기본값이 '통과' 면 검사가 아니다.
"""
import argparse
import glob
import os
import sys

_APP = None
try:
    from pxr import Gf, Usd, UsdGeom
except ModuleNotFoundError:
    # PC 에 따라 USD 모듈이 Isaac 을 띄워야 잡힌다 (10.10.0.1 은 바로 되고 10.10.0.2 는 안 됐다).
    # 여기서 흡수한다 — 도구를 쓰는 쪽이 PC 차이를 알 필요는 없다. 약 1분 더 걸린다.
    from isaacsim import SimulationApp
    _APP = SimulationApp({"headless": True})
    from pxr import Gf, Usd, UsdGeom

TOL_M = 0.0005


def bbox_center(stage):
    """스테이지 기본 prim 의 형상 AABB 중심과 크기. 비어 있으면 (None, None)"""
    prim = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None, None
    lo, hi = Gf.Vec3d(rng.GetMin()), Gf.Vec3d(rng.GetMax())
    return (lo + hi) * 0.5, hi - lo


def recenter(src, dst_dir):
    """src 를 읽어 원점이 형상 중심에 오도록 옮긴 사본을 dst_dir 에 만든다"""
    name = os.path.basename(src)
    dst = os.path.join(dst_dir, name)
    src_abs = os.path.abspath(src)

    src_stage = Usd.Stage.Open(src_abs)
    if src_stage is None:
        return name, None, "열 수 없음"
    center, size = bbox_center(src_stage)
    if center is None:
        return name, None, "형상 경계가 없음"

    # 원본을 자식으로 참조하고 부모에서 -중심 만큼 옮긴다.
    # 부모에 옮김을 두는 이유: 원본 prim 이 이미 갖고 있는 xformOp 들과 섞이지 않는다.
    work = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(work, "/book")
    root.AddTranslateOp().Set(Gf.Vec3d(-center[0], -center[1], -center[2]))
    child = work.DefinePrim("/book/geo")
    child.GetReferences().AddReference(src_abs)
    work.SetDefaultPrim(root.GetPrim())

    flat = work.Flatten()
    flat.Export(dst)

    # **다시 재서 확인한다.** 내보낸 파일을 새로 열어 잰다 (메모리 스테이지 말고)
    chk = Usd.Stage.Open(dst)
    new_center, new_size = bbox_center(chk)
    if new_center is None:
        return name, None, "내보낸 파일에 형상이 없다"
    off = Gf.Vec3d(new_center).GetLength()
    if size is not None and new_size is not None:
        grew = max(abs(new_size[i] - size[i]) for i in range(3))
        if grew > 1e-4:
            return name, off, f"크기가 바뀌었다 ({grew*1000:.2f} mm)"
    return name, off, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="pattern", required=True, help="원본 glob")
    ap.add_argument("--out", required=True, help="내보낼 디렉터리")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.expanduser(a.pattern)))
    if not files:
        print(f"**원본을 못 찾았다**: {a.pattern}")
        return 2
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)

    bad = 0
    print(f"{'파일':52s} {'남은 어긋남':>12s}  결과")
    print("-" * 90)
    for f in files:
        name, off, err = recenter(f, out)
        if err:
            print(f"{name[:50]:52s} {'-':>12s}  **{err}**")
            bad += 1
            continue
        ok = off <= TOL_M
        print(f"{name[:50]:52s} {off*1000:9.3f} mm  {'OK' if ok else '**기준 초과**'}")
        if not ok:
            bad += 1

    print()
    if bad:
        print(f"**{bad}개 실패** — 원본은 그대로다. 쓰지 말 것")
        return 1
    print(f"{len(files)}개 전부 원점이 형상 중심에 왔다 → {out}")
    print("원본은 건드리지 않았다. 되돌리려면 --book-variants 를 예전 경로로 두면 된다")
    return 0


if __name__ == "__main__":
    _code = main()
    if _APP is not None:
        _APP.close()
    sys.exit(_code)
