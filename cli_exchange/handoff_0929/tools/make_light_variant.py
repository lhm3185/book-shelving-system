#!/usr/bin/env python3
"""레벨의 RectLight 두 값만 바꾼 사본을 만든다 (원본은 건드리지 않는다).

    python3 make_light_variant.py <레벨.usdc> <낼 파일.usdc> [intensity] [scale_z]

기본값은 2026-09-29 의 light1 이다: intensity 8500 (원래 8000), scale z 0.2 (원래 0.1).
네 권 레벨(md5 8fc1e9f5…)에 걸면 md5 e63aeca1… 이 나온다 — 교육장 두 PC 의 기본 레벨이다.

RectLight 는 로컬 x·y 평면의 면이라 scale z 는 빛나는 넓이를 바꾸지 않는다. 화면 밝기는 약 1.5% 올랐다(손목 카메라, 같은 구도).
"""
import hashlib
import shutil
import sys

from pxr import Gf, Sdf

LIGHT = "/Environment/RectLight"


def main(src, dst, intensity=8500.0, scale_z=0.2):
    shutil.copy(src, dst)
    layer = Sdf.Layer.FindOrOpen(dst)
    prim = layer.GetPrimAtPath(LIGHT)
    if not prim:
        sys.exit(f"조명이 없다: {LIGHT}")
    scale = prim.attributes["xformOp:scale"]
    power = prim.attributes["inputs:intensity"]
    print("전", scale.default, power.default)
    scale.default = Gf.Vec3d(scale.default[0], scale.default[1], float(scale_z))
    power.default = float(intensity)
    print("후", scale.default, power.default)
    layer.Save()
    print("md5:", hashlib.md5(open(dst, "rb").read()).hexdigest())


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4, 5):
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], *[float(v) for v in sys.argv[3:]])
