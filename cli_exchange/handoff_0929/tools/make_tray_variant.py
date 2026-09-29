#!/usr/bin/env python3
"""트레이 책 한 권을 뺀 레벨을 **따로 된 파일**로 만든다 (원본은 건드리지 않는다).

    python3 make_tray_variant.py <다섯 권 레벨.usdc> <낼 파일.usdc> <뺄 책>

뺄 책은 /World/tray_books 아래 prim 이름의 뒷부분이다. 예:
    book_hardcover_01_cover41_01          → 지금 쓰는 네 권 레벨 (md5 8fc1e9f5…)
    catalogue_hardcover_01_cover60_01     → 바꾼 판 swap41 (md5 1df62da4…)

다섯 권 레벨은 저장소 이력에서 꺼낸다 (md5 43c1f035…):
    git cat-file -p 26285f4:simulation/assets/level/Final_Level_Shaded/Final_Level_Shaded_robot.usdc \
        | git lfs smudge > five.usdc

루트 레이어에서 그 prim 하나만 지운다. 로봇 · 서가 · 조명 · 나머지 책은 글자 하나 안 바뀐다.
낸 파일은 textures/ · env_base_collider.usd · franka_camera.usd 와 **같은 폴더**에 둔다(상대 경로 참조).
"""
import hashlib
import shutil
import sys

from pxr import Sdf

TRAY = "/World/tray_books/decorative_book_set_01_2k__"


def main(src, dst, book):
    shutil.copy(src, dst)
    layer = Sdf.Layer.FindOrOpen(dst)
    spec = layer.GetPrimAtPath(TRAY + book)
    if not spec:
        sys.exit(f"그런 책이 없다: {TRAY + book}")
    del spec.nameParent.nameChildren[spec.name]
    layer.Save()
    left = [c.name for c in layer.GetPrimAtPath("/World/tray_books").nameChildren]
    print("남은 것:", left)
    print("md5:", hashlib.md5(open(dst, "rb").read()).hexdigest())


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
