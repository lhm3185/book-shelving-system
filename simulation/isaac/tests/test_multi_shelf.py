"""서가 둘 — 명령의 shelf_id 로 현재 서가를 고르고, 다른 서가의 책은 세지 않는다.

2026-09-25 다음 그림(서가 둘·네 권). 매핑이 없으면 지금까지처럼 서가 하나 — 동작이 안 바뀐다.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from shelf_gap import BOOKS_ROOTS, book_in_shelf, is_floor_group, nearest_shelf, parse_shelf_prims  # noqa: E402

SHELF_A = [1.8, -2.9, 0.0, 3.3, -2.5, 2.6]      # x0 y0 z0 x1 y1 z1 (월드)


def test_매핑_문자열을_읽는다():
    m = parse_shelf_prims("shelf_01=/World/bookshelves/a; shelf_02=/World/bookshelves/b")
    assert m == {"shelf_01": "/World/bookshelves/a", "shelf_02": "/World/bookshelves/b"}
    assert parse_shelf_prims("") == {} and parse_shelf_prims(None) == {}
    assert parse_shelf_prims("shelf_01=/a,shelf_02=/b") == {"shelf_01": "/a", "shelf_02": "/b"}


def test_잘못된_항목은_버린다():
    assert parse_shelf_prims("shelf_01;=/x;shelf_02=") == {}


def test_이_서가의_책만_센다():
    inside = [2.4, -2.8, 0.5, 2.45, -2.6, 0.72]
    other = [5.0, -2.8, 0.5, 5.05, -2.6, 0.72]          # 같은 판 높이, 다른 서가
    assert book_in_shelf(inside, SHELF_A)
    assert not book_in_shelf(other, SHELF_A)


def test_서가_상자_가장자리는_여유_안이면_센다():
    edge = [3.30, -2.8, 0.5, 3.36, -2.6, 0.72]         # 중심 x 3.33 — 상자 x1 3.3 + 여유 0.05 안
    assert book_in_shelf(edge, SHELF_A)
    assert not book_in_shelf(edge, SHELF_A, margin=0.0)


def test_층_그룹은_이름으로_고른다():
    """옛 레벨 thirdFloor/forthFloor, 새 레벨 thirdFloor_01/forthFloor_01 — 깊이가 아니라 이름."""
    assert is_floor_group("thirdFloor") and is_floor_group("forthFloor_01")
    assert not is_floor_group("shelf_A") and not is_floor_group("books") and not is_floor_group("book_012")


def test_책_루트는_새_레벨을_먼저_본다():
    assert BOOKS_ROOTS[0] == "/World/bookshelves_main/books" and "/World/books" in BOOKS_ROOTS


SHELF_B = [-1.3727, 1.6692, 0.0, 0.0396, 1.9738, 2.52]
SHELF_A_BB = [1.8659, -2.5748, 0.0, 3.2782, -2.2702, 2.52]


def test_서_있는_자리로_서가를_고른다():
    """FSM 이 shelf_id 를 안 채우므로 팔 베이스 위치로 — B 앞(-0.704, 1.225)이면 B, A 앞(2.535, -3.019)이면 A."""
    boxes = {"shelf_01": SHELF_A_BB, "shelf_02": SHELF_B}
    assert nearest_shelf(boxes, (-0.704, 1.225))[0] == "shelf_02"
    assert nearest_shelf(boxes, (2.535, -3.019))[0] == "shelf_01"


def test_너무_멀면_안_고른다():
    name, dist = nearest_shelf({"shelf_01": SHELF_A_BB}, (4.986, -5.607))    # 홈에서 A 중심까지 ≈ 4.0 m
    assert name is None and dist > 3.0
    assert nearest_shelf({}, (0, 0)) == (None, None)
