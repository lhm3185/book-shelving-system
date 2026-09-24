"""판은 하나가 아니라 목록이다 — 칸 바닥 스냅의 산수.

2026-09-25 01:02: 위 판에 꽂을 때 칸 바닥을 아래 판 스칼라로만 스냅해서 계산값 1.1007 이
그대로 나갔다. 실제 판 1.0417 — 책이 59 mm 떨어져 5.4° 틀어졌다.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from shelf_gap import nearest_board  # noqa: E402

BOARDS = [0.4976, 1.0417]


def test_위_판_계산값은_위_판으로_간다():
    """그날 밤의 숫자 그대로: 계산 1.1007 → 실제 1.0417 (59 mm)."""
    assert nearest_board(1.1007, BOARDS) == 1.0417


def test_아래_판_계산값은_아래_판으로_간다():
    assert nearest_board(0.5566, BOARDS) == 0.4976


def test_너무_멀면_없다고_한다():
    assert nearest_board(0.80, BOARDS) is None
    assert nearest_board(1.1007, BOARDS, tol=0.05) is None


def test_가장_가까운_판을_고른다():
    assert nearest_board(0.77, [0.70, 0.80, 0.90]) == 0.80
