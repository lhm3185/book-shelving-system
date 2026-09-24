"""reach_check 의 산수 — 운반 경유점 y 유도와 판정 논리. IK 는 안 돌린다(느리다).

2026-09-25: 위 판 401 의 정체가 꽂는 자세가 아니라 **운반 경유점 `transfer`** 였다.
reach_check 는 꽂는 자세만 보고 "쓸 수 있다" 고 했다. 이제 경유점까지 본다.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

from reach_check import (MEASURED_INSET, TIP_DOWN, carry_waypoints)  # noqa: E402


def test_경유점_y_는_book_scene_의_식과_같다():
    """y_front = 중심 − W/2 − 0.024 · y_pre = y_front − (W − 0.035) − 0.03 · transfer = y_pre − 0.02."""
    y_t, y_p = carry_waypoints(0.5439, 0.1517)
    y_front = 0.5439 - 0.1517 / 2 - MEASURED_INSET
    assert y_p == pytest.approx(y_front - (0.1517 - TIP_DOWN) - 0.03)
    assert y_t == pytest.approx(y_p - 0.02)
    assert y_p == pytest.approx(0.2974, abs=5e-4)          # 2026-09-25 오프라인 표의 값


def test_책이_넓으면_경유점이_몸쪽으로_온다():
    y_t1, _ = carry_waypoints(0.5439, 0.1517)
    y_t2, _ = carry_waypoints(0.5439, 0.20)
    assert y_t2 < y_t1
