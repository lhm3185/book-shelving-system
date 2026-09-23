"""406 판정식 단위시험 — 세 자가 왜 10배씩 다른지 산수로 고정한다."""

import math
import os
import sys

import numpy as np

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from arm_geometry import quat_from_R  # noqa: E402
from hand_drift import amplification_mm, drift_between, hand_frame_points  # noqa: E402

#: 이 레벨 책의 실측 지렛대 (prim 원점 ↔ 형상중심). 2026-09-22 실측 75~177 cm
LEVER = 1.70
#: 책 좌표계에서 원점 → 형상중심
C_LOC = np.array([LEVER, 0.0, 0.0])
#: 형상중심 → 쥔점(윗면 중심)
UP_LOC, HZ = np.array([0.0, 0.0, 1.0]), 0.08

HAND_P, HAND_Q = np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])


def _rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _pts(book_p, R):
    return hand_frame_points(HAND_P, HAND_Q, book_p, quat_from_R(R), C_LOC, UP_LOC, HZ)


def test_nothing_moves_means_zero_everywhere():
    """아무것도 안 움직이면 세 기준 모두 0 이다 (기준선)."""
    ref = _pts(np.zeros(3), np.eye(3))
    d = drift_between(ref, _pts(np.zeros(3), np.eye(3)))
    assert (d.origin, d.center, d.grip, d.rot_deg) == pytest.approx((0, 0, 0, 0), abs=1e-12)


def test_rotating_about_the_grip_point_is_not_slipping():
    """쥔 점을 축으로 도는 것은 미끄러진 것이 아니다 — 쥔점 기준만 그렇게 말한다."""
    ref = _pts(np.zeros(3), np.eye(3))
    R = _rot_z(0.3)
    grip0 = C_LOC + UP_LOC * HZ                       # 책 좌표계의 쥔 점
    book_p = grip0 - R @ grip0                        # 쥔 점을 제자리에 고정
    d = drift_between(ref, _pts(book_p, R))
    assert d.rot_deg == pytest.approx(0.3, abs=1e-6)
    assert d.grip == pytest.approx(0.0, abs=1e-9)     # ← 미끄러지지 않았다
    # 원점은 1.7 m 떨어져 있어 그대로 부풀린다
    assert d.origin * 1000 == pytest.approx(amplification_mm(LEVER, 0.3), rel=0.02)
    # 형상중심은 이 경우 회전축 위에 있어 안 움직인다 — **기준이 옳아서가 아니라
    # 회전축이 하필 거기였기 때문이다.** 축이 조금만 달라지면 같이 부푼다
    assert d.center == pytest.approx(0.0, abs=1e-9)


def test_the_measured_case_reproduces():
    """9/24 실측: 회전 0.3°, 원점 0.1 cm, 형상중심 1.0 cm.

    원점이 1 mm 밖에 안 움직였다는 것은 **회전축이 원점 근처**였다는 뜻이다.
    그러면 1.7 m 떨어진 형상중심은 9 mm 움직인 것으로 찍힌다 — 실측 1.0 cm 다.
    """
    ref = _pts(np.zeros(3), np.eye(3))
    d = drift_between(ref, _pts(np.zeros(3), _rot_z(0.3)))   # 원점을 축으로 0.3°
    assert d.origin == pytest.approx(0.0, abs=1e-12)
    assert d.center * 100 == pytest.approx(0.89, abs=0.05)   # ≈ 1.0 cm
    assert d.grip * 100 == pytest.approx(0.89, abs=0.05)
    # 이 경우엔 쥔점도 같이 부풀지만, **회전각이 그대로 보고된다** — 0.3° 로 걸러야 한다
    assert d.rot_deg == pytest.approx(0.3, abs=1e-6)


def test_real_slip_shows_up_in_every_metric():
    """진짜로 미끄러지면(평행이동) 세 기준이 **같은 값**을 낸다 — 증폭이 없다."""
    ref = _pts(np.zeros(3), np.eye(3))
    d = drift_between(ref, _pts(np.array([0.0, 0.012, 0.0]), np.eye(3)))
    assert d.origin == pytest.approx(0.012, abs=1e-12)
    assert d.center == pytest.approx(0.012, abs=1e-12)
    assert d.grip == pytest.approx(0.012, abs=1e-12)
    assert d.rot_deg == pytest.approx(0.0, abs=1e-9)


def test_the_lever_is_the_whole_story():
    """지렛대 1.7 m 에서 1° 는 30 mm 다 — 문턱 10 mm 는 0.34° 에서 터진다."""
    assert amplification_mm(LEVER, 1.0) == pytest.approx(29.7, abs=0.1)
    assert amplification_mm(LEVER, 0.34) == pytest.approx(10.1, abs=0.2)
    # 쥔점 기준의 지렛대는 형상중심까지의 거리가 아니라 0 이다 (책과 함께 움직인다)
    assert amplification_mm(0.0, 5.0) == 0.0


def test_hand_rotation_alone_is_not_drift():
    """손이 돌아도 책이 같이 돌면 어긋남이 아니다 — 손 기준으로 보기 때문이다."""
    R = _rot_z(20.0)
    ref = hand_frame_points(HAND_P, np.array([1.0, 0.0, 0.0, 0.0]),
                            np.zeros(3), quat_from_R(np.eye(3)), C_LOC, UP_LOC, HZ)
    now = hand_frame_points(HAND_P, quat_from_R(R),
                            np.zeros(3), quat_from_R(R), C_LOC, UP_LOC, HZ)
    d = drift_between(ref, now)
    assert d.grip == pytest.approx(0.0, abs=1e-9)
    assert d.rot_deg == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------- 9/24 마찰 파지 판
#
#   지렛대 156.2 cm · 회전 0.2° · 원점 0.1 cm · 형상중심 0.5 cm · 쥔점 0.50 cm
#
# 이 네 숫자가 **하나의 강체 운동**으로 설명되는지 확인한다. 설명되면 "자가 이상하다"
# 가 아니라 "책이 실제로 그만큼 움직였다" 는 뜻이고, 판단이 달라진다.

MEASURED_LEVER = 1.562


def test_the_friction_run_is_one_rigid_motion():
    """실측 네 숫자가 평행이동 1 mm + 원점 둘레 0.2° 회전 하나로 맞아떨어진다."""
    c_loc = np.array([MEASURED_LEVER, 0.0, 0.0])
    up, hz = np.array([0.0, 0.0, 1.0]), 0.08

    def pts(bp, R):
        return hand_frame_points(HAND_P, HAND_Q, bp, quat_from_R(R), c_loc, up, hz)

    ref = pts(np.zeros(3), np.eye(3))
    now = pts(np.array([0.001, 0.0, 0.0]), _rot_z(0.2))   # 1 mm 이동 + 원점 둘레 0.2°
    d = drift_between(ref, now)
    assert d.rot_deg == pytest.approx(0.2, abs=1e-6)
    assert d.origin * 100 == pytest.approx(0.1, abs=0.01)     # 실측 0.1 cm
    assert d.center * 100 == pytest.approx(0.5, abs=0.06)     # 실측 0.5 cm
    assert d.grip * 100 == pytest.approx(0.5, abs=0.06)       # 실측 0.50 cm


def test_so_the_book_really_moved_five_millimetres():
    """**자의 문제가 아니다.** 원점이 안 움직였다고 책이 안 움직인 게 아니다.

    원점은 형상에서 1.56 m 떨어진 허공의 점이다. 그 점이 제자리여도 책 자체는
    회전 × 지렛대만큼 쓸고 지나간다. 쥔점 기준 5 mm 는 **실제 변위**다.
    """
    assert amplification_mm(MEASURED_LEVER, 0.2) == pytest.approx(5.45, abs=0.05)
    # 문턱 5 mm 는 이 지렛대에서 0.18° 에 해당한다 — 문턱이 곧 각도 문턱이다
    assert amplification_mm(MEASURED_LEVER, 0.184) == pytest.approx(5.0, abs=0.1)
