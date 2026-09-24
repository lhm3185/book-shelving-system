"""레벨에서 로봇 자세를 읽는 산수 — **손으로 적은 상수를 없애려고 만든 것**이다.

2026-09-24: 복귀 좌표가 `waypoints.yaml` 에 상수로 박혀 있어 레벨과 4.99 m · 90°
어긋나 있었다. 도윤님 물음이 정확했다 — *"레벨에 배치된 로봇의 각도랑 좌표를
그대로 가져다 쓰면 안 되는 거야?"* 된다. 그 산수를 여기서 지킨다.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

from level_robot_pose import (  # noqa: E402
    ARM_BASE_OFFSET_LOCAL, arm_base_from_root, quat_from_yaw, yaw_from_matrix)


def _m(yaw, x=0.0, y=0.0):
    """USD 스타일 4x4 (행 우선, 마지막 행이 평행이동)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, s, 0, 0], [-s, c, 0, 0], [0, 0, 1, 0], [x, y, 0, 1]]


@pytest.mark.parametrize("deg", [0, 45, 90, -90, 180, -179.9])
def test_yaw_round_trips(deg):
    got = math.degrees(yaw_from_matrix(_m(math.radians(deg))))
    assert math.isclose((got - deg + 180) % 360 - 180, 0.0, abs_tol=1e-6)


def test_a_degenerate_matrix_does_not_blow_up():
    """x 축이 0 이면 각을 알 수 없다 — **0 을 주고 죽지 않는다.**"""
    assert yaw_from_matrix([[0, 0, 0, 0]] * 4) == 0.0


def test_quat_matches_the_waypoint_convention():
    """yaw 90° → (0, 0, 0.7071, 0.7071). 경유점 yaml 이 쓰는 값과 같아야 한다."""
    x, y, z, w = quat_from_yaw(math.radians(90))
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(0.70710678, abs=1e-7)
    assert w == pytest.approx(0.70710678, abs=1e-7)


def test_both_measured_pairs_agree_on_the_offset_direction():
    """**실측 두 쌍이 같은 오프셋을 가리켜야 한다.** 하나만 보면 방향을 못 가린다.

    2026-09-24:
      출발  루트 (4.986, −5.607) yaw +90° → 팔 베이스 (4.9850, −5.3071)
      서가  루트 (2.535, −3.019) yaw   0° → 팔 베이스 (2.835,  −3.019)

    **처음에 로컬 +y 로 적었다가 이 시험이 잡았다.** 출발 쪽 한 쌍만 봤으면
    x 와 y 중 어느 쪽인지 못 가렸을 것이다.
    """
    ax, ay = arm_base_from_root(4.986, -5.607, math.radians(90))
    assert (ax, ay) == pytest.approx((4.986, -5.307), abs=1e-3)
    bx, by = arm_base_from_root(2.535, -3.019, 0.0)
    assert (bx, by) == pytest.approx((2.835, -3.019), abs=1e-3)


def test_root_and_arm_base_are_not_interchangeable():
    """로그가 주는 것은 **팔 베이스**, 경유점에 넣을 것은 **루트**다.

    섞으면 300 mm 가 통째로 틀어진다.
    """
    x, y = 4.986, -5.607
    ax, ay = arm_base_from_root(x, y, math.radians(90))
    assert math.dist((x, y), (ax, ay)) == pytest.approx(0.30, abs=1e-9)


def test_the_offset_is_a_single_documented_number():
    """오프셋이 여러 군데 흩어지지 않게 — **한 곳에서만 적는다.**"""
    assert ARM_BASE_OFFSET_LOCAL == (0.30, 0.0)
