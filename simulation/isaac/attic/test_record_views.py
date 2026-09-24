"""녹화 시점 단위시험 — **숫자를 베끼지 않고 성질을 확인한다.**

시점의 정확한 좌표를 시험에 적으면 구현을 두 번 적는 것이고, 레벨이 바뀌면 둘 다
고쳐야 한다. 여기서 보는 것은 **"옆에서 보는 시점이 정말 옆에 있는가"** 같은 성질이다.
"""

import math
import os
import sys

import numpy as np

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from record_views import (  # noqa: E402
    VIEW_NAMES, all_poses, look_at_quat, parse_views, view_pose)

ROBOT = (2.1, -3.4, 0.0, 3.0, -2.7, 1.3)      # 카트+팔 월드 AABB
SHELF = (1.87, -2.57, 0.0, 3.28, -2.27, 2.6)  # 서가 (로봇보다 +y 쪽)
HOME = (5.0, -5.6)                            # 키오스크
RMID = np.array([(ROBOT[0] + ROBOT[3]) / 2, (ROBOT[1] + ROBOT[4]) / 2,
                 (ROBOT[2] + ROBOT[5]) / 2])


def _forward(q):
    """쿼터니언 → 카메라가 보는 방향 (USD 카메라는 **−z** 를 본다)."""
    w, x, y, z = q
    return -np.array([2 * (x * z + w * y), 2 * (y * z - w * x),
                      1 - 2 * (x * x + y * y)])


# ------------------------------------------------------------------ look_at
def test_the_camera_actually_points_at_the_target():
    eye, tgt = np.array([3.0, -5.0, 2.0]), np.array([2.5, -2.5, 0.6])
    want = (tgt - eye) / np.linalg.norm(tgt - eye)
    assert _forward(look_at_quat(eye, tgt)) == pytest.approx(want, abs=1e-6)


def test_it_points_straight_down_without_blowing_up():
    """바로 위에서 내려다보는 시점은 시선이 `up` 과 나란하다 — 거기서 죽으면 안 된다."""
    d = _forward(look_at_quat([2.5, -3.0, 4.0], [2.5, -3.0, 0.5]))
    assert d == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)


def test_a_degenerate_look_gives_the_identity_not_garbage():
    """눈과 목표가 같으면 방향이 없다 — **아무 방향이나 내지 않는다.**"""
    assert look_at_quat([1, 1, 1], [1, 1, 1]) == pytest.approx([1, 0, 0, 0])


def test_the_quaternion_is_normalised():
    q = look_at_quat([4.0, -4.0, 1.5], [2.5, -2.6, 0.7])
    assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=1e-9)


# ------------------------------------------------------------------ 시점 고르기
def test_parse_handles_empty_list_and_all():
    assert parse_views("") == () and parse_views("   ") == ()
    assert parse_views("all") == VIEW_NAMES
    assert parse_views("side, top ,side") == ("side", "top")   # 중복은 한 번만


def test_an_unknown_view_is_refused_with_the_list():
    """**조용히 무시하지 않는다.** 오타 때문에 안 찍힌 것을 나중에 알면 늦다."""
    with pytest.raises(ValueError) as e:
        parse_views("sied")
    assert "side" in str(e.value)


# ------------------------------------------------------------------ 시점의 성질
def test_side_is_actually_to_the_side():
    """옆 시점은 로봇의 +x 쪽에 있고 높이는 로봇 언저리다."""
    eye, look = view_pose("side", ROBOT, SHELF)
    assert eye[0] > RMID[0] and abs(eye[1] - RMID[1]) < 0.05
    assert look == pytest.approx(RMID)


def test_top_is_actually_above():
    eye, _look = view_pose("top", ROBOT, SHELF)
    assert eye[2] > ROBOT[5]                       # 로봇 꼭대기보다 위
    assert abs(eye[0] - RMID[0]) < 0.05


def test_front_is_on_the_opposite_side_from_the_shelf():
    """앞 시점은 **서가 반대쪽**에서 로봇을 본다 — 서가에 가리면 팔이 안 보인다."""
    eye, _look = view_pose("front", ROBOT, SHELF)
    shelf_mid_y = (SHELF[1] + SHELF[4]) / 2
    assert (eye[1] - RMID[1]) * (shelf_mid_y - RMID[1]) < 0    # 부호가 반대다


def test_shelf_view_looks_at_the_near_face_from_the_robot_side():
    """서가 근접 시점은 **로봇이 있는 쪽 면**을 본다 — 반대편을 보면 뒤판만 찍힌다."""
    eye, look = view_pose("shelf", ROBOT, SHELF)
    assert look[1] in (SHELF[1], SHELF[4])
    assert (eye[1] - look[1]) * (RMID[1] - look[1]) > 0        # 로봇과 같은 쪽


def test_wide_holds_both_the_robot_and_home():
    """넓은 시점은 키오스크와 서가가 **한 화면에** 들어야 한다."""
    eye, look = view_pose("wide", ROBOT, SHELF, HOME)
    span = math.dist(HOME, RMID[:2])
    assert math.dist(look[:2], RMID[:2]) < span                # 둘 사이를 본다
    assert math.dist(look[:2], HOME) < span
    assert math.dist(eye[:2], look[:2]) > span * 0.5           # 충분히 물러났다


def test_every_view_is_outside_the_robot_box():
    """어느 시점도 로봇 안에 있으면 안 된다 — 안에서는 아무것도 안 보인다."""
    for name, (eye, _look) in all_poses(VIEW_NAMES, ROBOT, SHELF, HOME).items():
        inside = all(ROBOT[i] <= eye[i] <= ROBOT[i + 3] for i in range(3))
        assert not inside, name


def test_views_follow_the_level_instead_of_being_hard_coded():
    """서가를 옮기면 시점도 따라간다 — **자리를 코드에 박지 않았다.**"""
    moved = tuple(v + (1.0 if i in (0, 3) else 0.0) for i, v in enumerate(ROBOT))
    a, _ = view_pose("side", ROBOT, SHELF)
    b, _ = view_pose("side", moved, SHELF)
    assert b[0] > a[0]
