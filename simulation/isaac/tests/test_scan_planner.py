"""스캔 자세 계획 단위시험 — Isaac 없이 돈다 (순수 기하)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from scan_planner import (  # noqa: E402
    ARM_BASE_Z, BOOK_CENTER_H, R_HAND_CAM, SHELF_FACE_Y, T_HAND_CAM,
    camera_pose, camera_rotation, hand_pose, is_reachable_board,
    scan_poses, unreachable_slots)


def test_camera_looks_at_the_shelf_when_level():
    """수평(0°)이면 카메라는 서가 쪽(+Y)을 보고, 목표점만큼 물러나 있다."""
    pos, R = camera_pose(1.042, 0.0, 0.65, 0.0)
    view = -R[:, 2]                                   # 카메라는 자기 -Z 를 본다
    assert view == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    assert pos[1] == pytest.approx(SHELF_FACE_Y - 0.65, abs=1e-9)
    # 수평이면 카메라 높이 = 목표 높이
    assert pos[2] == pytest.approx(1.042 + BOOK_CENTER_H - ARM_BASE_Z, abs=1e-9)


def test_tilt_lowers_the_camera_and_raises_the_view():
    """올려다보면 카메라는 **더 낮은 곳**에 놓인다 — 목표를 향해 비스듬히 물러나기 때문."""
    level, _ = camera_pose(1.581, 0.0, 0.60, 0.0)
    tilted, R = camera_pose(1.581, 0.0, 0.60, 40.0)
    assert tilted[2] < level[2]
    assert tilted[1] > level[1]                       # 서가에 더 가까워진다 (수평거리가 짧다)
    view = -R[:, 2]
    assert math.degrees(math.asin(view[2])) == pytest.approx(40.0, abs=1e-6)


def test_camera_distance_is_measured_along_the_view():
    """거리는 시선 방향으로 잰다 — 각도를 줘도 목표까지 거리는 그대로다."""
    for tilt in (0.0, 20.0, 40.0):
        pos, R = camera_pose(1.581, 0.0, 0.60, tilt)
        target = np.array([0.0, SHELF_FACE_Y, 1.581 + BOOK_CENTER_H - ARM_BASE_Z])
        assert float(np.linalg.norm(target - pos)) == pytest.approx(0.60, abs=1e-9)


def test_hand_pose_puts_the_camera_where_we_asked():
    """손 자세에 고정 변환을 다시 걸면 원하던 카메라 자세가 나와야 한다."""
    for board, cx, d, tilt in ((0.498, -0.30, 0.65, 0.0), (1.581, 0.30, 0.60, 40.0)):
        want_p, want_R = camera_pose(board, cx, d, tilt)
        hp, hR = hand_pose(board, cx, d, tilt)
        got_p = hp + hR @ T_HAND_CAM
        got_R = hR @ R_HAND_CAM
        assert got_p == pytest.approx(want_p, abs=1e-12)
        assert got_R == pytest.approx(want_R, abs=1e-12)


def test_rotation_is_orthonormal():
    """회전 행렬이 정규직교여야 IK 가 받아들인다."""
    for tilt in (0.0, 30.0, 40.0, 50.0):
        R = camera_rotation(math.radians(tilt))
        assert R.T @ R == pytest.approx(np.eye(3), abs=1e-12)
        assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-12)


def test_scan_table_matches_what_ik_actually_solved():
    """표에 든 조합은 **IK 해가 있다고 실측된 것만** 이어야 한다 (2026-09-22).

    0.498 판은 수평·올려보기로는 정면(cx=0)에 해가 없어 **내려다본다**.
    2.058 판은 어떤 각도·거리로도 해가 없어 표에서 뺐다.
    """
    poses = scan_poses()
    boards = {m["board_z"] for _, _, _, m in poses}
    assert 2.058 not in boards, "2.058 판은 IK 해가 없다 — 표에 넣으면 안 된다"
    for _, _, _, m in poses:
        if m["board_z"] == 0.498:
            assert m["tilt_deg"] < 0.0, "0.498 판은 **내려다봐야** 정면이 풀린다"
        if m["board_z"] == 1.581:
            assert m["tilt_deg"] >= 30.0, "1.581 판은 30° 이상 올려다봐야 한다"


def test_bottom_board_is_covered_left_to_right():
    """아래 판은 좌·중·우 세 자세로 덮는다 — 내려다보면 정면도 풀리기 때문."""
    cxs = sorted(m["cx"] for _, _, _, m in scan_poses() if m["board_z"] == 0.498)
    assert cxs == [-0.30, 0.00, 0.30]


def test_seed_list_includes_previous_and_home():
    """씨앗에는 앞 자세와 홈이 반드시 들어간다 — 가지가 튀는 것을 막는다."""
    from scan_planner import SEED_DELTAS, SEED_JOINTS, seeds_for
    q = np.zeros(7)
    home = np.ones(7) * 0.5
    s = seeds_for(q, home)
    assert s[0] == pytest.approx(q)
    assert s[1] == pytest.approx(home)
    assert len(s) == 2 + len(SEED_JOINTS) * len(SEED_DELTAS)


def test_pick_closest_minimises_the_largest_joint_move():
    """후보 중 **가장 크게 도는 관절**이 작은 것을 고른다 (합이 아니다)."""
    from scan_planner import pick_closest, step_size
    q_prev = np.zeros(7)
    far = np.array([3.0, 0, 0, 0, 0, 0, 0])          # 한 관절이 크게
    spread = np.ones(7) * 0.5                        # 여럿이 조금씩 (합은 더 큼)
    got = pick_closest([far, spread], q_prev)
    assert got is pytest.approx(spread) or np.allclose(got, spread)
    assert step_size(q_prev, spread) < step_size(q_prev, far)


def test_pick_closest_handles_no_solution():
    from scan_planner import pick_closest
    assert pick_closest([], np.zeros(7)) is None


def test_scan_goes_bottom_to_top():
    """아래에서 위로 훑는다 — 팔을 접었다 펴는 큰 이동을 줄인다."""
    boards = [m["board_z"] for _, _, _, m in scan_poses()]
    assert boards == sorted(boards)


def test_reachable_is_narrower_than_scannable():
    """**보이는 판이 꽂을 수 있는 판보다 넓다.** 이 차이가 413 의 존재 이유다."""
    poses = scan_poses()
    scanned = {m["board_z"] for _, _, _, m in poses}
    placeable = {m["board_z"] for _, _, _, m in poses if m["reachable"]}
    assert placeable < scanned
    assert 1.581 in scanned and 1.581 not in placeable


def test_unreachable_slots_flags_the_upper_board():
    """비전이 준 빈칸 중 위 판의 것은 1차 거름에서 걸러진다."""
    def slot(board):
        return (0.0, 0.5495, board + BOOK_CENTER_H - ARM_BASE_Z)
    slots = [slot(0.498), slot(1.581), slot(1.042), slot(2.058)]
    bad = unreachable_slots(slots)
    assert [i for i, _ in bad] == [1, 3]
    assert bad[0][1] == pytest.approx(1.581, abs=1e-9)


def test_is_reachable_board_uses_tolerance():
    """선반판 높이는 실측값이라 딱 떨어지지 않는다 — 약간의 오차를 받아들인다."""
    assert is_reachable_board(0.498)
    assert is_reachable_board(1.0425, tol=0.01)
    assert not is_reachable_board(1.06, tol=0.01)
