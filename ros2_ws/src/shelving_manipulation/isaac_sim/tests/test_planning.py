"""기하·경로 계획 단위 테스트. Isaac 없이 가짜 IK 로 돈다."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from arm_geometry import (R_from_quat, in_frame, orientation_from_axes,  # noqa: E402
                          quat_angle, quat_from_R, slerp)
from arm_planning import MAX_JOINT_STEP, plan_chain, plan_path  # noqa: E402

DOWN_WORLD = ([0, 0, -1], [1, 0, 0])
HORIZ_WORLD = ([0, 1, 0], [1, 0, 0])
A_LOC, C_LOC = [0, 0, 1], [0, 1, 0]        # Isaac 에서 실측한 right_gripper 로컬 축


def smooth_ik(pos, quat, seed):
    """위치를 그대로 관절처럼 쓰는 연속 IK"""
    return np.concatenate([np.asarray(pos, float), [0, 0, 0, 0]]), True


# ------------------------------------------------------------ 기하

def test_quat_matrix_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        q2 = quat_from_R(R_from_quat(q))
        assert quat_angle(q, q2) < 1e-6


def test_slerp_endpoints_and_midpoint():
    q0 = np.array([1.0, 0, 0, 0])
    q1 = quat_from_R(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))   # z 90°
    assert quat_angle(slerp(q0, q1, 0), q0) < 1e-9
    assert quat_angle(slerp(q0, q1, 1), q1) < 1e-9
    assert abs(quat_angle(q0, slerp(q0, q1, 0.5)) - math.pi / 4) < 1e-6


def test_orientation_maps_measured_axes():
    q = orientation_from_axes(A_LOC, C_LOC, *DOWN_WORLD)
    Rm = R_from_quat(q)
    assert np.allclose(Rm @ A_LOC, DOWN_WORLD[0], atol=1e-9)
    assert np.allclose(Rm @ C_LOC, DOWN_WORLD[1], atol=1e-9)


def test_down_to_horizontal_is_90_degrees():
    """손목 세우기는 90° 회전이다 (책등 위 → 책등 로봇 쪽)"""
    qd = orientation_from_axes(A_LOC, C_LOC, *DOWN_WORLD)
    qh = orientation_from_axes(A_LOC, C_LOC, *HORIZ_WORLD)
    assert abs(math.degrees(quat_angle(qd, qh)) - 90) < 1e-6


def test_in_frame_detects_rotation_slip():
    """손끝을 중심으로 책이 돌면 거리는 같지만 손 좌표계 위치는 바뀐다 (M406 감지 근거)"""
    hand_p, hand_q = np.zeros(3), np.array([1.0, 0, 0, 0])
    book = np.array([0.0, 0.3, 0.0])
    rotated = np.array([0.3 * math.sin(0.2), 0.3 * math.cos(0.2), 0.0])
    assert abs(np.linalg.norm(book) - np.linalg.norm(rotated)) < 1e-9          # 거리는 같다
    assert np.linalg.norm(in_frame(hand_p, hand_q, book) - in_frame(hand_p, hand_q, rotated)) > 0.03


# ------------------------------------------------------------ 계획

def test_plan_discretizes_by_distance():
    wps = [(np.zeros(3), np.array([1.0, 0, 0, 0])), (np.array([0.1, 0, 0]), np.array([1.0, 0, 0, 0]))]
    plan = plan_path(wps, np.zeros(7), smooth_ik)
    assert plan.ok
    assert len(plan.qs) - 1 >= 21          # 시작점 1 + 10cm / 5mm = 20 구간


def test_plan_discretizes_by_rotation():
    q0 = orientation_from_axes(A_LOC, C_LOC, *DOWN_WORLD)
    q1 = orientation_from_axes(A_LOC, C_LOC, *HORIZ_WORLD)
    plan = plan_path([(np.zeros(3), q0), (np.zeros(3), q1)], np.zeros(7), smooth_ik)
    assert plan.ok and len(plan.qs) - 2 >= math.ceil((math.pi / 2) / 0.035)


def test_plan_reports_ik_failure_401():
    def failing_ik(pos, quat, seed):
        return np.zeros(7), pos[0] < 0.05
    wps = [(np.zeros(3), np.array([1.0, 0, 0, 0])), (np.array([0.1, 0, 0]), np.array([1.0, 0, 0, 0]))]
    plan = plan_path(wps, np.zeros(7), failing_ik)
    assert not plan.ok and plan.error_code == 401


def test_plan_detects_branch_flip_402():
    """해 가지가 바뀌어 관절이 크게 뛰면 실행하지 않는다 (손목 세우기 38회 튐의 원인)"""
    def flipping_ik(pos, quat, seed):
        q = np.concatenate([np.asarray(pos, float), [0, 0, 0, 0]])
        if pos[0] > 0.05:
            q[3] = 2.0                         # 가지 전환
        return q, True
    wps = [(np.zeros(3), np.array([1.0, 0, 0, 0])), (np.array([0.1, 0, 0]), np.array([1.0, 0, 0, 0]))]
    plan = plan_path(wps, np.zeros(7), flipping_ik)
    assert plan.error_code == 402 and plan.worst_step > MAX_JOINT_STEP


def test_chain_continues_from_previous_end_and_names_failure():
    q = np.array([1.0, 0, 0, 0])
    segs = [("a", [(np.zeros(3), q), (np.array([0.02, 0, 0]), q)]),
            ("b", [(np.array([0.02, 0, 0]), q), (np.array([0.04, 0, 0]), q)])]
    plans, summary = plan_chain(segs, np.zeros(7), smooth_ik)
    assert plans is not None
    assert np.allclose(plans["b"].qs[0], plans["a"].qs[-1])

    def fail_b(pos, quat, seed):
        return np.zeros(7), pos[0] <= 0.021
    plans, summary = plan_chain(segs, np.zeros(7), fail_b)
    assert plans is None and summary.error.startswith("b:")


def test_tucked_moves_fold_before_large_turn():
    from arm_planning import tucked_joint_moves
    stow = np.array([0.0, -1.75, 0.0, -2.8, 0.0, 1.1, 0.79])
    q_now = np.array([0.1, -0.3, 0.0, -1.5, 0.0, 1.4, 0.7])
    goal = np.array([2.77, -0.25, 0.18, -1.93, 0.04, 1.68, -0.99])
    moves = tucked_joint_moves(q_now, goal, stow)
    assert len(moves) == 3
    # 1번 관절은 접힌 상태에서만 돈다
    assert moves[0][0] == q_now[0] and np.allclose(moves[0][1:6], stow[1:6])
    assert moves[1][0] == goal[0] and np.allclose(moves[1][1:6], stow[1:6])
    assert np.allclose(moves[2], goal)


def test_tucked_moves_direct_for_small_turn():
    from arm_planning import tucked_joint_moves
    stow = np.zeros(7)
    q_now = np.array([2.6, -0.3, 0.0, -1.5, 0.0, 1.4, 0.7])
    goal = np.array([2.77, -0.25, 0.18, -1.93, 0.04, 1.68, -0.99])
    moves = tucked_joint_moves(q_now, goal, stow)
    assert len(moves) == 1 and np.allclose(moves[0], goal)
