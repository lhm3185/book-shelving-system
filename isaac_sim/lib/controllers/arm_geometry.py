"""회전·쿼터니언 계산. 순수 numpy — Isaac 없이 테스트한다.

쿼터니언은 전부 (w, x, y, z) 순서다 (Isaac Sim 규약).
"""

import math

import numpy as np


def quat_from_R(M) -> np.ndarray:
    M = np.asarray(M, float)
    w = math.sqrt(max(0.0, 1 + M[0, 0] + M[1, 1] + M[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + M[0, 0] - M[1, 1] - M[2, 2])) / 2, M[2, 1] - M[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] + M[1, 1] - M[2, 2])) / 2, M[0, 2] - M[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] - M[1, 1] + M[2, 2])) / 2, M[1, 0] - M[0, 1])
    return np.array([w, x, y, z])


def R_from_quat(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, t: float) -> np.ndarray:
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


def quat_angle(q0, q1) -> float:
    """두 자세 사이 회전각(rad)"""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(np.asarray(q0, float), np.asarray(q1, float))))))


def orientation_from_axes(approach_local, closing_local, approach_world, closing_world) -> np.ndarray:
    """그리퍼 로컬 접근축·닫힘축이 월드의 approach·closing 방향을 향하게 하는 자세.

    로컬 축은 Isaac 에서 실측한다(손끝-손 방향, 두 손가락 사이 방향). 규약을 기억으로 가정하지 않는다.
    """
    a_l = np.round(np.asarray(approach_local, float))
    c_l = np.round(np.asarray(closing_local, float))
    a_w = np.asarray(approach_world, float)
    c_w = np.asarray(closing_world, float)
    L = np.stack([a_l, c_l, np.cross(a_l, c_l)], axis=1)
    W = np.stack([a_w, c_w, np.cross(a_w, c_w)], axis=1)
    return quat_from_R(W @ L.T)


def in_frame(frame_pos, frame_quat, point) -> np.ndarray:
    """월드 점을 (frame_pos, frame_quat) 좌표계로 옮긴다"""
    return R_from_quat(frame_quat).T @ (np.asarray(point, float) - np.asarray(frame_pos, float))
