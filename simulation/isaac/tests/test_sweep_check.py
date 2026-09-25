"""경로 스침 검사의 산수 — 쥔 책 상자, 회전 뒤 AABB, 두 상자 사이 이격."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from sweep_check import aabb_gap, aabb_of_points, box_corners, held_book_box_local, sweep  # noqa: E402


def test_쥔_책_상자는_손끝_아래로_매달린다():
    lo, hi = held_book_box_local((0.0352, 0.2265, 0.1517), 0.035)
    assert hi[2] == 0.035 and abs(lo[2] - (-(0.2265 - 0.035))) < 1e-9
    assert hi[0] == 0.1517 / 2 and lo[1] == -0.1517 / 2          # 보수적: 두께·폭 중 큰 쪽


def test_이격은_축별_최대이고_겹치면_음수다():
    a = [0, 0, 0, 1, 1, 1]
    assert aabb_gap(a, [1.02, 0, 0, 2, 1, 1]) == 0.02 or abs(aabb_gap(a, [1.02, 0, 0, 2, 1, 1]) - 0.02) < 1e-9
    assert abs(aabb_gap(a, [0.9, 0, 0, 2, 1, 1]) - (-0.1)) < 1e-9          # x 로 0.1 겹침
    assert aabb_gap(a, [0.9, 5, 0, 2, 6, 1]) == 4.0                          # y 로 4 떨어짐


def test_회전한_상자의_AABB_는_원본을_감싼다():
    c = box_corners([-1, -1, -1], [1, 1, 1])
    R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])    # z 축 90°
    bb = aabb_of_points((R @ c.T).T)
    assert np.allclose(bb, [-1, -1, -1, 1, 1, 1])


def test_덤프_없는_이웃이면_구간이_비고_최악도_없다():
    plan = {"l0p": [0, 0, 0], "Rl0": list(np.eye(3).ravel()), "tool": None,
            "book_dims": [0.035, 0.2265, 0.1517], "tip_down": 0.035,
            "waypoints": [[2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033]], "labels": ["approach"],
            "neighbours": []}
    per, worst = sweep(plan)
    assert per == {} and worst is None


def test_이웃이_손끝_바로_옆이면_이격이_작다():
    import arm_kinematics as ak
    q = [2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033]
    p, R = ak.fk(q)
    nb = [p[0] + 0.10, p[1] - 0.05, p[2] - 0.3, p[0] + 0.14, p[1] + 0.05, p[2] + 0.05]   # x 로 손끝 오른쪽 10 cm
    plan = {"l0p": [0, 0, 0], "Rl0": list(np.eye(3).ravel()), "tool": None,
            "book_dims": [0.035, 0.2265, 0.1517], "tip_down": 0.035,
            "waypoints": [q], "labels": ["carry_rotate"], "neighbours": [nb]}
    per, worst = sweep(plan)
    assert per["carry_rotate"]["n"] == 1 and worst is not None
    assert -0.16 < worst[0] < 0.10          # 보수적 상자(반폭 7.6 cm)라 10 cm 이웃은 2.4 cm 안팎이거나 살짝 겹침
