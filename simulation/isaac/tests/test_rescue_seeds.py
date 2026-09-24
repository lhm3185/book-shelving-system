"""씨앗 구조 — 국소 풀이기가 못 풀 때 건네줄 해를 고르는 산수.

2026-09-25 00:40: 스윙 끝에서 DOWN→HORIZ 뒤집기를 Lula 가 못 풀었는데 같은 점이 오프라인에서는
여유 0.76 으로 풀렸다. 도달이 아니라 씨앗 문제. 여기서 "여러 씨앗에서 풀어 여유 큰 순, 서로 다른
것만, 최대 k 개" 를 지킨다.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak  # noqa: E402

HOME = np.array([2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033])
HORIZ = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])


def test_구조_씨앗은_풀리는_해이고_여유_큰_순이다():
    """스윙 끝 자리(방위각 141.6°, 반지름 0.56, z 0.339)에서 HORIZ — 실측에서 Lula 가 못 푼 점."""
    p = np.array([0.56 * np.cos(np.radians(141.6)), 0.56 * np.sin(np.radians(141.6)), 0.339])
    seeds = ak.rescue_seeds(p, HORIZ, HOME, pos_tol=0.004, rot_tol=0.05, min_margin=0.0)
    assert 1 <= len(seeds) <= 3
    margins = [ak.limit_margin(q) for q in seeds]
    assert margins == sorted(margins, reverse=True)
    for q in seeds:
        pq, Rq = ak.fk(q)
        assert np.linalg.norm(pq - p) < 0.004
        assert np.linalg.norm(ak._rot_error(Rq, HORIZ)) < 0.05


def test_서로_같은_해는_하나만_남는다():
    p, R = ak.fk(HOME)
    seeds = ak.rescue_seeds(p, R, HOME, k=5, min_margin=0.0)
    for i, a in enumerate(seeds):
        for b in seeds[i + 1:]:
            assert float(np.max(np.abs(a - b))) >= 0.05


def test_못_푸는_점은_빈_목록이다():
    seeds = ak.rescue_seeds(np.array([0.0, 0.0, 3.0]), HORIZ, HOME, min_margin=0.0)
    assert seeds == []
