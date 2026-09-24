"""가지 넘김 판별기 — **로그 두 줄로 GPU 없이 가르는 산수**를 지킨다.

2026-09-24 밤: 스윙 `c(reach)` 가 위 판에서만 401 이고, 32배로 잘게 나눠도 걸음이
0.126~0.168 rad 로 안 줄었다. 데스크탑은 "가지 넘김이니 관절공간으로 이으면 된다" 고
읽었고, 나는 "관절공간은 손끝을 1 m 솟게 한 전례가 있다" 고 읽었다. **둘 다 추측이라
판을 한 번 더 태워야 답이 나오는 상태**였다. 그걸 없애려고 만든 도구다.

가르는 산수는 하나다 — 앞·뒤 자세를 각각 FK 해서 **손끝이 움직였는가**를 본다.
안 움직였는데 관절이 멀면 같은 손 자세의 다른 팔 모양이다(가지). 움직였으면 아니다.
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak                        # noqa: E402,I100,I201
from swing_check import compare, envelope, parse_q  # noqa: E402,I100,I201

Q = np.array([0.0, -0.3, 0.0, -2.0, 0.0, 1.7, 0.8])


def test_관절값_읽기는_두_서식_모두():
    """쉼표든 대괄호든 로그에서 그대로 복사해 넣을 수 있어야 한다."""
    a = parse_q("0.1,-0.5,0.2,-2.0,0.0,1.7,0.8")
    b = parse_q("[0.1, -0.5, 0.2, -2.0, 0.0, 1.7, 0.8]")
    assert np.allclose(a, b)


def test_관절값이_모자라면_거절한다():
    """6개만 붙여넣는 실수를 조용히 넘기지 않는다."""
    with pytest.raises(SystemExit):
        parse_q("0.1, 0.2, 0.3")


def test_같은_손자세_다른_팔모양은_가지_넘김이다():
    """**이 판별이 이 도구의 전부다.** 손끝은 제자리인데 관절만 멀면 가지다."""
    p, R = ak.fk(Q)
    r = ak.ik(p, R, Q + np.array([1.2, 0.0, 0.9, 0.0, 1.1, 0.0, 0.0]))
    assert r.ok, "다른 씨앗에서 같은 손 자세의 해가 나와야 이 시험이 성립한다"
    assert float(np.max(np.abs(r.q - Q))) > 0.12, "충분히 먼 해여야 한다"
    out = compare(Q, r.q)
    assert out["판정"] == "가지 넘김"
    assert out["손끝_이동_mm"] < 10.0


def test_손끝이_움직였으면_가지가_아니다():
    """목표가 옮겨 간 것이면 관절공간 되돌림으로 풀 문제가 아니다."""
    p, R = ak.fk(Q)
    r = ak.ik(p + np.array([0.0, 0.10, 0.0]), R, Q)
    assert r.ok
    out = compare(Q, r.q)
    assert out["판정"] == "진짜 이동"
    assert out["손끝_이동_mm"] > 50.0


def test_봉투는_직선에서_벗어난_거리다():
    """두 끝을 잇는 직선에서 얼마나 벗어나는가 — 1 m 솟았던 그 값이다."""
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([1.0, 0.0, 0.0])

    # FK 를 가짜로 바꿔 산수만 본다: 가운데서 z 로 0.2 솟는 경로
    real_fk = ak.fk
    pts = [np.array([0.0, 0.0, 0.0]), np.array([0.5, 0.0, 0.2]), np.array([1.0, 0.0, 0.0])]
    it = iter(pts)
    ak.fk = lambda q: (next(it), np.eye(3))                   # noqa: E731
    try:
        dev, rise = envelope([0, 1, 2], a, b)
    finally:
        ak.fk = real_fk
    assert dev == pytest.approx(0.2, abs=1e-6)
    assert rise == pytest.approx(0.2, abs=1e-6)


def test_제자리_두_자세는_판정하지_않는다():
    """둘 다 작으면 "모르겠다" 라고 말한다 — 지어내지 않는다."""
    out = compare(Q, Q.copy())
    assert out["판정"] == "판정 불가"
    assert out["관절보간_직선이탈_m"] == 0.0
