"""운반 경유점 사다리 — **아래 판은 한 점도 안 바뀌고, 위 판은 자세를 바꿔 지나간다**.

2026-09-25 00:00: 위 판에서 `transfer@DOWN` 이 어떤 y 에서도 안 풀렸다(arm_kinematics 표).
경로를 구부리는 다섯 수가 전부 실패하고 점을 옮기는 것만 남았다. 이 시험이 지키는 것:
첫 후보가 지금 경로라는 것, 되돌림이 조용히 지나가지 않는다는 것, 전부 실패하면 사유가
전부 남는다는 것.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from arm_planning import carry_ladder, plan_first  # noqa: E402

LIFT = np.array([-0.5, 0.0, 0.3]); TRANSFER = np.array([-0.35, 0.45, 0.95]); PRE = np.array([-0.35, 0.47, 0.90])
DOWN = np.array([0.0, 1.0, 0.0, 0.0]); HORIZ = np.array([0.7, 0.7, 0.0, 0.0])


def test_첫_후보가_지금까지의_경로다():
    """`(lift,DOWN) → (transfer,DOWN) → (pre_ins,HORIZ)` — 2026-09-24 까지 아래 판이 통과한 그 경유점."""
    label, wps = carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ)[0]
    assert label == "transfer@DOWN"
    assert [tuple(np.round(p, 3)) for p, _ in wps] == [tuple(LIFT), tuple(TRANSFER), tuple(PRE)]
    assert [tuple(o) for _, o in wps] == [tuple(DOWN), tuple(DOWN), tuple(HORIZ)]


def test_후보는_자세를_바꾸고_그다음_빼는_순서다():
    labels = [l for l, _ in carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ)]
    assert labels == ["transfer@DOWN", "transfer@HORIZ", "transfer 없이"]
    last = carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ)[-1][1]
    assert len(last) == 2 and tuple(last[1][0]) == tuple(PRE)


def test_아래_판은_첫_후보에서_끝나_되돌림이_없다():
    calls = []

    def planner(wps):
        calls.append(len(wps))
        return [np.zeros(7)], 0.01, ""

    qs, worst, err, i, errs = plan_first(carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ), planner)
    assert qs is not None and i == 0 and errs == [] and err == ""
    assert calls == [3], "첫 후보가 풀리면 다른 후보를 **시도조차** 하지 않는다"


def test_위_판은_DOWN_이_안_풀리면_HORIZ_로_간다():
    def planner(wps):
        # transfer 를 DOWN 으로 들르는 후보만 실패시킨다
        if any(tuple(o) == tuple(DOWN) and tuple(np.round(p, 3)) == tuple(TRANSFER) for p, o in wps):
            return None, 0.0, "IK 실패 [transfer]"
        return [np.zeros(7)], 0.02, ""

    qs, worst, err, i, errs = plan_first(carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ), planner)
    assert qs is not None and i == 1
    assert errs == ["transfer@DOWN: IK 실패 [transfer]"], "앞 후보의 실패 사유가 남아 로그에 실린다"


def test_전부_실패하면_사유가_전부_남는다():
    qs, worst, err, i, errs = plan_first(
        carry_ladder(LIFT, TRANSFER, PRE, DOWN, HORIZ), lambda w: (None, 0.0, f"{len(w)}점 실패"))
    assert qs is None and i == 3
    assert "transfer@DOWN: 3점 실패" in err and "transfer@HORIZ: 3점 실패" in err and "transfer 없이: 2점 실패" in err
