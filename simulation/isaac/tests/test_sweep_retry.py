"""스윕 재시도 — **반쪽 계획을 받아들이지 않는다.**

2026-09-24 SW1: 아래 판이 `정지점 3/5` 로 반쪽만 풀렸는데 그대로 실행했다.
3개까지 가고 멈췄고, 못 간 구간에서 404 시간 초과가 났다.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from sweep_retry import attempts, plan_full  # noqa: E402

POINTS = 5


class _Plan:
    def __init__(self, n, margin=0.2):
        self.hold_idx = tuple(range(n))
        self.min_margin = margin


def _planner(table):
    """`(x_from, x_to) → 정지점 수` 표대로 답하는 가짜 계획기."""
    seen = []

    def fn(seed, a, b):
        seen.append((seed, a, b))
        return _Plan(table.get((a, b), 0))
    fn.seen = seen
    return fn


def test_the_first_attempt_wins_when_it_solves():
    """되면 거기서 멈춘다 — 쓸데없이 더 풀지 않는다."""
    fn = _planner({(-0.35, 0.35): 5})
    p, name, tried = plan_full(fn, attempts("qA", "qH", -0.35, 0.35), POINTS)
    assert p is not None and name == "이어가기"
    assert len(fn.seen) == 1 and tried == ["이어가기 5/5 여유 0.200"]


def test_reversing_direction_rescues_the_lower_board():
    """**SW1 의 모양.** 이어가기는 3/5, 되돌아오기는 5/5 — 뱀처럼 훑으면 풀린다."""
    fn = _planner({(-0.35, 0.35): 3, (0.35, -0.35): 5})
    p, name, tried = plan_full(fn, attempts("qA", "qH", -0.35, 0.35), POINTS)
    assert p is not None and name == "되돌아오기"
    assert tried[0].startswith("이어가기 3/5")


def test_changing_the_seed_is_the_last_resort():
    """방향으로 안 되면 앞 판의 **가지를 버리고** 홈에서 다시 푼다."""
    calls = []

    def fn(seed, a, b):
        calls.append(seed)
        return _Plan(5 if seed == "qH" else 2)
    p, name, _tried = plan_full(fn, attempts("qA", "qH", -0.35, 0.35), POINTS)
    assert p is not None and name == "홈에서 다시"
    assert calls[:2] == ["qA", "qA"]          # 이어가기를 먼저 다 해 본다


def test_all_partial_means_no_plan_at_all():
    """**가장 나은 반쪽을 돌려주지 않는다.** 반쪽을 실행하면 거기서 죽는다."""
    fn = _planner({(-0.35, 0.35): 3, (0.35, -0.35): 4})
    p, name, tried = plan_full(fn, attempts("qA", "qH", -0.35, 0.35), POINTS)
    assert p is None and name == ""
    assert len(tried) == 4                     # 네 가지를 다 해 보고 포기했다


def test_the_record_says_what_was_tried():
    """**무엇을 해 봤는지가 남아야** 다음에 어디를 고칠지 안다."""
    fn = _planner({(-0.35, 0.35): 3, (0.35, -0.35): 4})
    _p, _n, tried = plan_full(fn, attempts("qA", "qH", -0.35, 0.35), POINTS)
    assert "이어가기 3/5" in tried[0] and "되돌아오기 4/5" in tried[1]
    assert any("홈에서" in t for t in tried)


def test_without_a_home_seed_only_direction_is_tried():
    """홈 자세를 모르면 그건 빼고 해 본다 — 없는 것을 지어내지 않는다."""
    assert len(attempts("qA", None, -0.35, 0.35)) == 2


def test_continuing_is_tried_before_anything_else():
    """이어가기가 움직임이 가장 적다 — 되면 그게 제일 좋다."""
    names = [a[0] for a in attempts("qA", "qH", -0.35, 0.35)]
    assert names[0] == "이어가기" and names[1] == "되돌아오기"
