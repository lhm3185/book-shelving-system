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


# ---------------------- 닿는 데까지 훑는다 (2026-09-24 SW2)
#
#   아래 판(**우리가 책을 꽂는 그 판**)을 한 정지점도 안 보고 사이클이 통과했다.
#   세 전략이 모두 오른쪽에서 실패했다 — 이어가기는 x=+0.175, 되돌아오기는
#   첫 자세 x=+0.35, 홈에서 다시도 +0.175. **오른쪽이 안 닿는 것이다.**
#   그런데 **왼쪽 절반은 닿고, 우리 빈칸도 거기(x −0.077)에 있다.**

def _reach_planner(reach_lo, reach_hi, points=POINTS):
    """`[reach_lo, reach_hi]` 안에 완전히 들어가야 풀리는 가짜 계획기."""
    def fn(seed, a, b):
        lo, hi = min(a, b), max(a, b)
        return _Plan(points if lo >= reach_lo - 1e-9 and hi <= reach_hi + 1e-9 else 0)
    return fn


def test_the_full_range_is_tried_first():
    from sweep_retry import plan_reachable
    p, span, tried = plan_reachable(_reach_planner(-1.0, 1.0), "q", -0.35, 0.35, POINTS)
    assert p is not None and span == (-0.35, 0.35) and len(tried) == 1


def test_it_shrinks_the_unreachable_side():
    """**SW2 의 모양.** 오른쪽이 안 닿으면 왼쪽을 남긴다."""
    from sweep_retry import plan_reachable
    p, span, _t = plan_reachable(_reach_planner(-0.40, 0.10), "q", -0.35, 0.35, POINTS)
    assert p is not None
    assert span[0] == -0.35 and span[1] <= 0.10          # 왼쪽 끝은 지킨다
    assert span[1] - span[0] >= 0.20


def test_it_keeps_the_widest_that_works():
    """줄이더라도 **가장 넓은 것**을 고른다 — 덜 보면 빈칸을 놓친다."""
    from sweep_retry import plan_reachable
    _p, span, _t = plan_reachable(_reach_planner(-0.40, 0.30), "q", -0.35, 0.35, POINTS)
    assert span[1] - span[0] > 0.45                      # 0.8 배(0.56)가 잡힌다


def test_it_refuses_when_only_a_sliver_is_reachable():
    """**그만큼만 보고 '훑었다' 고 하면 거짓말이다.** 너무 좁으면 포기한다."""
    from sweep_retry import plan_reachable
    p, span, tried = plan_reachable(_reach_planner(-0.35, -0.25), "q", -0.35, 0.35, POINTS)
    assert p is None and span is None and tried


def test_the_record_shows_every_range_tried():
    from sweep_retry import plan_reachable
    _p, _s, tried = plan_reachable(_reach_planner(-0.40, 0.10), "q", -0.35, 0.35, POINTS)
    assert tried[0].startswith("[-0.35,+0.35]")          # 넓은 것부터
    assert any("0/" in t for t in tried)                 # 실패한 것도 남는다


def test_both_ends_are_tried_because_we_do_not_know_which_side_fails():
    """어느 쪽이 안 닿는지 모른다 — 양쪽 끝을 다 줄여 본다."""
    from sweep_retry import shrink_candidates
    c = shrink_candidates(-0.35, 0.35)
    assert (-0.35, 0.35) == c[0]
    assert any(lo == -0.35 and hi < 0.35 for lo, hi in c)   # 오른쪽을 줄인 것
    assert any(hi == 0.35 and lo > -0.35 for lo, hi in c)   # 왼쪽을 줄인 것
