"""자리 맞추기 상한 — **두 무리 사이가 비어 있어야 상한으로 가를 수 있다.**"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from base_move import (  # noqa: E402
    OBSERVED_MAX, OBSERVED_OUTLIER, move_limit_m, refuse_far_move, within_shelf_span)


def test_every_observed_normal_move_passes():
    """43회 실측의 정상값이 전부 통과한다 — **상한이 일을 막으면 안 된다.**"""
    for d in (0.266, 0.267, 0.273, 0.274, 0.288, 0.303, 0.444, OBSERVED_MAX):
        assert refuse_far_move(d) is None, d


def test_the_vd2_outlier_is_refused():
    """VD2 의 2.676 m 는 거절한다 — 키오스크에서 서가 자리를 맞추려 한 값이다."""
    why = refuse_far_move(OBSERVED_OUTLIER)
    assert why is not None
    assert "2.676" in why and "0.447" in why      # 사유에 실측 정상 최대를 같이 싣는다


def test_the_gap_between_the_two_groups_is_wide():
    """**분포가 갈라져 있어서** 고를 수 있는 상한이다. 이어져 있었으면 못 가른다."""
    assert OBSERVED_OUTLIER / OBSERVED_MAX > 5.0
    lim = move_limit_m()
    assert OBSERVED_MAX < lim < OBSERVED_OUTLIER
    assert lim / OBSERVED_MAX > 1.3               # 정상 최대에 30% 넘는 여유
    assert OBSERVED_OUTLIER / lim > 4.0           # 이상치보다 4배 아래


def test_the_limit_can_be_turned_off_for_regression():
    """0 이하면 끈다 — 옛 동작과 비교할 길을 막지 않는다."""
    assert refuse_far_move(OBSERVED_OUTLIER, limit_m=0.0) is None


def test_a_broken_env_value_falls_back_instead_of_crashing():
    """환경변수가 깨져 있다고 회전이 죽으면 안 된다."""
    old = os.environ.get("SIM_ROTATE_MAX_MOVE")
    os.environ["SIM_ROTATE_MAX_MOVE"] = "육점육"
    try:
        assert move_limit_m() == 0.6
    finally:
        os.environ.pop("SIM_ROTATE_MAX_MOVE", None)
        if old is not None:
            os.environ["SIM_ROTATE_MAX_MOVE"] = old


def test_서가_구간_안의_이동은_상한을_안_본다():
    """서가 B: 중심에 선 팔(x 0)이 95.8 mm 빈칸(팔 기준 +0.624)으로 — 서가 구간 [-0.70, +0.71] 안이다."""
    span = (-0.7065, +0.7061)
    assert within_shelf_span(0.0, 0.624, span)
    assert refuse_far_move(0.624, along_shelf=True) is None
    assert refuse_far_move(0.624, along_shelf=False) is not None     # 상한만 보면 4 % 초과


def test_서가_밖에서_시작하면_상한이_그대로_잡는다():
    """VD2: 키오스크에 선 채로 서가 자리를 맞추려 2.676 m — 출발점이 서가 구간 밖이라 뜻으로도 막힌다."""
    span = (2.3, 3.7)          # 서가가 팔 기준 2.3~3.7 m 옆에 있다
    assert not within_shelf_span(0.0, OBSERVED_OUTLIER, span)
    assert refuse_far_move(OBSERVED_OUTLIER, along_shelf=within_shelf_span(0.0, OBSERVED_OUTLIER, span)) is not None


def test_구간_정보가_없으면_상한으로_돌아간다():
    assert not within_shelf_span(0.0, 0.3, None)
    assert within_shelf_span(0.0, 0.70, (-0.7, 0.7)) and not within_shelf_span(0.0, 0.80, (-0.7, 0.7))
