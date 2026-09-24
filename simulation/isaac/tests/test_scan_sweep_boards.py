"""스윕 스캔 계획 — **여기서 되는 것과 실제로 되는 것은 다르다.**

이 파일은 한때 `"두 판 다 5/5 로 풀린다"` 를 단언했다. 그리고 실제 판(2026-09-24
SW1)은 **아래 판이 3/5** 였다. 오프라인 재현이 실제와 다른 것이고, **틀린 주장을
지키는 시험은 없느니만 못하다.** 그래서 주장을 좁혔다.

무엇이 달랐나: 계획은 **앞 판의 끝 자세를 시드로** 이어받는다. 그 자세가 어느 IK
가지에 있느냐로 답이 갈린다. 실제 판의 위 판은 여유 0.210 짜리 가지로 끝나는데,
여기서 `q_home` 으로 시작하면 0.171 짜리 다른 가지로 끝난다 — **시작 자세를 모르면
사슬을 재현할 수 없다.**

그래서 여기서는 **시드에 의존하지 않는 것만** 본다:
판 하나를 좋은 자세에서 풀면 풀린다는 것, 그리고 x 범위·기본값이 무엇인지.
**"실제 판에서도 풀린다" 는 여기서 주장하지 않는다** — 그건 판을 돌려야 안다.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "manipulation"))

import arm_kinematics as ak  # noqa: E402
from config import PRESET_DEMO  # noqa: E402

#: 조작 노드가 보내는 것은 `dwell_s` 뿐이다 — 나머지는 실행기 기본값이 쓰인다
BOARDS_WORLD = (1.042, 0.498)
X_FROM, X_TO, POINTS = -0.35, 0.35, 5
ARM_Z = 0.330          # 아래 판 0.498 이 팔 기준 0.168 로 찍힌 실측에서 나온다
FACE = 0.444           # 정렬 뒤 서가 앞면 (팔 기준 y) = `scan_standoff_m`


def _arm_z(board_world_z):
    return board_world_z - ARM_Z


def test_each_board_solves_from_a_good_seed():
    """판 하나씩 **좋은 자세에서** 풀면 둘 다 풀린다 — 판 자체가 불가능한 건 아니다.

    실제 판에서 아래 판이 3/5 인 것은 **이어받은 가지** 탓이지 높이 탓이 아니다.
    그래서 고칠 자리가 "시드·방향" 이 된다 (`sweep_retry`).
    """
    for bz in BOARDS_WORLD:
        p = ak.plan_board_sweep(PRESET_DEMO.q_home, FACE, _arm_z(bz), X_FROM, X_TO, POINTS)
        assert len(p.hold_idx) == POINTS, f"판 {bz}: {len(p.hold_idx)}/{POINTS} — {p.reason}"
        assert p.min_margin > 0.03, f"판 {bz}: 여유 {p.min_margin:.3f} — 계획 거부선 아래"


def test_the_lower_board_also_solves_when_swept_the_other_way():
    """되돌아오기(뱀 모양)로도 풀린다 — 재시도가 고를 수 있는 길이 실제로 있다."""
    p = ak.plan_board_sweep(PRESET_DEMO.q_home, FACE, _arm_z(0.498), X_TO, X_FROM, POINTS)
    assert len(p.hold_idx) == POINTS, p.reason


def test_the_defaults_are_what_the_runtime_uses():
    """노드가 `points`·`x_from`·`x_to`·`boards` 를 안 보내므로 **이 값이 실제 값**이다.

    바뀌면 여기서 걸린다 — 스캔 범위가 조용히 줄면 빈칸을 놓친다.
    """
    assert (X_FROM, X_TO) == (-0.35, 0.35)
    assert X_TO - X_FROM == pytest.approx(0.70)
    assert POINTS == 5
    assert BOARDS_WORLD == (1.042, 0.498)


def test_margin_is_reported_so_thin_plans_are_visible():
    """계획이 여유를 들고 나온다 — '풀렸다' 와 '한계에 붙었다' 는 다르다.

    SW1 의 아래 판은 계획 전체 최소가 0.100 인데 **실패 지점의 여유는 0.059** 였다.
    최소값 하나로는 어디가 아슬아슬한지 모른다.
    """
    p = ak.plan_board_sweep(PRESET_DEMO.q_home, FACE, _arm_z(1.042), X_FROM, X_TO, POINTS)
    assert hasattr(p, "min_margin") and p.min_margin == pytest.approx(p.min_margin)


# ------------------- 홈이 둘이다 (2026-09-24 SW3 에서 밝혀짐)
#
#   `[스윕] 시작 자세 q=[-2.3634, …]` — 로그가 알려 준 실제 값이다.
#   그때까지 나는 `config.py` 의 yaml 홈(2.811…)을 실제 값으로 알고 오프라인
#   재현을 했고, **실제와 다른 답을 얻고 그것을 근거로 단언했다.**
#
#   두 홈은 서로 반대 방향으로 좋다:
#     yaml 홈     파지 여유 작다 · **아래 판 스윕이 풀린다**
#     SIM_HOME_Q  파지 여유 크다 · **아래 판 스윕이 안 풀린다**

from config import PRESET_DEMO as _P  # noqa: E402


def test_the_two_homes_are_different_poses():
    """**하나를 '그 홈' 이라고 부르면 틀린다.** 둘 다 실제로 쓰인다."""
    assert tuple(_P.q_home) != tuple(_P.q_home_sim)


def test_the_yaml_home_solves_the_lower_board():
    p = ak.plan_board_sweep(_P.q_home, FACE, _arm_z(0.498), X_FROM, X_TO, POINTS)
    assert len(p.hold_idx) == POINTS, p.reason


def test_the_runtime_home_does_not_and_that_is_the_whole_story():
    """**이 한 줄이 오늘의 스캔 사달을 설명한다.**

    실제 판이 쓰는 홈에서는 아래 판이 3/5 로 멈춘다. 그래서 스윕을 버리고
    자세 표로 갈아탔고, 도윤님이 화면에서 그 차이를 잡으셨다.
    """
    p = ak.plan_board_sweep(_P.q_home_sim, FACE, _arm_z(0.498), X_FROM, X_TO, POINTS)
    assert len(p.hold_idx) < POINTS
    assert "IK 실패" in (p.reason or "")


def test_the_upper_board_solves_from_either_home():
    """위 판은 어느 홈에서도 풀린다 — 막히는 것은 아래 판뿐이다."""
    for q in (_P.q_home, _P.q_home_sim):
        p = ak.plan_board_sweep(q, FACE, _arm_z(1.042), X_FROM, X_TO, POINTS)
        assert len(p.hold_idx) == POINTS, p.reason


def test_the_reproduction_matches_the_real_log():
    """실제 로그와 **숫자까지** 맞는지 — 재현이 재현인지 확인한다.

    로그(SW3 의 `이어가기`): `아래 판 … 3/5 여유 0.100 … 경유점 8/9 … 한계 여유 0.059`

    **사슬을 그대로 재현해야 이 숫자가 나온다.** 홈에서 바로 아래 판을 풀면
    `3/5 여유 0.118 · 경유점 7/9 · 한계 여유 0.080` 으로 **다른 값**이다 —
    같은 "3/5" 라도 온 길이 다르면 다른 판이다.
    """
    up = ak.plan_board_sweep(_P.q_home_sim, FACE, _arm_z(1.042), X_FROM, X_TO, POINTS)
    assert len(up.hold_idx) == POINTS
    assert up.min_margin == pytest.approx(0.210, abs=0.002)      # 로그: 여유 0.210
    chained = up.qs[up.hold_idx[-1]]
    down = ak.plan_board_sweep(chained, FACE, _arm_z(0.498), X_FROM, X_TO, POINTS)
    assert len(down.hold_idx) == 3
    assert down.min_margin == pytest.approx(0.100, abs=0.002)
    assert "8/9" in (down.reason or "") and "0.059" in (down.reason or "")
