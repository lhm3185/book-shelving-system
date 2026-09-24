"""스윕 스캔이 **두 층을 다 푼다** — 이게 비전팀이 구현한 동작이다.

2026-09-24 에 우리가 스캔 명령을 `scan_sweep`(부드러운 수평 스윕)에서
`scan_shelf`(옛 자세 표, 9자세 띄엄띄엄)로 바꿨다. 까닭으로 적힌 것이
**"아래 판에서 안 풀린다 (경유점 8/9 IK 실패, 여유 0.100 rad)"** 였다.

**그 숫자는 스윕의 것이 아니다.** 스윕은 정지점이 5개(`points` 기본 5)이고
조작 노드는 `points` 를 안 실어 보낸다. `8/9` 는 자세 표 스캔(`plan_scan`, 9자세)의
숫자다 — **안 풀리던 쪽으로 갈아탔다.**

여기서 스윕을 직접 돌려 확인한다. Isaac 없이 돈다 (순수 기구학).
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "manipulation"))

import arm_kinematics as ak  # noqa: E402
from config import PRESET_DEMO  # noqa: E402

#: 조작 노드가 보내는 것은 `dwell_s` 뿐이다 — 나머지는 실행기 기본값이 쓰인다.
#: 그래서 여기 값이 **실제로 도는 값**이다 (`manipulation_executor.start_sweep`).
BOARDS_WORLD = (1.042, 0.498)
X_FROM, X_TO, POINTS = -0.35, 0.35, 5
#: 팔 베이스 월드 z. 아래 판 0.498 이 팔 기준 0.168 로 찍힌 실측에서 나온다
ARM_Z = 0.330
FACE = 0.444          # 정렬 뒤 서가 앞면 (팔 기준 y). `scan_standoff_m` 과 같다


def _arm_z(board_world_z):
    return board_world_z - ARM_Z


def _chain(q0, boards, face=FACE, x_from=X_FROM, x_to=X_TO):
    """실행기와 **같은 순서로** 계획한다 — 다음 판은 앞 판의 끝 자세에서 시작한다."""
    q, out = list(q0), []
    for bz in boards:
        p = ak.plan_board_sweep(q, face, _arm_z(bz), x_from, x_to, POINTS)
        out.append(p)
        if p.hold_idx:
            q = p.qs[p.hold_idx[-1]]
    return out


def test_both_boards_solve_from_home():
    """**두 층이 다 풀린다.** 한 층만 훑고 끝나면 이 시험이 죽는다."""
    plans = _chain(PRESET_DEMO.q_home, BOARDS_WORLD)
    assert len(plans) == 2
    for bz, p in zip(BOARDS_WORLD, plans):
        assert len(p.hold_idx) == POINTS, f"판 {bz}: 정지점 {len(p.hold_idx)}/{POINTS} — {p.reason}"


def test_the_lower_board_is_not_the_problem():
    """"아래 판이 안 풀린다" 는 스윕에 해당하지 않는다 — 혼자 돌려도 풀린다."""
    p = ak.plan_board_sweep(PRESET_DEMO.q_home, FACE, _arm_z(0.498), X_FROM, X_TO, POINTS)
    assert len(p.hold_idx) == POINTS, p.reason
    assert p.min_margin > 0.10


def test_it_holds_across_standoff_and_start_pose():
    """서 있는 거리와 시작 자세가 달라져도 두 층이 풀린다 — 한 조건에서만 되는 게 아니다."""
    for face in (0.36, 0.444, 0.55, 0.75):
        for q0 in (PRESET_DEMO.q_home, PRESET_DEMO.q_stow):
            plans = _chain(q0, BOARDS_WORLD, face=face)
            got = [len(p.hold_idx) for p in plans]
            assert got == [POINTS, POINTS], f"앞면 {face}, 시작 {q0[:2]} → {got}"


def test_every_waypoint_keeps_a_margin():
    """여유가 계획 거부선(0.03 rad) 위다 — '풀렸다' 와 '한계에 붙었다' 는 다르다."""
    for p in _chain(PRESET_DEMO.q_home, BOARDS_WORLD):
        assert p.min_margin > 0.03, p.reason


def test_the_default_sweep_covers_the_shelf_width():
    """서가 안쪽 폭(1.405 m)을 팔 기준으로 덮는가 — 덜 덮으면 빈칸을 놓친다.

    기본 −0.35~+0.35 는 **0.70 m** 다. 서가 절반보다 조금 넓다. 차체가 서가
    중앙에 정렬한 뒤이므로 양쪽 0.35 m 씩이 실제로 닿는 범위다.
    **이 시험은 "충분하다" 를 주장하지 않는다** — 지금 값이 얼마인지를 박아,
    누가 바꾸면 눈에 띄게 한다.
    """
    assert X_TO - X_FROM == pytest.approx(0.70)
    assert (X_FROM, X_TO) == (-0.35, 0.35)
