"""
시연에 쓰는 좌표가 **우리 노드의 검증을 통과하는지** 고정한다.

왜 있나 (2026-09-20~21)
    로봇을 M0609 로 바꾸면서 꽂는 높이가 팔 기준 0.3399 → 0.5097 로 올라갔다.
    `place_region` 은 Franka 기준으로 z 상한이 0.45 였고, 그대로였다면 시연 당일
    **모든 목표가 410(요청 오류)으로 거절**됐을 것이다. 사람이 눈으로 맞추는 대신
    테스트가 잡게 한다.

    그리고 지금 M0609 의 바깥쪽 칸은 **여유가 9.7 mm 뿐**이다. 누가 상자를 조금만
    좁히면 조용히 거절이 시작된다 — 그 여유도 같이 고정한다.
"""

from pathlib import Path
import sys

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from shelving_manipulation.grasp_planner import DEFAULT_LIMITS  # noqa: E402

# 발행된 계약 좌표 (config/book_profiles.yaml 주석과 같아야 한다)
FRANKA = [(-0.5097, 0.5495, 0.3400), (-0.4297, 0.5495, 0.3400),
          (-0.3497, 0.5495, 0.3399), (-0.2697, 0.5495, 0.3398)]
M0609 = [(-0.4697, 0.5495, 0.5097), (-0.3897, 0.5495, 0.5097),
         (-0.3097, 0.5495, 0.5097), (-0.2297, 0.5495, 0.5097)]

# 책 높이의 절반 + 삽입 여유. book_profiles.yaml 의 유도와 같아야 한다
BOOK_HEIGHT = 0.2374
INSERT_CLEARANCE = 0.004


def _inside(p):
    lo = DEFAULT_LIMITS['place_region_min']
    hi = DEFAULT_LIMITS['place_region_max']
    return all(lo[i] <= p[i] <= hi[i] for i in range(3))


def _margin(p):
    lo = DEFAULT_LIMITS['place_region_min']
    hi = DEFAULT_LIMITS['place_region_max']
    return min(min(p[i] - lo[i], hi[i] - p[i]) for i in range(3))


@pytest.mark.parametrize('p', FRANKA, ids=lambda p: 'franka{:+.4f}'.format(p[0]))
def test_franka_좌표가_검증을_통과한다(p):
    """Franka 경로는 9/17 에 검증됐다. 이게 깨지면 대안 시연도 못 한다."""
    assert _inside(p), '{} 가 place_region 밖이다 — 410 으로 거절된다'.format(p)


@pytest.mark.parametrize('p', M0609, ids=lambda p: 'm0609{:+.4f}'.format(p[0]))
def test_m0609_좌표가_검증을_통과한다(p):
    """M0609 는 팔 베이스가 37cm 높아 꽂는 z 가 0.5097 이다."""
    assert _inside(p), (
        '{} 가 place_region 밖이다 — 410 으로 거절된다. '
        'place_region_max 의 z 를 0.45 로 되돌리지 않았는지 볼 것'.format(p))


def test_바깥쪽_칸의_여유가_줄지_않았다():
    """바깥쪽 칸은 여유가 1cm 도 안 된다. 상자를 좁히면 여기부터 거절된다."""
    worst = min(_margin(p) for p in M0609)
    assert worst >= 0.009, (
        '가장 좁은 여유가 {:.1f} mm 로 줄었다. '
        'place_region 을 좁혔거나 좌표를 옮겼다면 둘이 맞는지 확인할 것'.format(worst * 1000))


def test_두_로봇의_z_가_같은_식에서_나온다():
    """
    계약 z 는 '선반판 + 책높이/2 + 삽입여유 - 팔베이스' 로 재현돼야 한다.

    재현이 안 되면 숫자 안에 설명 안 되는 상수가 섞인 것이다.
    (2026-09-20: M0609 에서 삽입 여유 4mm 를 빠뜨려 0.5056 을 썼다 — 허용 오차의 절반)
    """
    cases = [  # (선반판 월드 z, 팔 베이스 월드 z, 계약 z)
        (0.4970, 0.280, 0.3399),   # Franka
        (1.0420, 0.655, 0.5097),   # M0609
    ]
    for floor_z, base_z, contract_z in cases:
        derived = floor_z + BOOK_HEIGHT / 2 + INSERT_CLEARANCE - base_z
        assert abs(derived - contract_z) < 0.001, (
            '선반판 {} / 베이스 {} → 유도 {:.4f} 인데 계약값은 {}. 차이 {:.1f} mm'.format(
                floor_z, base_z, derived, contract_z,
                abs(derived - contract_z) * 1000))
