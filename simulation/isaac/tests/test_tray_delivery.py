"""트레이 이송 좌표 단위시험 — Isaac 없이 돈다 (순수 산수).

이 파일이 없어서 2026-09-23 밤을 넘겼다. 판단이 `book_scene.py` 안에 있었고,
그 파일은 GPU 가 있어야 한 줄이라도 읽히기 때문에 **틀린 것을 확인할 방법이
없었다.** 결함은 산수였다 — "레벨에서 잰 값" 이라 써 놓고 상수로 박았다.
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from tray_delivery import (  # noqa: E402
    deliver_from_to, drift_mm, TRAY_FROM_FALLBACK, TRAY_TO_XY)

#: 2026-09-24 레벨의 트레이 원점 (데스크탑 실측)
LEVEL = [5.817607391996635, -5.659500598907469, 0.333765]
#: 그날 밤 데스크탑이 손으로 넘기던 값. **이것이 기본값으로 나와야 한다**
DESKTOP_FROM = [5.817607, -5.659501, 0.333765]
DESKTOP_TO = [4.993110, -5.659414, 0.333765]


def test_default_reads_the_level_not_a_constant():
    """환경변수 없이 레벨 자리가 그대로 나온다 — 이것이 이 모듈의 존재 이유다."""
    frm, to, src = deliver_from_to(LEVEL, env={})
    assert src == "레벨"
    assert frm == pytest.approx(LEVEL, abs=1e-12)
    assert to[:2] == pytest.approx(TRAY_TO_XY, abs=1e-12)


def test_destination_z_equals_start_z():
    """도착 z 는 **적지 않는다 — 출발을 따라간다** (2026-09-23 도윤님 지시).

    옛 코드는 출발·도착 z 를 따로 박아 두었다. 레벨이 내려가자 출발만 보정되고
    도착은 그 차이만큼 **더** 끌려 내려가 데크 아래를 겨눴다.
    """
    for z in (0.333765, 0.37, 0.28):
        frm, to, _ = deliver_from_to([1.0, 2.0, z], env={})
        assert to[2] == pytest.approx(frm[2], abs=1e-12)


def test_matches_what_the_desktop_passed_by_hand():
    """손으로 넘기던 두 환경변수가 이제 필요 없다."""
    frm, to, _ = deliver_from_to(LEVEL, env={})
    assert frm == pytest.approx(DESKTOP_FROM, abs=1e-6)
    assert to == pytest.approx(DESKTOP_TO, abs=1e-6)


def test_env_wins_for_ab_comparison():
    """못 박으면 그것이 이긴다 — 회귀 비교를 막지 않는다."""
    frm, to, src = deliver_from_to(LEVEL, env={"SIM_TRAY_FROM": "1,2,3",
                                               "SIM_TRAY_TO": "4,5,6"})
    assert src == "SIM_TRAY_FROM"
    assert frm.tolist() == [1.0, 2.0, 3.0] and to.tolist() == [4.0, 5.0, 6.0]


def test_pinned_start_still_drags_the_destination_z():
    """출발만 못 박아도 도착 z 는 따라간다 — 둘이 따로 놀지 않는다."""
    _frm, to, _ = deliver_from_to(LEVEL, env={"SIM_TRAY_FROM": "1,2,3"})
    assert to[2] == pytest.approx(3.0, abs=1e-12)


def test_falls_back_only_when_the_level_tray_is_missing():
    """레벨 트레이가 없을 때만 대체값을 쓰고, 그 사실을 말한다."""
    frm, to, src = deliver_from_to(None, env={})
    assert "대체값" in src
    assert frm == pytest.approx(TRAY_FROM_FALLBACK, abs=1e-12)
    assert to[2] == pytest.approx(TRAY_FROM_FALLBACK[2], abs=1e-12)


def test_drift_reports_a_changed_level():
    """레벨이 바뀌면 mm 로 말한다 — 조용히 썩지 않게."""
    assert drift_mm(LEVEL) == pytest.approx(0.0, abs=1e-6)
    # 옛 상수(0.37)와 지금 레벨의 차이가 그날 밤의 3.6 cm 다
    assert drift_mm([0, 0, 0.37]) == pytest.approx(36.235, abs=1e-3)


def test_start_is_a_copy_not_the_level_array():
    """반환값을 고쳐도 레벨에서 읽어 온 배열이 바뀌지 않는다."""
    level = np.array(LEVEL, float)
    frm, _to, _ = deliver_from_to(level, env={})
    frm[2] = 99.0
    assert level[2] == pytest.approx(LEVEL[2], abs=1e-12)
