"""프리셋과 **실제로 숫자를 낸 실행기**가 같은지 본다.

2026-09-24 에 `config.py` 는 스스로를 "숫자의 유일한 출처" 라고 적어 놓고 있었는데,
**아무도 그것을 읽지 않았다.** `PRESET_DEMO` 를 import 하는 코드가 한 줄도 없었고,
그 사이 적힌 값이 실제로 돌린 값과 세 군데 갈라져 있었다:

    carry_mode          swing  ← 적힘        joint  ← 실제로 돈 것
    return_mode         swing                joint
    release_open_first  True                 False
    hand_drift_m        (없음 → 코드 기본 0.03)   0.25

**한 번도 돌린 적 없는 조합이 "검증된 조합" 이라는 이름을 달고 있었다.** 그대로
두면 저장소를 받은 사람이 그 이름을 믿고 돌린다.

그래서 이 시험은 프리셋을 `run_vision.sh` 의 기본 조합과 **글자 단위로** 맞춘다.
어느 쪽을 고쳐도 다른 쪽을 안 고치면 여기서 걸린다.
"""

import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "manipulation"))

from config import PRESET_DEMO, goal_y_for  # noqa: E402

RUN_VISION = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "..", "..", "cli_exchange", "scripts", "run_vision.sh")


def _harness_defaults():
    """`run_vision.sh` 의 기본 조합 블록에서 `export SIM_*=...` 를 읽는다.

    스크립트를 읽는다. 값을 여기 옮겨 적으면 옮겨 적는 순간 갈라지고, 갈라진 것을
    아무도 모른다 — 이 시험이 막으려는 바로 그 일이다.
    """
    out, inside = {}, False
    for line in open(RUN_VISION, encoding="utf-8"):
        if 'NO_COMBO:-0' in line:
            inside = True
            continue
        if inside and line.strip() == "fi":
            break
        if not inside or not line.lstrip().startswith("export "):
            continue
        for k, v in re.findall(r"(SIM_[A-Z0-9_]+)=(\S+)", line):
            out[k] = v
    return out


HARNESS = _harness_defaults()


def test_the_harness_block_was_actually_found():
    """스크립트 모양이 바뀌어 아무것도 못 읽었는데 통과하면 안 된다."""
    assert len(HARNESS) >= 8, HARNESS
    assert "SIM_BOOK_COLL" in HARNESS


def test_the_preset_matches_the_harness():
    """**프리셋이 실제로 돈 조합과 같아야 한다.** 한쪽만 고치면 여기서 걸린다."""
    env = PRESET_DEMO.env()
    mismatched = {k: (env.get(k), v) for k, v in HARNESS.items() if env.get(k) != v}
    assert not mismatched, f"프리셋 != 실행기: {mismatched}"


def test_the_three_that_were_wrong_stay_fixed():
    """갈라져 있던 셋을 회귀로 박는다 — 실행기가 안 건드리니 코드 기본이 곧 실제다."""
    assert PRESET_DEMO.carry_mode == "joint"
    assert PRESET_DEMO.return_mode == "joint"
    assert PRESET_DEMO.release_open_first is False
    assert PRESET_DEMO.hand_drift_m == 0.25
    # 실행기가 이 셋을 안 건드린다는 것 자체도 확인한다 (건드리면 위 시험이 잡는다)
    for k in ("SIM_CARRY_MODE", "SIM_RETURN_MODE", "SIM_RELEASE_OPEN_FIRST"):
        assert k not in HARNESS


def test_goal_y_is_derived_not_written():
    """짝 규칙은 상수가 아니라 관계다 — 선 자리가 바뀌면 목표도 같이 간다."""
    assert goal_y_for(-3.019) == pytest.approx(0.5495)
    assert goal_y_for(-3.0695) == pytest.approx(0.6000, abs=1e-9)
    # 베이스를 50.5 mm 뒤로 물리면 팔 기준 목표 y 가 그만큼 늘어난다
    assert goal_y_for(-3.0695) - goal_y_for(-3.019) == pytest.approx(0.0505, abs=1e-9)
    assert PRESET_DEMO.goal_y == pytest.approx(goal_y_for(PRESET_DEMO.pick_y))
