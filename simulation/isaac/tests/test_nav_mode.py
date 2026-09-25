"""주행 모드 스위치 — 순간이동과 Nav2 가 같은 조인트를 건드리므로 한 번에 하나만."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from nav_mode import read_nav_mode, teleport_allowed  # noqa: E402


def test_기본은_teleport_지금_동작():
    assert read_nav_mode({}) == ("teleport", "")
    assert teleport_allowed("teleport", "goto") and teleport_allowed("teleport", "patrol")


def test_nav2_모드는_방_단위_주행을_거절하고_취소는_받는다():
    mode, note = read_nav_mode({"SIM_NAV_MODE": "nav2"})
    assert mode == "nav2" and note == ""
    assert not teleport_allowed(mode, "goto") and not teleport_allowed(mode, "patrol")
    assert teleport_allowed(mode, "cancel")


def test_모르는_값은_teleport_로_두고_말한다():
    mode, note = read_nav_mode({"SIM_NAV_MODE": "NAV3"})
    assert mode == "teleport" and "모르는 값" in note
