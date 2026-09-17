"""작업 경유점 계산 테스트 — 오늘 겪은 실패를 규칙으로 고정한다."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from book_tasks import (BACKOFF_BEHIND_SPINE, FINGER_THICK, TIP_DOWN, BookBox,  # noqa: E402
                        ShelfSlot, finger_outer_half_width, grip_open_width,
                        min_neighbor_pitch, spine_out_job)

BOOK = BookBox(np.array([2.155, -3.058, 0.308]), np.array([2.190, -2.821, 0.471]))   # 두께 3.5 길이 23.7 폭 16.3
SLOT = ShelfSlot(x=2.485, front_y=-2.575, floor_z=0.497)
HOME = np.array([2.36, -2.94, 0.77])


def job():
    return spine_out_job(BOOK, SLOT, HOME)


def test_grasp_is_near_spine_top():
    assert abs(job().points["grasp"][2] - (BOOK.hi[2] - TIP_DOWN)) < 1e-9


def test_gripper_closes_only_after_fingertips_clear_spine():
    """놓고 빠지는 도중 닫으면 표지를 감싼 채 닫혀 책이 딸려 나왔다 (6회 중 2회)"""
    j = job()
    spine_when_wedged = j.points["wedge"][1] - TIP_DOWN
    assert j.points["back"][1] <= spine_when_wedged - BACKOFF_BEHIND_SPINE + 1e-9
    names = [n for n, _ in j.segments]
    assert names.index("back") < names.index("touch") < names.index("push")


def test_forward_edge_outside_shelf_before_wedging():
    j = job()
    fore_edge_y = j.points["pre_ins"][1] - TIP_DOWN + BOOK.width
    assert fore_edge_y < SLOT.front_y


def test_final_spine_inside_front():
    assert job().spine_final_y > SLOT.front_y


def test_book_bottom_above_slot_floor_when_wedged():
    j = job()
    bottom = j.points["wedge"][2] - BOOK.length / 2
    assert 0 < bottom - SLOT.floor_z < 0.01


def test_tray_open_width_does_not_hit_neighbors_at_tray_pitch():
    """트레이 칸 간격 7.5cm 에서 손가락이 옆 책에 닿지 않는다 (4cm 로 벌리면 닿는다)"""
    pitch = 0.075
    assert min_neighbor_pitch(BOOK) <= pitch
    wide_open = 0.04
    assert finger_outer_half_width(wide_open) + BOOK.thick / 2 > pitch


def test_open_width_clears_book():
    assert grip_open_width(BOOK) > BOOK.thick / 2
    assert job().grip_hold < BOOK.thick / 2        # 눌러 잡는다


def test_push_is_slowest_segment():
    j = job()
    assert j.speeds["push"] == min(j.speeds.values())


def test_every_job_returns_home():
    j = job()
    assert np.allclose(j.segments[0][1][0][0], HOME) and np.allclose(j.segments[-1][1][-1][0], HOME)
