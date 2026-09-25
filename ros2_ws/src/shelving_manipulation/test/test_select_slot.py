"""
빈칸 선택 — '판이 비었다' 관측도 실측 빈칸을 거친다.

2026-09-25 00:15 실측: 비전 관측이 전부 서가 밖으로 걸러진 판에서 '아래 판이 비어 있음'
분기가 열렸고, 그 분기는 실측 빈칸을 안 보고 아래 판 x -0.35 를 목표로 박았다. 아래 판
실측은 14 / 10 mm 조각뿐인데 35.2 mm 책을 밀어 넣으러 가 손에서 1.1 cm 밀렸다(406).
"못 한다" 대신 "꽉 찬 데 밀어 넣는다" — 401 보다 나쁘다. 여기서 그 우회로를 막는다.
"""

import os
import sys
import types

from geometry_msgs.msg import PointStamped

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from shelving_manipulation.manipulation_node import ManipulationNode  # noqa: E402

LOWER_Z, UPPER_REL = 0.3399, 1.042 - 0.498


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warning(self, m):
        self.lines.append(m)


def _stub(shelf_gaps):
    s = types.SimpleNamespace()
    s.limits = {'frame_id': 'arm_base_link', 'side_clearance': 0.005}
    s._shelf_box = [-0.60, 0.60, 0.493, 0.793, 0.15, 1.10]      # 팔 기준 (x0 x1 y앞 y뒤 z0 z1)
    s.scan_standoff_m = 0.444
    s.place_standoff_m = 0.444
    s.place_x = -0.35
    s.shelf_x_min, s.shelf_x_max = -0.5, 0.2
    s.slot_y_from_shelf_front = True
    s.slot_center_inset = 0.024
    s.slot_x_snap_to_gap = True
    s.slot_x_snap_max_m = 0.12
    s._shelf_gaps = shelf_gaps
    s._shelf_gaps_at = 0
    s._place_count = 0
    s.profile = types.SimpleNamespace(thickness=0.0352)
    s._slot_reject = None
    log = _Log()
    s.get_logger = lambda: log
    return s


def _open_board_obs():
    """판이 비어 뒤가 보인 관측 — 서가 x 폭 안, y 는 서가 너머(2.9 m)."""
    m = PointStamped()
    m.header.frame_id = 'arm_base_link'
    m.point.x, m.point.y, m.point.z = -0.20, 2.9, 0.40
    return m


def test_아래_판이_꽉_찼으면_위_판으로_옮긴다():
    """실측: 아래 판 14 / 10 mm 조각, 위 판 55 mm. 비전은 '아래 판이 비었다' 고 한다."""
    s = _stub([[-0.400, -0.386, 0.168], [-0.300, -0.290, 0.168], [0.100, 0.155, 0.712]])
    out = ManipulationNode._select_empty_slot(s, [_open_board_obs()], 0.1517)
    assert out is not None, s._slot_reject
    _msg, (x, y, z) = out
    assert x == -0.35, '팔이 꽂는 x 는 늘 -0.35 다 — 차체가 옆으로 간다'
    assert abs(z - (LOWER_Z + UPPER_REL)) < 1e-6, '높이가 위 판으로 같이 옮겨졌다'
    assert abs(s.place_lateral_dynamic - (0.1275 - (-0.35))) < 1e-3, '차체를 위 판 빈칸으로 옮긴다'
    assert any('판을 옮긴다' in ln for ln in s.get_logger().lines)


def test_어느_판에도_자리가_없으면_거절한다_밀어_넣지_않는다():
    s = _stub([[-0.400, -0.386, 0.168], [-0.300, -0.290, 0.168]])
    out = ManipulationNode._select_empty_slot(s, [_open_board_obs()], 0.1517)
    assert out is None
    assert s._slot_reject and '빈칸이 없다' in s._slot_reject


def test_실측_빈칸이_없으면_예전과_같은_자리다():
    """스냅할 근거가 없을 때는 검증된 아래 판 자리(-0.35, 계약 y, 0.3399) — 동작이 안 바뀐다."""
    s = _stub(None)
    out = ManipulationNode._select_empty_slot(s, [_open_board_obs()], 0.1517)
    assert out is not None
    _msg, (x, y, z) = out
    assert (x, round(y, 4), z) == (-0.35, 0.5495, LOWER_Z)
    assert s.place_lateral_dynamic == 0.0


def test_책_두께는_요청_값을_쓴다_프로파일_기본이_아니라():
    """네 권 판: 44.3 mm 책이 35.2 로 재져 여유가 9.1 mm 부풀려졌다. 요청 두께로 들어가는 칸을 가른다."""
    s = _stub([[-0.400, -0.360, 0.168], [0.100, 0.155, 0.712]])         # 아래 40 mm · 위 55 mm
    thin = ManipulationNode._select_empty_slot(s, [_open_board_obs()], 0.1517, 0.0352)
    assert thin is not None and abs(thin[1][2] - LOWER_Z) < 1e-6          # 35.2 는 아래 40 mm 칸에 들어간다
    s2 = _stub([[-0.400, -0.360, 0.168], [0.100, 0.155, 0.712]])
    thick = ManipulationNode._select_empty_slot(s2, [_open_board_obs()], 0.1517, 0.0443)
    assert thick is not None and abs(thick[1][2] - (LOWER_Z + UPPER_REL)) < 1e-6   # 44.3 은 위 판으로 옮긴다
    assert any('[배정] 책 44.3 mm' in ln for ln in s2.get_logger().lines)
