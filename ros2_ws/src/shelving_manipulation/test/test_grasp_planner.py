"""목표 검사·트레이 칸 선택·명령 생성 단위시험 (ROS 없이)."""

from dataclasses import replace
import math
from pathlib import Path

import pytest
from shelving_manipulation.grasp_planner import (
    BookDims, build_place_command, DEFAULT_LIMITS, parse_profile, parse_tray, PlaceGoal,
    required_slot_width, resolve_book, select_tray_slot, SlotGoal, TraySlot, validate_goal,
    yaw_and_tilt)
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'book_profiles.yaml'
BOOK = BookDims(0.0353, 0.2374, 0.1631)
YAW90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
VERIFIED = (-0.3497, 0.5495, 0.3399)     # Isaac 검증 위치 중 하나
_S, _C = math.sin(math.pi / 4), math.cos(math.pi / 4)
TILTED = (-_S * math.sin(0.15), _C * math.sin(0.15), _S * math.cos(0.15), _C * math.cos(0.15))


def goal(**kw):
    slot = {'frame_id': 'arm_base_link', 'position': VERIFIED, 'orientation_xyzw': YAW90,
            'available_width': 0.0, 'available_height': 0.0, 'insertion_depth': 0.16,
            'pre_insert_offset': 0.05, 'confidence': 1.0, 'age_s': None}
    slot.update(kw.pop('slot', {}))
    g = {'job_id': 'j1', 'book_id': 'b1', 'slot': SlotGoal(**slot), 'book_width': 0.0,
         'book_height': 0.0, 'book_thickness': 0.0, 'insertion_speed': 0.0}
    g.update(kw)
    return PlaceGoal(**g)


def test_verified_goal_passes():
    assert validate_goal(goal(), BOOK).ok


@pytest.mark.parametrize('slot, words', [
    ({'frame_id': 'base_link'}, 'frame_id'),
    ({'frame_id': 'camera_color_optical_frame'}, 'frame_id'),
    ({'orientation_xyzw': (0.0, 0.0, 0.0, 1.0)}, 'yaw'),          # 비전 현재 기본값(단위 쿼터니언)
    ({'orientation_xyzw': (0.0, 0.0, 0.0, 0.0)}, '비어'),
    ({'orientation_xyzw': TILTED}, '기울'),                         # yaw 90° + 0.3 rad 기울임
    ({'position': (float('nan'), 0.5, 0.3)}, 'NaN'),
    ({'position': (-0.35, 0.55, 1.2)}, '범위'),
    ({'available_width': 0.04, 'available_height': 0.4}, '폭'),    # 책은 들어가도 닫힌 손가락이 안 들어감
    ({'available_width': 0.2, 'available_height': 0.24}, '높이'),
])
def test_invalid_slot_is_410(slot, words):
    check = validate_goal(goal(slot=slot), BOOK)
    assert check.code == 410 and words in check.message


def test_missing_ids_410():
    assert validate_goal(goal(job_id=''), BOOK).code == 410


def test_confidence_and_age_checks_when_enabled():
    limits = {'min_confidence': 0.5, 'max_target_age_s': 1.0}
    assert validate_goal(goal(slot={'confidence': 0.3, 'age_s': 0.1}), BOOK, limits).code == 410
    assert validate_goal(goal(slot={'confidence': 0.9, 'age_s': None}), BOOK, limits).code == 410
    assert validate_goal(goal(slot={'confidence': 0.9, 'age_s': 2.0}), BOOK, limits).code == 410
    assert validate_goal(goal(slot={'confidence': 0.9, 'age_s': 0.5}), BOOK, limits).ok


def test_slot_width_counts_closed_fingers():
    need = required_slot_width(BOOK, DEFAULT_LIMITS)
    assert need == pytest.approx(2 * 0.0264 + 2 * 0.005)
    assert validate_goal(goal(slot={'available_width': need, 'available_height': 0.3}), BOOK).ok


def test_zero_dims_use_profile_partial_dims_rejected():
    book, check = resolve_book(goal(), BOOK)
    assert check.ok and book == BOOK
    _, check = resolve_book(goal(book_thickness=0.03), BOOK)
    assert check.code == 410
    book, check = resolve_book(goal(book_thickness=0.03, book_height=0.2, book_width=0.15), BOOK)
    assert check.ok and book == BookDims(0.03, 0.2, 0.15)


def test_negative_speed_rejected_and_limit_clamped():
    assert validate_goal(goal(insertion_speed=-0.1), BOOK).code == 410
    cmd = build_place_command('t', goal(insertion_speed=0.5), BOOK, TraySlot(0, (0, 0, 0)))
    assert cmd['insertion_speed'] == DEFAULT_LIMITS['max_insertion_speed']
    cmd = build_place_command('t', goal(), BOOK, TraySlot(0, (0, 0, 0)))
    assert cmd['insertion_speed'] == DEFAULT_LIMITS['default_insertion_speed']
    cmd = build_place_command('t', goal(insertion_speed=0.01), BOOK, TraySlot(0, (0, 0, 0)))
    assert cmd['insertion_speed'] == 0.01


def test_yaw_and_tilt():
    yaw, tilt = yaw_and_tilt(YAW90)
    assert yaw == pytest.approx(math.pi / 2) and tilt == pytest.approx(0.0)
    s = math.sin(0.3 / 2)
    _, tilt = yaw_and_tilt((0.0, s, 0.0, math.cos(0.3 / 2)))     # y축 0.3 rad → +X 가 아래로
    assert tilt == pytest.approx(-0.3)


def test_tray_slot_selection():
    slots = [TraySlot(i, (float(i), 0.0, 0.0)) for i in range(3)]
    s, c = select_tray_slot('b', slots, {}, used=[])
    assert c.ok and s.index == 0
    s, c = select_tray_slot('b', slots, {}, used=[0, 1])
    assert s.index == 2
    s, c = select_tray_slot('b', slots, {}, used=[0, 1, 2])
    assert s is None and c.code == 411
    s, c = select_tray_slot('b', slots, {'b': 1}, used=[0])
    assert s.index == 1
    assert select_tray_slot('b', slots, {'b': 1}, used=[1])[1].code == 411
    assert select_tray_slot('b', slots, {'b': 9}, used=[])[1].code == 410


def test_package_config_matches_contract():
    cfg = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    profile = parse_profile(cfg)
    slots, assignments = parse_tray(cfg)
    assert cfg['tray']['frame_id'] == 'arm_base_link'
    assert len(slots) == 6 and assignments == {}
    pitches = [b.center[0] - a.center[0] for a, b in zip(slots, slots[1:])]
    assert all(p == pytest.approx(0.075, abs=1e-3) for p in pitches)
    assert profile.thickness < profile.width < profile.height


def test_vision_grasp_becomes_aabb_center():
    """비전은 윗면 중심을 주고, 로봇팔이 책 폭의 절반을 빼 AABB 중심으로 바꾼다."""
    from shelving_manipulation.grasp_planner import GraspGoal, grasp_pick_center
    g = goal(book_width=BOOK.width, book_height=BOOK.height,
             book_thickness=BOOK.thickness)
    top_z = 0.1119 + g.book_width / 2          # 칸 중심 + 책 폭/2 = 윗면
    with_grasp = replace(g, grasp=GraspGoal(
        frame_id='arm_base_link', top_center=(-0.5123, 0.0788, top_z),
        spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width, confidence=0.9))
    center = grasp_pick_center(with_grasp)
    assert center == pytest.approx([-0.5123, 0.0788, 0.1119], abs=1e-6)
    cmd = build_place_command('tok', with_grasp, BOOK, TraySlot(2, (-0.51, 0.08, 0.11)))
    assert cmd['pick']['source'] == 'vision'
    assert cmd['pick']['center'] == pytest.approx([-0.5123, 0.0788, 0.1119], abs=1e-6)


def test_vision_grasp_wrong_point_is_rejected():
    """윗면이 아닌 점(예: AABB 중심)을 보내면 높이 검사에서 걸린다 — 계약 위반을 물리량으로 잡는다."""
    from shelving_manipulation.grasp_planner import GraspGoal, validate_grasp
    g = goal(book_width=BOOK.width, book_height=BOOK.height,
             book_thickness=BOOK.thickness)
    slots = [TraySlot(0, (-0.6623, 0.0788, 0.1119)), TraySlot(1, (-0.5873, 0.0788, 0.1119))]
    # 윗면 대신 AABB 중심을 보냈다 → 책 폭의 절반만큼 낮다
    bad = replace(g, grasp=GraspGoal(
        frame_id='arm_base_link', top_center=(-0.6623, 0.0788, 0.1119),
        spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width, confidence=0.9))
    check = validate_grasp(bad, slots)
    assert check.code == 410 and '높이' in check.message
    # 프레임이 틀려도 거절한다
    wrong_frame = replace(g, grasp=GraspGoal(
        frame_id='base_link', top_center=(-0.6623, 0.0788, 0.1119 + g.book_width / 2),
        spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width, confidence=0.9))
    assert validate_grasp(wrong_frame, slots).code == 410


def test_vision_grasp_offset_would_hit_neighbour():
    """칸 중심에서 너무 벗어난 파지 좌표는 거절한다 — 손가락이 옆 책에 닿는다."""
    from shelving_manipulation.grasp_planner import GraspGoal, validate_grasp
    g = goal(book_width=BOOK.width, book_height=BOOK.height,
             book_thickness=BOOK.thickness)
    slots = [TraySlot(i, (-0.6623 + 0.075 * i, 0.0788, 0.1119)) for i in range(6)]
    top_z = 0.1119 + g.book_width / 2

    def obs(x):
        return replace(g, grasp=GraspGoal(
            frame_id='arm_base_link', top_center=(x, 0.0788, top_z),
            spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width,
            confidence=0.9))

    # 칸 중심 그대로면 통과한다
    assert validate_grasp(obs(-0.5123), slots).ok
    # 4 mm 어긋남 — 여유(약 8 mm) 안이라 통과한다
    assert validate_grasp(obs(-0.5163), slots).ok
    # 16 mm 어긋남 — 2026-09-21 에 바깥쪽 칸에서 실제로 나온 오차. 거절해야 한다
    check = validate_grasp(obs(-0.5283), slots)
    assert check.code == 410 and '옆 책' in check.message


def test_command_contains_contract_fields():
    cmd = build_place_command('tok', goal(), BOOK, TraySlot(2, (-0.51, 0.08, 0.11)))
    assert cmd['type'] == 'place_book' and cmd['token'] == 'tok'
    assert cmd['frame_id'] == 'arm_base_link'
    # 비전 관측이 없으면 설정의 트레이 칸을 쓰고, source 로 그 사실을 남긴다
    assert cmd['pick'] == {'tray_slot': 2, 'center': [-0.51, 0.08, 0.11],
                           'source': 'config'}
    assert cmd['place']['center'] == list(VERIFIED)
    assert cmd['place']['yaw'] == pytest.approx(math.pi / 2)
    # 벌림 = 책 반두께 + grip_clearance. 이 여유가 곧 '손가락 ↔ 집을 책' 안쪽 간격이다
    assert cmd['grip']['open_per_finger'] == pytest.approx(BOOK.thickness / 2 + 0.005)


def _obs_goal(x):
    """트레이 6칸 위에 선 책을 x 에서 봤다는 관측을 담은 목표."""
    from shelving_manipulation.grasp_planner import GraspGoal
    g = goal(book_width=BOOK.width, book_height=BOOK.height, book_thickness=BOOK.thickness)
    slots = [TraySlot(i, (-0.6623 + 0.075 * i, 0.0788, 0.1119)) for i in range(6)]
    top_z = 0.1119 + g.book_width / 2
    return replace(g, grasp=GraspGoal(
        frame_id='arm_base_link', top_center=(x, 0.0788, top_z),
        spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width,
        confidence=0.9)), slots


def test_snap_pulls_vision_x_onto_slot_centre():
    """여유(8.3 mm)를 넘는 어긋남도 칸 중심으로 당겨 붙인다 — 트레이는 좌표를 아는 지그다."""
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot, validate_grasp
    g, slots = _obs_goal(-0.3424)                    # 2026-09-21 실측, 칸 중심에서 19.9 mm
    assert not validate_grasp(g, slots).ok           # 보정 전에는 거절당한다
    fixed, note = snap_grasp_to_slot(g, slots)
    assert fixed.grasp.top_center[0] == pytest.approx(-0.3623, abs=1e-9)
    assert validate_grasp(fixed, slots).ok           # 보정 후에는 통과한다
    assert '19.9 mm' in note


def test_snap_leaves_y_alone_by_default():
    """기본값에서는 y 를 건드리지 않는다 — 아직 검증 전인 보정이다."""
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot
    g, slots = _obs_goal(-0.3424)
    off = replace(g, grasp=replace(g.grasp, top_center=(
        -0.3424, 0.0788 + 0.044, g.grasp.top_center[2])))
    fixed, _ = snap_grasp_to_slot(off, slots)
    assert fixed.grasp.top_center[1:] == off.grasp.top_center[1:]


def test_snap_y_when_switch_on():
    """`snap_grasp_y` 를 켜면 y 도 칸 중심으로 맞춘다."""
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot
    g, slots = _obs_goal(-0.3424)
    off = replace(g, grasp=replace(g.grasp, top_center=(
        -0.3424, 0.0788 + 0.044, g.grasp.top_center[2])))
    fixed, note = snap_grasp_to_slot(off, slots, {'snap_grasp_y': True})
    assert fixed.grasp.top_center[1] == pytest.approx(0.0788, abs=1e-9)
    assert '+44.0 mm' in note


def test_snap_y_keeps_z_and_respects_half_book():
    """켜져 있어도 z 는 그대로고, 책 길이 절반을 넘는 y 는 손대지 않는다."""
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot
    g, slots = _obs_goal(-0.3623)
    far = replace(g, grasp=replace(g.grasp, top_center=(
        -0.3623, 0.0788 + 0.13, g.grasp.top_center[2])))
    fixed, note = snap_grasp_to_slot(far, slots, {'snap_grasp_y': True})
    assert fixed.grasp.top_center[1] == pytest.approx(0.0788 + 0.13, abs=1e-9)
    assert fixed.grasp.top_center[2] == far.grasp.top_center[2]
    assert note == ''


def test_snap_does_not_move_beyond_half_pitch():
    """**칸 간격 절반을 넘으면 손대지 않는다** — 어느 칸인지 알 수 없다.

    엉뚱한 칸으로 당겨 붙이면 옆 책을 집으러 간다. 그때는 검사가 거절하게 둔다.
    """
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot, validate_grasp
    g, slots = _obs_goal(-0.5123 + 0.0375)           # 정확히 두 칸 사이
    fixed, note = snap_grasp_to_slot(g, slots)
    assert fixed is g and note == ''
    assert not validate_grasp(fixed, slots).ok


def test_snap_is_noop_without_observation():
    """비전 관측이 없으면 아무 일도 하지 않는다 (1차 고정 칸 경로)."""
    from shelving_manipulation.grasp_planner import snap_grasp_to_slot
    g = goal()
    slots = [TraySlot(i, (-0.6623 + 0.075 * i, 0.0788, 0.1119)) for i in range(6)]
    assert snap_grasp_to_slot(g, slots) == (g, '')
    assert snap_grasp_to_slot(g, []) == (g, '')


def test_clearance_guard_uses_the_narrower_side():
    """**안쪽(손가락↔집을 책)과 바깥쪽(손가락↔옆 책) 중 좁은 쪽**이 허용치다.

    손이 x 로 어긋나면 양쪽이 동시에 줄어든다. 2026-09-21 까지 바깥쪽만 보고 있어
    검사가 느슨했다 (안쪽 5.0 mm 인데 바깥 8.3 mm 로 판정).
    """
    from shelving_manipulation.grasp_planner import (DEFAULT_LIMITS, GraspGoal,
                                                     grip_open_per_finger, validate_grasp)
    g = goal(book_width=BOOK.width, book_height=BOOK.height, book_thickness=BOOK.thickness)
    slots = [TraySlot(i, (-0.6623 + 0.075 * i, 0.0788, 0.1119)) for i in range(6)]
    top_z = 0.1119 + g.book_width / 2

    inner = grip_open_per_finger(BOOK, DEFAULT_LIMITS) - BOOK.thickness / 2
    outer = (0.075 - BOOK.thickness / 2) - (grip_open_per_finger(BOOK, DEFAULT_LIMITS)
                                            + DEFAULT_LIMITS['finger_thickness'])
    narrow = min(inner, outer)

    def obs(x):
        return replace(g, grasp=GraspGoal(
            frame_id='arm_base_link', top_center=(x, 0.0788, top_z),
            spine_yaw=0.0, thickness=g.book_thickness, width=g.book_width, confidence=0.9))

    lim = dict(DEFAULT_LIMITS, guard_inner_clearance=True)
    # 켜면 좁은 쪽이 허용치다 — 넓은 쪽 값으로 판정하면 안 된다
    assert validate_grasp(obs(-0.5123 + narrow * 0.9), slots, lim).ok
    assert not validate_grasp(obs(-0.5123 + narrow * 1.1), slots, lim).ok
    # **기본은 꺼져 있다** — 기준선을 깨지 않기 위해 지금까지의 동작(바깥쪽만)을 유지한다
    assert validate_grasp(obs(-0.5123 + narrow * 1.1), slots).ok


def test_clearances_are_currently_lopsided():
    """지금은 안쪽·바깥쪽 여유가 치우쳐 있다 — 균형값(0.00665)으로 바꾸는 판단은 사람이 한다.

    이 시험은 "치우쳐 있음"을 **기록**한다. 균형을 맞추면 이 시험이 깨지는데,
    그때가 바로 사람이 전환을 결정한 시점이어야 한다.
    """
    from shelving_manipulation.grasp_planner import DEFAULT_LIMITS, grip_open_per_finger
    inner = grip_open_per_finger(BOOK, DEFAULT_LIMITS) - BOOK.thickness / 2
    outer = (0.075 - BOOK.thickness / 2) - (grip_open_per_finger(BOOK, DEFAULT_LIMITS)
                                            + DEFAULT_LIMITS['finger_thickness'])
    assert inner == pytest.approx(0.005, abs=1e-6), '안쪽 여유 = grip_clearance 그 자체'
    assert outer == pytest.approx(0.0083, abs=1e-4)
    assert min(inner, outer) == inner, '좁은 쪽은 안쪽이다 — 허용치는 이 값이어야 한다'
