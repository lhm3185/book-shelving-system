"""
PlaceBook 목표 검사와 시뮬 작업 명령 생성. ROS 에 의존하지 않는다.

좌표 계약 (docs/FRAMES_CONTRACT, 웹 클로드 v6·v7 회신)
- 기준 프레임 arm_base_link, REP-103(+X 앞, +Y 왼쪽, +Z 위), 단위 m
- 모든 물체 위치는 **AABB 중심**
- TargetSlot.position = **꽂힌 뒤 책의 AABB 중심**
- TargetSlot.orientation 의 +X 축 = **삽입 방향** (서가 안쪽). 1차 서가는 arm_base_link +Y 방향 → yaw +90°

책 치수 이름 (PlaceBook goal)
- book_thickness : 책등 폭 (두께)                 예 0.035
- book_height    : 책등 길이 (세웠을 때 높이)       예 0.237
- book_width     : 책등→앞마구리 (서가에서의 깊이)  예 0.163

1차 전제: 트레이 칸 좌표는 설정값(book_profiles.yaml)이다. 비전은 그림자 모드(동작에 쓰지 않음).
"""

from dataclasses import dataclass, replace
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .book_placer import COMMAND_PLACE, OK


@dataclass(frozen=True)
class BookDims:
    thickness: float
    height: float
    width: float


@dataclass(frozen=True)
class SlotGoal:
    """TargetSlot 을 ROS 없이 다루기 위한 값."""

    frame_id: str
    position: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    available_width: float
    available_height: float
    insertion_depth: float
    pre_insert_offset: float
    confidence: float
    age_s: Optional[float] = None       # 촬영 시각으로부터 경과. 모르면 None


@dataclass(frozen=True)
class GraspGoal:
    """
    비전이 본 파지 관측 (top_center 는 윗면 중심이며 AABB 중심이 아니다).

    이름을 나눈 이유: 같은 필드 이름에 서로 다른 점을 넣어 8cm·0.67m 가 어긋난 적이 있다
    (2026-09-17 사고 #3·#5). 계약: docs/doyoon-kim/manipulation/PICKPLACE_CONTRACT_V2.md 4절
    """

    frame_id: str
    top_center: Tuple[float, float, float]
    spine_yaw: float
    thickness: float
    width: float
    confidence: float
    age_s: Optional[float] = None


@dataclass(frozen=True)
class PlaceGoal:
    job_id: str
    book_id: str
    slot: SlotGoal
    book_width: float
    book_height: float
    book_thickness: float
    insertion_speed: float
    grasp: Optional[GraspGoal] = None


@dataclass
class Check:
    code: int
    message: str

    @property
    def ok(self) -> bool:
        return self.code == OK


# ------------------------------------------------------------------ 설정 기본값
# 실제 값은 config/manipulation.yaml(노드 파라미터)과 config/book_profiles.yaml 에서 덮어쓴다

DEFAULT_LIMITS = {
    'frame_id': 'arm_base_link',
    'finger_thickness': 0.0264,        # Franka Hand 손가락 하나 벌림축 두께 (Isaac 5.1.0 실측)
    'side_clearance': 0.005,           # 칸 옆 책과 한쪽 여유
    'height_clearance': 0.02,          # 책 위 여유
    # 파지 전 벌림 여유 (한쪽). **이 값이 곧 '손가락 ↔ 집을 책' 안쪽 여유다** —
    # 손가락 안쪽면이 책 반두께 + 이 값에 놓이므로 책 두께가 상쇄된다.
    # **기본값은 0.005 그대로 둔다** (기준선을 깨지 않기 위해).
    # 지금은 안쪽 5.0 / 바깥 8.3 mm 로 치우쳐 있다. 양쪽을 같게 하려면 0.00665 →
    # 양쪽 6.65 mm. 전환은 사람이 정한다 (2026-09-22 야간 보고서 참조).
    'grip_clearance': 0.005,
    # 좁은 쪽(안쪽)까지 보고 거절할 것인가. **기본 꺼짐** — 켜면 검사가 3.3 mm 엄해져
    # 기준선 성공률이 달라진다. 켜는 판단은 야간 측정(블록 2-3) 뒤에.
    'guard_inner_clearance': False,
    'insertion_yaw': math.pi / 2,      # 1차 서가 삽입 방향 (arm_base_link +Y)
    'yaw_tolerance': 0.10,             # rad
    'tilt_tolerance': 0.10,            # rad, 삽입 방향의 기울기
    'default_insertion_speed': 0.02,   # m/s (Isaac 검증: 책등 밀기 0.018 m/s)
    'max_insertion_speed': 0.05,
    'min_confidence': 0.0,             # 1차는 고정 칸이라 0. 비전 연결 시 올린다
    'max_target_age_s': 0.0,           # 0 이면 검사하지 않음 (Isaac sim time 정합 전)
    'require_slot_dimensions': False,  # False 면 폭·높이 0 은 '미제공' 으로 보고 넘어간다
    'max_book_dimension': 0.5,
    # 검증된 꽂기 범위 (Isaac 레벨 v3, 책 4곳 × 트레이 6칸 계획 통과 영역 + 여유)
    'place_region_min': [-0.56, 0.45, 0.25],
    # 0.45 → 0.55: M0609 는 팔 베이스가 높아 꽂는 단이 팔 기준 0.5097 이다 (manipulation.yaml 과 같이 유지).
    # 이 값이 계약 좌표를 담는지는 test/test_contract_coords.py 가 지킨다
    'place_region_max': [-0.22, 0.65, 0.55],
    # 파지 y 를 칸 중심에 맞출 것인가. **기본 꺼짐 — 아직 검증 전이다.**
    # 근거: 9/21 에 '설정 칸 y 가 실제와 어긋난다' 며 껐는데, 그 측정이 좌표계가
    # 4.36° 돌아간 상태에서 읽은 값이었다. 기울기를 걷어내면 칸별 퍼짐이 0 이다.
    # 복귀 보정이 들어간 지금 다시 재 봐야 한다 (A/B 용 스위치).
    'snap_grasp_y': False,
}


def _merged(limits: Optional[dict]) -> dict:
    merged = dict(DEFAULT_LIMITS)
    if limits:
        merged.update({k: v for k, v in limits.items() if v is not None})
    return merged


# ------------------------------------------------------------------ 기하

def yaw_and_tilt(q_xyzw: Sequence[float]) -> Tuple[float, float]:
    """쿼터니언의 +X 축(삽입 방향)이 수평면에서 이루는 yaw 와, 수평면에서 벗어난 기울기."""
    x, y, z, w = q_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0, math.pi
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    ax = 1 - 2 * (y * y + z * z)
    ay = 2 * (x * y + w * z)
    az = 2 * (x * z - w * y)
    return math.atan2(ay, ax), math.asin(max(-1.0, min(1.0, az)))


def angle_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def closed_gripper_width(limits: dict) -> float:
    return 2 * limits['finger_thickness']


def required_slot_width(book: BookDims, limits: dict) -> float:
    """책등을 닫은 그리퍼로 밀어 넣을 때 손가락도 칸 안으로 들어간다 → 책과 닫힌 손가락 중 넓은 쪽."""
    return max(book.thickness, closed_gripper_width(limits)) + 2 * limits['side_clearance']


def grip_open_per_finger(book: BookDims, limits: dict) -> float:
    """옆 칸 책을 치지 않을 만큼만 벌린다 (Isaac: 크게 벌리면 옆 책 충돌)."""
    return book.thickness / 2 + limits['grip_clearance']


# ------------------------------------------------------------------ 검사

def resolve_book(goal: PlaceGoal, profile: BookDims,
                 limits: Optional[dict] = None) -> Tuple[Optional[BookDims], Check]:
    """책 치수가 셋 다 0 이면 기본 프로파일, 일부만 0 이면 잘못된 목표."""
    lim = _merged(limits)
    dims = (goal.book_thickness, goal.book_height, goal.book_width)
    if all(d == 0 for d in dims):
        return profile, Check(OK, '')
    if any((not math.isfinite(d)) or d <= 0 or d > lim['max_book_dimension'] for d in dims):
        return None, Check(410, f'책 치수가 잘못됨 두께 {dims[0]} 높이 {dims[1]} 폭 {dims[2]}')
    return BookDims(*dims), Check(OK, '')


def validate_grasp(goal: PlaceGoal, tray_slots, limits: Optional[dict] = None) -> Check:
    """
    비전이 준 파지 관측을 쓰기 전에 검사한다 (계약 5절).

    핵심은 높이 검사다 — 트레이에 세운 책은 윗면이 칸 중심에서 **책 폭(W)의 절반**만큼
    위에 있다. 비전이 윗면 대신 책등 중앙이나 AABB 중심을 보냈으면 **여기서 수 cm 가
    어긋난다.** 계약 위반을 이름뿐 아니라 **물리량으로도** 잡는 것이다.
    """
    lim = _merged(limits)
    g = goal.grasp
    if g is None:
        return Check(OK, '')
    if g.frame_id != lim['frame_id']:
        return Check(410, f"파지 관측 frame_id '{g.frame_id}' ≠ '{lim['frame_id']}'")
    if not all(math.isfinite(v) for v in g.top_center):
        return Check(410, '파지 관측 top_center 에 NaN/inf')
    if lim['max_target_age_s'] > 0 and g.age_s is not None \
            and g.age_s > lim['max_target_age_s']:
        return Check(411, f'파지 관측이 오래됨 {g.age_s:.1f}s')
    if tray_slots:
        xs = [sl.center[0] for sl in tray_slots]
        ys = [sl.center[1] for sl in tray_slots]
        if not (min(xs) - 0.10 <= g.top_center[0] <= max(xs) + 0.10):
            return Check(410, f'파지 관측 x {g.top_center[0]:+.3f} 가 트레이 범위 밖')
        if not (min(ys) - 0.15 <= g.top_center[1] <= max(ys) + 0.15):
            return Check(410, f'파지 관측 y {g.top_center[1]:+.3f} 가 트레이 범위 밖')
        want_top = tray_slots[0].center[2] + goal.book_width / 2.0
        if goal.book_width > 0 and abs(g.top_center[2] - want_top) > 0.01:
            return Check(410,
                         f'관측 높이가 책 규격과 다르다: top z {g.top_center[2]:.4f}, '
                         f'기대 {want_top:.4f} (칸 중심 + 책 폭/2). '
                         f'윗면 중심이 아닌 다른 점을 보냈을 수 있다')
    if g.thickness > 0 and goal.book_thickness > 0 \
            and abs(g.thickness - goal.book_thickness) > 0.005:
        return Check(410, f'관측 두께 {g.thickness:.4f} ≠ 규격 {goal.book_thickness:.4f}')
    # **손가락이 무언가에 닿지 않는가.** 여기가 가장 좁다.
    # 손이 x 로 어긋나면 **양쪽이 동시에** 줄어든다 — 한쪽 손가락은 옆 책으로,
    # 반대쪽 손가락은 집을 책으로 간다. 그래서 **둘 중 좁은 쪽**이 허용치다.
    #
    #   안쪽 여유 = 손가락 안쪽면 − 책 반두께 = grip_clearance 그 자체 (책 두께가 상쇄된다)
    #   바깥 여유 = (칸 간격 − 책 반두께) − 손가락 바깥면
    #
    # 2026-09-21 까지 **바깥쪽만 보고 있었다** (8.3 mm). 실제로 먼저 닿는 것은
    # 안쪽(5.0 mm)이라 검사가 3.3 mm 만큼 느슨했다.
    if tray_slots and len(tray_slots) >= 2 and goal.book_thickness > 0:
        centers = sorted(sl.center[0] for sl in tray_slots)
        pitch = min(b - a for a, b in zip(centers, centers[1:]))
        book = BookDims(goal.book_thickness, goal.book_height, goal.book_width)
        finger_inner = grip_open_per_finger(book, lim)
        finger_outer = finger_inner + lim['finger_thickness']
        inner_gap = finger_inner - goal.book_thickness / 2
        outer_gap = (pitch - goal.book_thickness / 2) - finger_outer
        # 기본은 바깥쪽만 본다 (지금까지의 동작). `guard_inner_clearance` 를 켜면
        # 좁은 쪽까지 본다 — 손이 어긋나면 양쪽이 동시에 줄기 때문에 그쪽이 옳지만,
        # 검사가 엄해져 기준선이 달라지므로 전환은 측정 뒤에 사람이 정한다
        margin = min(inner_gap, outer_gap) if lim['guard_inner_clearance'] else outer_gap
        side = '집을 책' if margin == inner_gap else '옆 책'
        nearest = min(centers, key=lambda c: abs(c - g.top_center[0]))
        dx = abs(g.top_center[0] - nearest)
        if margin > 0 and dx > margin:
            return Check(410,
                         f'파지 좌표가 칸 중심에서 {dx*1000:.1f} mm 어긋났다 — '
                         f'여유 {margin*1000:.1f} mm 를 넘는다. 손가락이 {side} 에 닿는다 '
                         f'(안쪽 {inner_gap*1000:.1f} / 바깥 {outer_gap*1000:.1f} mm, '
                         f'칸 간격 {pitch*1000:.0f} mm)')
    return Check(OK, '')


def snap_grasp_to_slot(goal: PlaceGoal, tray_slots, limits: Optional[dict] = None):
    """
    비전이 준 x 를 가장 가까운 트레이 칸 중심에 맞춘다 → (새 goal, 알린 글 또는 '').

    왜 이렇게 하나: 트레이는 좌표를 **아는** 고정 지그다. 비전이 정할 것은
    "몇 번 칸에 있는 책인가"이고, 그 칸의 정확한 x 는 지그 형상이 이미 알고 있다.
    비전 x 는 실측에서 한 판마다 20 mm 까지 흔들렸는데(2026-09-21, 같은 장면에서
    -0.3620 → -0.3424), 손가락 여유는 8.3 mm 뿐이라 그대로 쓰면 옆 책을 친다.

    **칸 간격의 절반을 넘게 벗어나면 손대지 않는다.** 그때는 어느 칸인지 알 수 없고,
    엉뚱한 칸으로 당겨 붙이면 옆 책을 집으러 간다 — validate_grasp 가 거절하게 둔다.
    z 는 건드리지 않는다 (높이는 책 규격에서 나오고 validate_grasp 가 따로 검사한다).
    y 는 `snap_grasp_y` 가 켜졌을 때만 맞춘다.
    """
    lim = _merged(limits)
    g = goal.grasp
    if g is None or not tray_slots or len(tray_slots) < 2:
        return goal, ''
    centers = sorted(sl.center[0] for sl in tray_slots)
    pitch = min(b - a for a, b in zip(centers, centers[1:]))
    slot = min(tray_slots, key=lambda sl: abs(sl.center[0] - g.top_center[0]))
    dx = g.top_center[0] - slot.center[0]
    if abs(dx) >= pitch / 2.0:
        return goal, ''

    # y 는 **스위치가 켜졌을 때만** 맞춘다 (`snap_grasp_y`). 9/21 에 한 번 껐는데,
    # 끈 근거였던 측정이 좌표계가 4.36° 돌아간 상태에서 읽은 값이었다 —
    # 기울기를 걷어내면 칸별 퍼짐이 0 이므로 '칸마다 어긋난다' 는 틀린 결론이었다.
    # 복귀 보정이 들어간 지금 다시 재야 해서 A/B 스위치로 둔다.
    dy = 0.0
    new_y = g.top_center[1]
    if lim['snap_grasp_y']:
        dy = g.top_center[1] - slot.center[1]
        # 책 길이의 절반을 넘게 벗어났으면 그 책을 보고 있는 것이 아니다 — 손대지 않는다
        y_limit = goal.book_height / 2.0 if goal.book_height > 0 else 0.10
        if abs(dy) < y_limit:
            new_y = slot.center[1]

    if abs(dx) < 1e-6 and new_y == g.top_center[1]:
        return goal, ''
    snapped = replace(g, top_center=(slot.center[0], new_y, g.top_center[2]))
    moved_y = '' if new_y == g.top_center[1] else f', y {dy * 1000:+.1f} mm'
    return (replace(goal, grasp=snapped),
            f'파지 좌표를 칸 중심에 맞췄다: x {g.top_center[0]:+.4f} → {slot.center[0]:+.4f} '
            f'({dx * 1000:+.1f} mm{moved_y}, 칸 간격 {pitch * 1000:.0f} mm)')


def grasp_pick_center(goal: PlaceGoal):
    """
    비전 윗면 중심을 로봇팔이 쓰는 AABB 중심으로 바꾼다 (관측이 없으면 None).

    세운 책의 윗면은 AABB 중심에서 **책 폭(W)의 절반**만큼 위다 (좌표 계약 2번).
    유도는 치수를 아는 쪽(로봇팔)이 한다 — 비전에게 가려진 면까지 추정하게 하지 않는다.
    """
    g = goal.grasp
    if g is None or goal.book_width <= 0:
        return None
    x, y, z = g.top_center
    return [float(x), float(y), float(z) - goal.book_width / 2.0]


def validate_goal(goal: PlaceGoal, book: BookDims, limits: Optional[dict] = None) -> Check:
    lim = _merged(limits)
    s = goal.slot
    if not goal.job_id or not goal.book_id:
        return Check(410, 'job_id 와 book_id 가 필요하다')
    if s.frame_id != lim['frame_id']:
        return Check(410, f"TargetSlot frame_id '{s.frame_id}' ≠ '{lim['frame_id']}'")
    if not all(math.isfinite(v) for v in s.position):
        return Check(410, 'TargetSlot position 에 NaN/inf')
    if lim['max_target_age_s'] > 0:
        if s.age_s is None:
            return Check(410, 'TargetSlot timestamp 없음')
        if s.age_s > lim['max_target_age_s']:
            return Check(410, f"TargetSlot 이 오래됨 {s.age_s:.1f}s > {lim['max_target_age_s']}s")
    if s.confidence < lim['min_confidence']:
        return Check(410, f"TargetSlot 신뢰도 {s.confidence:.2f} < {lim['min_confidence']}")

    if sum(v * v for v in s.orientation_xyzw) < 1e-12:
        return Check(410, 'TargetSlot orientation 이 비어 있음 (0 쿼터니언)')
    yaw, tilt = yaw_and_tilt(s.orientation_xyzw)
    if angle_diff(yaw, lim['insertion_yaw']) > lim['yaw_tolerance']:
        want = math.degrees(lim['insertion_yaw'])
        tol = math.degrees(lim['yaw_tolerance'])
        return Check(410, f'삽입 방향 yaw {math.degrees(yaw):.1f}° — 1차 지원 {want:.0f}°±{tol:.0f}°')
    if abs(tilt) > lim['tilt_tolerance']:
        return Check(410, f'삽입 방향이 수평에서 {math.degrees(tilt):.1f}° 기울어짐')

    provided = s.available_width > 0 or s.available_height > 0
    if lim['require_slot_dimensions'] or provided:
        need_w = required_slot_width(book, lim)
        if s.available_width < need_w:
            return Check(410, f'칸 폭 {s.available_width:.3f} < 필요 {need_w:.3f} m')
        need_h = book.height + lim['height_clearance']
        if s.available_height < need_h:
            return Check(410, f'칸 높이 {s.available_height:.3f} < 필요 {need_h:.3f} m')

    lo, hi = lim['place_region_min'], lim['place_region_max']
    if not all(lo[i] <= s.position[i] <= hi[i] for i in range(3)):
        return Check(410, f'꽂을 위치 {tuple(round(v, 3) for v in s.position)} 가 검증 범위 밖 '
                          f'{lo}~{hi}')
    if goal.insertion_speed < 0:
        return Check(410, f'삽입 속도 음수 {goal.insertion_speed}')
    return Check(OK, '')


def insertion_speed(goal: PlaceGoal, limits: Optional[dict] = None) -> float:
    """목표의 삽입 속도는 '제한' 이다. 0 이면 기본값, 최대값을 넘으면 최대값."""
    lim = _merged(limits)
    requested = goal.insertion_speed
    if requested <= 0:
        requested = lim['default_insertion_speed']
    return min(requested, lim['max_insertion_speed'])


# ------------------------------------------------------------------ 트레이 칸

@dataclass(frozen=True)
class TraySlot:
    index: int
    center: Tuple[float, float, float]


def select_tray_slot(book_id: str, slots: Sequence[TraySlot], assignments: Dict[str, int],
                     used: Iterable[int]) -> Tuple[Optional[TraySlot], Check]:
    """1차: book_id 지정 칸이 있으면 그 칸, 없으면 아직 꺼내지 않은 첫 칸."""
    by_index = {s.index: s for s in slots}
    used = set(used)
    if book_id in assignments:
        index = int(assignments[book_id])
        if index not in by_index:
            return None, Check(410, f'book_id {book_id} 의 트레이 칸 {index} 이 설정에 없음')
        if index in used:
            return None, Check(411, f'트레이 칸 {index} 은 이미 꺼냄 (book_id {book_id})')
        return by_index[index], Check(OK, '')
    for slot in sorted(slots, key=lambda s: s.index):
        if slot.index not in used:
            return slot, Check(OK, '')
    return None, Check(411, '트레이에 남은 책이 없음')


def parse_tray(config: dict) -> Tuple[List[TraySlot], Dict[str, int]]:
    tray = config.get('tray', {}) or {}
    slots = [TraySlot(int(s['index']), tuple(float(v) for v in s['center']))
             for s in tray.get('slots', [])]
    assignments = {str(k): int(v) for k, v in (tray.get('assignments') or {}).items()}
    return slots, assignments


def parse_profile(config: dict, name: str = 'default') -> BookDims:
    p = config['profiles'][name]
    return BookDims(float(p['thickness']), float(p['height']), float(p['width']))


# ------------------------------------------------------------------ 명령

def build_place_command(token: str, goal: PlaceGoal, book: BookDims, tray_slot: TraySlot,
                        limits: Optional[dict] = None) -> dict:
    lim = _merged(limits)
    yaw, _ = yaw_and_tilt(goal.slot.orientation_xyzw)
    return {
        'type': COMMAND_PLACE,
        'token': token,
        'job_id': goal.job_id,
        'book_id': goal.book_id,
        'frame_id': lim['frame_id'],
        'book': {'thickness': book.thickness, 'height': book.height, 'width': book.width},
        # 비전 관측이 있으면 **그 좌표로 집는다**. 없으면 설정의 트레이 칸 (기존 동작)
        'pick': {'tray_slot': tray_slot.index,
                 'center': grasp_pick_center(goal) or list(tray_slot.center),
                 'source': 'vision' if goal.grasp is not None else 'config'},
        'place': {
            'center': list(goal.slot.position),
            'yaw': yaw,
            'insertion_depth': goal.slot.insertion_depth,
            'pre_insert_offset': goal.slot.pre_insert_offset,
        },
        'grip': {
            'open_per_finger': grip_open_per_finger(book, lim),
            'closed_width': closed_gripper_width(lim),
        },
        'insertion_speed': insertion_speed(goal, lim),
    }
