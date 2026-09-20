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

from dataclasses import dataclass
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
class PlaceGoal:
    job_id: str
    book_id: str
    slot: SlotGoal
    book_width: float
    book_height: float
    book_thickness: float
    insertion_speed: float


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
    'grip_clearance': 0.005,           # 파지 전 벌림 여유 (한쪽)
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
    # 0.45 → 0.55: M0609 는 팔 베이스가 높아 꽂는 단이 팔 기준 0.5056 이다 (manipulation.yaml 과 같이 유지)
    'place_region_max': [-0.22, 0.65, 0.55],
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
        'pick': {'tray_slot': tray_slot.index, 'center': list(tray_slot.center)},
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
