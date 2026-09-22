"""
고정 좌표 지도 — waypoint 를 읽고 "어디에 어떻게 설 것인가"를 보정한다.

ROS 에 의존하지 않는다 (단위시험이 ROS 없이 돈다).

## 왜 보정이 필요한가

task_manager 가 주는 좌표는 "그 근처"일 뿐이다. 서가 앞에 **수평으로** 서야
로봇팔이 책을 꽂을 수 있고, 반납구는 **정면으로 마주봐야** 한다.
그 보정 규칙을 `behavior` 로 둔다.

| behavior | 뜻 |
| --- | --- |
| `as_is` | 받은 좌표 그대로 (home 등) |
| `approach_front` | 요소 앞면을 정면으로 마주보게 (반납구) |
| `park_parallel` | 앞면에서 `parking_distance` 만큼 떨어져, **옆 위치는 유지**하고 마주본다 (서가) |

`park_parallel` 이 옆 위치를 유지하는 이유: 긴 책장에서 **어느 칸 앞인지**는
task_manager 가 정한다. 우리는 "얼마나 떨어져서, 어느 방향을 보고" 만 고친다.

규약: 구성 요소 pose 의 orientation 은 **요소 앞면이 바라보는 방향**이다.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

BEHAVIOR_AS_IS = 'as_is'
BEHAVIOR_APPROACH_FRONT = 'approach_front'
BEHAVIOR_PARK_PARALLEL = 'park_parallel'

BEHAVIORS = (BEHAVIOR_AS_IS, BEHAVIOR_APPROACH_FRONT, BEHAVIOR_PARK_PARALLEL)

DEFAULT_FRAME_ID = 'map'
DEFAULT_PARKING_DISTANCE = 1.2


def default_behavior(component_type: str) -> str:
    """behavior 를 안 적었을 때 쓸 값. 서가는 수평 주차, 나머지는 그대로."""
    if (component_type or '').startswith('shelf'):
        return BEHAVIOR_PARK_PARALLEL
    if (component_type or '').startswith(('return', 'station', 'dock')):
        return BEHAVIOR_APPROACH_FRONT
    return BEHAVIOR_AS_IS


class MapLoadError(ValueError):
    """지도 파일이 규약을 어겼다. 조용히 넘어가면 엉뚱한 곳으로 간다."""


def normalize_angle(angle: float) -> float:
    """각도를 (-pi, pi] 로 접는다."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    # atan2 는 -pi 를 돌려주기도 한다. 시험이 +pi 를 기대하므로 맞춘다
    if wrapped <= -math.pi + 1e-12:
        return math.pi
    return wrapped


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """yaw(rad) → (x, y, z, w). 평면 주행이라 roll·pitch 는 0 이다."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """(x, y, z, w) → yaw(rad)."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _num(value, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


@dataclass(frozen=True)
class Waypoint:
    """지도 위의 한 자세. ROS PoseStamped 에 대응한다."""

    frame_id: str = DEFAULT_FRAME_ID
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0

    @property
    def yaw(self) -> float:
        """이 자세의 yaw(rad)."""
        return yaw_from_quaternion(
            self.orientation_x, self.orientation_y,
            self.orientation_z, self.orientation_w)

    @classmethod
    def from_dict(cls, data: Optional[dict], frame_id: str = DEFAULT_FRAME_ID) -> 'Waypoint':
        """position/orientation 딕셔너리에서 만든다. 빠진 값은 항등으로 둔다."""
        data = data or {}
        pos = data.get('position') or {}
        ori = data.get('orientation') or {}
        return cls(
            frame_id=data.get('frame_id') or frame_id or DEFAULT_FRAME_ID,
            position_x=_num(pos.get('x')),
            position_y=_num(pos.get('y')),
            position_z=_num(pos.get('z')),
            orientation_x=_num(ori.get('x')),
            orientation_y=_num(ori.get('y')),
            orientation_z=_num(ori.get('z')),
            orientation_w=_num(ori.get('w'), 1.0) if 'w' in ori else 1.0,
        )

    @classmethod
    def from_xy_yaw(cls, x: float, y: float, yaw: float,  # noqa: D102
                    frame_id: str = DEFAULT_FRAME_ID, z: float = 0.0) -> 'Waypoint':
        ox, oy, oz, ow = quaternion_from_yaw(yaw)
        return cls(frame_id=frame_id, position_x=x, position_y=y, position_z=z,
                   orientation_x=ox, orientation_y=oy, orientation_z=oz, orientation_w=ow)


@dataclass(frozen=True)
class MapComponent:
    """지도의 구성 요소 하나 (서가·반납구·home)."""

    component_id: str
    component_type: str
    pose: Waypoint
    behavior: str = BEHAVIOR_AS_IS
    parking_distance: Optional[float] = None

    @property
    def yaw(self) -> float:
        """요소 앞면의 yaw(rad)."""
        return self.pose.yaw

    @property
    def face_direction(self) -> Tuple[float, float]:
        """요소 **앞면**이 향하는 단위 벡터 (평면)."""
        yaw = self.pose.yaw
        return (math.cos(yaw), math.sin(yaw))

    @property
    def line_direction(self) -> Tuple[float, float]:
        """요소가 **늘어선** 방향 (앞면의 왼쪽). 긴 책장의 길이 방향이다."""
        yaw = self.pose.yaw
        return (-math.sin(yaw), math.cos(yaw))


class IsaacMap:
    """waypoint 모음. `waypoints:` 와 `components:` 두 형식을 모두 읽는다."""

    def __init__(self, components: Dict[str, MapComponent],
                 frame_id: str = DEFAULT_FRAME_ID):
        """구성 요소 맵과 기준 프레임으로 만든다."""
        self._components = dict(components)
        self.frame_id = frame_id or DEFAULT_FRAME_ID

    # -------------------------------------------------- 만들기
    @classmethod
    def empty(cls) -> 'IsaacMap':
        """빈 지도."""
        return cls({}, DEFAULT_FRAME_ID)

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> 'IsaacMap':
        """딕셔너리에서 지도를 만든다 (waypoints/components 두 형식)."""
        data = data or {}
        frame_id = data.get('frame_id') or DEFAULT_FRAME_ID
        raw = data.get('waypoints')
        if raw is None:
            raw = data.get('components')
        if raw is None:
            return cls({}, frame_id)
        if not isinstance(raw, dict):
            raise MapLoadError('waypoints/components 는 이름→값 맵이어야 한다')

        components = {}
        for name, entry in raw.items():
            entry = entry or {}
            component_type = entry.get('type') or cls._type_from_name(name)
            # **behavior 를 안 적으면 종류가 정한다.** 서가는 수평 주차가 기본이고,
            # 나머지는 그대로 간다. 매번 적게 하면 빠뜨렸을 때 조용히 틀어진다
            behavior = entry.get('behavior') or default_behavior(component_type)
            if behavior not in BEHAVIORS:
                raise MapLoadError(
                    f"'{name}' 의 behavior '{behavior}' 를 모른다. "
                    f"쓸 수 있는 값: {', '.join(BEHAVIORS)}")
            # pose 가 따로 묶여 있는 형식도 받는다
            pose_src = entry.get('pose') if isinstance(entry.get('pose'), dict) else entry
            pose = Waypoint.from_dict(pose_src, frame_id)
            parking = entry.get('parking_distance')
            components[name] = MapComponent(
                component_id=name,
                component_type=component_type,
                pose=pose,
                behavior=behavior,
                parking_distance=None if parking is None else _num(parking,
                                                                   DEFAULT_PARKING_DISTANCE),
            )
        return cls(components, frame_id)

    @classmethod
    def from_yaml_file(cls, path: str) -> 'IsaacMap':
        """YAML 파일에서 지도를 읽는다."""
        import yaml
        try:
            with open(path, encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
        except OSError as error:
            raise MapLoadError(f'지도 파일을 못 읽었다: {path} ({error})') from error
        return cls.from_dict(data)

    @staticmethod
    def _type_from_name(name: str) -> str:
        """이름에서 종류를 뽑는다 — `shelf_01` → `shelf`, `home` → `home`."""
        return name.rsplit('_', 1)[0] if '_' in name and name.rsplit('_', 1)[1].isdigit() else name

    # -------------------------------------------------- 읽기
    def __len__(self) -> int:
        """구성 요소 개수."""
        return len(self._components)

    def component_ids(self) -> Tuple[str, ...]:
        """구성 요소 이름들 (읽은 순서)."""
        return tuple(self._components.keys())

    def get_component(self, component_id: str) -> MapComponent:
        """없으면 KeyError. **조용히 None 을 주지 않는다** — 엉뚱한 곳으로 가게 된다."""
        return self._components[component_id]

    def find_component(self, target_type: Optional[str] = None,
                       target_id: Optional[str] = None) -> Optional[MapComponent]:
        """id 로 찾는다. id 를 안 주면 종류로 찾는다. 없으면 None (호출자가 판단한다).

        **id 를 줬는데 없으면 종류로 대체하지 않는다.** `shelf_99` 를 달라고 했는데
        `shelf_01` 을 주면 엉뚱한 서가 앞에 서게 된다 — 모르면 모른다고 해야 한다.
        """
        if target_id:
            return self._components.get(target_id)
        if target_type:
            for component in self._components.values():
                if component.component_type == target_type:
                    return component
        return None


def apply_behavior(component: MapComponent, source: Waypoint,
                   parking_distance: Optional[float] = None,
                   keep_lateral: bool = True,
                   correct_yaw: bool = True) -> Waypoint:
    """구성 요소의 `behavior` 에 따라 받은 좌표를 보정한다.

    `as_is` 는 **받은 것을 그대로 돌려준다** (같은 객체여도 된다).
    """
    if component is None or component.behavior == BEHAVIOR_AS_IS:
        return source

    distance = parking_distance
    if component.parking_distance is not None:
        distance = component.parking_distance
    if distance is None:
        distance = DEFAULT_PARKING_DISTANCE

    face_x, face_y = component.face_direction
    line_x, line_y = component.line_direction
    cx, cy = component.pose.position_x, component.pose.position_y

    # 요소 앞면에서 distance 만큼 떨어진 점
    base_x = cx + face_x * distance
    base_y = cy + face_y * distance

    if component.behavior == BEHAVIOR_PARK_PARALLEL and keep_lateral:
        # **옆 위치는 받은 좌표를 따른다** — 긴 책장에서 어느 칸 앞인지는 상위가 정한다
        lateral = (source.position_x - cx) * line_x + (source.position_y - cy) * line_y
        base_x += line_x * lateral
        base_y += line_y * lateral

    yaw = source.yaw
    if correct_yaw:
        # 요소를 마주본다 = 앞면의 반대 방향을 향한다
        yaw = normalize_angle(component.pose.yaw + math.pi)

    return Waypoint.from_xy_yaw(base_x, base_y, yaw,
                                frame_id=source.frame_id, z=source.position_z)
