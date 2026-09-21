"""
주행 목표 계획 — task_manager 가 준 좌표를 "실제로 설 자리"로 바꾼다.

ROS 에 의존하지 않는다 (단위시험이 ROS 없이 돈다).

## 무엇을 하나

`NavigateToTarget` 으로 들어온 `target_type`/`target_id`/`pose` 를 지도와 맞춰,
`isaac_map.apply_behavior` 로 보정한 목표를 돌려준다.

**지도에 없는 요소면 받은 좌표를 그대로 쓴다.** 모른다고 멈추지 않는다 —
지도에 없는 곳으로 가라는 지시도 유효할 수 있고, 판단은 상위(task_manager)가 한다.
다만 **조용히 다른 곳으로 바꾸지는 않는다.**
"""

from dataclasses import dataclass
from typing import Optional

from .isaac_map import (
    apply_behavior,
    DEFAULT_FRAME_ID,
    DEFAULT_PARKING_DISTANCE,
    IsaacMap,
    Waypoint,
)


@dataclass(frozen=True)
class ParkingConfig:
    """주차 보정 설정. 지도에 `parking_distance` 가 있으면 그쪽이 이긴다."""

    parking_distance: float = DEFAULT_PARKING_DISTANCE
    keep_lateral: bool = True      # 긴 책장에서 옆 위치를 받은 좌표대로 둘지
    correct_yaw: bool = True       # 요소를 마주보게 방향을 고칠지
    clamp_side: bool = True        # (예약) 책장 길이 밖으로 못 나가게
    position_tolerance: float = 0.10
    yaw_tolerance: float = 0.10


@dataclass(frozen=True)
class PlannedTarget:
    """계획된 주행 목표."""

    target_type: str
    target_id: str
    pose: Waypoint
    position_tolerance: float
    yaw_tolerance: float
    component_id: Optional[str] = None      # 지도에서 찾은 요소. 못 찾으면 None
    corrected: bool = False                 # 보정이 실제로 일어났는가


class MovingPlanner:
    """지도를 보고 주행 목표를 정한다. 지도가 없으면 받은 좌표를 그대로 쓴다."""

    def __init__(self, isaac_map: Optional[IsaacMap] = None,
                 config: Optional[ParkingConfig] = None):
        self.map = isaac_map
        self.config = config or ParkingConfig()

    def plan_target(self, target_type: str, target_id: str, frame_id: str,
                    position: dict, orientation: dict,
                    position_tolerance: float = 0.0,
                    yaw_tolerance: float = 0.0) -> PlannedTarget:
        """받은 목표를 보정해 돌려준다.

        허용오차가 0 이면 **설정 기본값**을 쓴다 — 0 을 그대로 쓰면 어떤 오차도
        용납하지 않게 되어 영원히 도착하지 못한다.
        """
        source = Waypoint.from_dict(
            {'position': position or {}, 'orientation': orientation or {}},
            frame_id or DEFAULT_FRAME_ID)

        component = None
        if self.map is not None:
            component = self.map.find_component(target_type=target_type, target_id=target_id)

        if component is None:
            # 지도에 없다 → 받은 좌표 그대로. 바꾸지 않는다
            return PlannedTarget(
                target_type=target_type,
                target_id=target_id,
                pose=source,
                position_tolerance=position_tolerance or self.config.position_tolerance,
                yaw_tolerance=yaw_tolerance or self.config.yaw_tolerance,
                component_id=None,
                corrected=False,
            )

        planned = apply_behavior(
            component, source,
            parking_distance=self.config.parking_distance,
            keep_lateral=self.config.keep_lateral,
            correct_yaw=self.config.correct_yaw,
        )
        return PlannedTarget(
            target_type=target_type,
            target_id=target_id,
            pose=planned,
            position_tolerance=position_tolerance or self.config.position_tolerance,
            yaw_tolerance=yaw_tolerance or self.config.yaw_tolerance,
            component_id=component.component_id,
            corrected=planned is not source,
        )

    def plan_route(self, names, frame_id: str = DEFAULT_FRAME_ID):
        """지도에 있는 이름들을 **순서대로** 도는 경로. 순회 주행에 쓴다.

        이름이 지도에 없으면 **조용히 건너뛰지 않고** KeyError 를 낸다 —
        한 점이 빠진 채로 도는 것은 대개 실수다.
        """
        if self.map is None:
            raise KeyError('지도가 없다')
        route = []
        for name in names:
            component = self.map.get_component(name)
            route.append(PlannedTarget(
                target_type=component.component_type,
                target_id=component.component_id,
                pose=apply_behavior(
                    component, component.pose,
                    parking_distance=self.config.parking_distance,
                    keep_lateral=False,               # 순회는 지도 좌표 그대로 선다
                    correct_yaw=self.config.correct_yaw,
                ),
                position_tolerance=self.config.position_tolerance,
                yaw_tolerance=self.config.yaw_tolerance,
                component_id=component.component_id,
                corrected=component.behavior != 'as_is',
            ))
        return route
