# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import pytest

from shelving_navigation.isaac_map import (
    apply_behavior,
    BEHAVIOR_AS_IS,
    BEHAVIOR_PARK_PARALLEL,
    IsaacMap,
    MapLoadError,
    normalize_angle,
    quaternion_from_yaw,
    Waypoint,
    yaw_from_quaternion,
)


def test_quaternion_yaw_roundtrip():
    for yaw in (0.0, 0.5, math.pi / 2, math.pi, -math.pi / 2):
        q = quaternion_from_yaw(yaw)
        assert yaw_from_quaternion(*q) == pytest.approx(yaw, abs=1e-9)


def test_normalize_angle_wraps_to_pi():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(math.pi)


def test_waypoint_from_dict_defaults_to_identity():
    point = Waypoint.from_dict({}, 'map')
    assert point.position_x == 0.0
    assert point.position_y == 0.0
    assert point.position_z == 0.0
    assert point.orientation_w == 1.0
    assert point.yaw == pytest.approx(0.0)


def test_isaac_map_loads_flat_waypoints():
    data = {
        'frame_id': 'map',
        'waypoints': {
            'home': {
                'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
            'shelf_01': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 1.0, 'w': 0.0},
                'behavior': BEHAVIOR_PARK_PARALLEL,
                'parking_distance': 1.5,
            },
        },
    }
    isaac_map = IsaacMap.from_dict(data)
    assert len(isaac_map) == 2
    assert isaac_map.component_ids() == ('home', 'shelf_01')

    home = isaac_map.get_component('home')
    assert home.component_type == 'home'
    assert home.behavior == BEHAVIOR_AS_IS

    shelf = isaac_map.find_component(
        target_type='shelf',
        target_id='shelf_01',
    )
    assert shelf is not None
    assert shelf.component_id == 'shelf_01'
    assert shelf.parking_distance == 1.5


def test_isaac_map_handles_nested_pose():
    data = {
        'components': {
            'shelf_01': {
                'type': 'shelf',
                'pose': {
                    'position': {'x': 5.0, 'y': 6.0, 'z': 0.0},
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0,
                    },
                },
            },
        },
    }
    isaac_map = IsaacMap.from_dict(data)
    shelf = isaac_map.get_component('shelf_01')
    assert shelf.pose.position_x == 5.0
    assert shelf.pose.position_y == 6.0


def test_isaac_map_unknown_component_raises():
    isaac_map = IsaacMap.empty()
    with pytest.raises(KeyError):
        isaac_map.get_component('shelf_99')


def test_isaac_map_rejects_unknown_behavior():
    with pytest.raises(MapLoadError):
        IsaacMap.from_dict(
            {
                'waypoints': {
                    'shelf_01': {
                        'behavior': 'moonwalk',
                    },
                },
            }
        )


def test_apply_as_is_returns_source_unchanged():
    component = IsaacMap.from_dict(
        {
            'waypoints': {
                'home': {
                    'position': {'x': 1.0, 'y': 1.0, 'z': 0.0},
                },
            },
        }
    ).get_component('home')
    source = Waypoint.from_dict(
        {
            'position': {'x': 9.0, 'y': 9.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.5, 'w': 0.5},
        },
        'map',
    )
    result = apply_behavior(component, source)
    assert result == source


def test_park_parallel_aligns_robot_in_front_of_shelf():
    # 서가가 (0,0) 에서 yaw=0 (앞면 +X) 일 때 task_manager 가 준 좌표
    # (-3, -4) 를 앞면 1.2 m 앞, 같은 측면 위치로 보정한다.
    component = IsaacMap.from_dict(
        {
            'waypoints': {
                'shelf_01': {
                    'behavior': BEHAVIOR_PARK_PARALLEL,
                    'parking_distance': 1.2,
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0,
                    },
                },
            },
        }
    ).get_component('shelf_01')
    source = Waypoint.from_dict(
        {
            'position': {'x': -3.0, 'y': -4.0, 'z': 0.0},
            'orientation': {'z': 0.0, 'w': 1.0},
        },
        'map',
    )
    result = apply_behavior(component, source)
    assert result.position_x == pytest.approx(1.2)
    assert result.position_y == pytest.approx(-4.0)
    assert result.yaw == pytest.approx(math.pi)


def test_map_component_face_and_line_directions():
    _, _, oz, ow = quaternion_from_yaw(math.pi / 2)
    component = IsaacMap.from_dict(
        {
            'waypoints': {
                'shelf_01': {
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'z': oz, 'w': ow},  # yaw = pi/2
                },
            },
        }
    ).get_component('shelf_01')
    face_x, face_y = component.face_direction
    line_x, line_y = component.line_direction
    assert (face_x, face_y) == pytest.approx((0.0, 1.0))
    assert (line_x, line_y) == pytest.approx((-1.0, 0.0))
