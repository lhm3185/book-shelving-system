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

from shelving_navigation.isaac_map import IsaacMap
from shelving_navigation.moving_planner import MovingPlanner, ParkingConfig

DATA = {
    'frame_id': 'map',
    'waypoints': {
        'home': {
            'position': {'x': 1.0, 'y': 1.0, 'z': 0.0},
        },
        'shelf_01': {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'z': 0.0, 'w': 1.0},
            'parking_distance': 1.5,
        },
    },
}

POSITION = {'x': -3.0, 'y': -4.0, 'z': 0.0}
ORIENTATION = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def test_plan_target_corrects_shelf_pose():
    planner = MovingPlanner(IsaacMap.from_dict(DATA))
    planned = planner.plan_target(
        target_type='shelf',
        target_id='shelf_01',
        frame_id='map',
        position=POSITION,
        orientation=ORIENTATION,
        position_tolerance=0.12,
        yaw_tolerance=0.2,
    )
    assert planned.target_id == 'shelf_01'
    assert planned.position_tolerance == 0.12
    assert planned.yaw_tolerance == 0.2
    assert planned.pose.position_x == pytest.approx(1.5)
    assert planned.pose.position_y == pytest.approx(-4.0)
    assert planned.pose.yaw == pytest.approx(math.pi)


def test_plan_target_unknown_component_keeps_raw_pose():
    planner = MovingPlanner(IsaacMap.from_dict(DATA))
    planned = planner.plan_target(
        target_type='shelf',
        target_id='shelf_99',
        frame_id='map',
        position=POSITION,
        orientation=ORIENTATION,
    )
    assert planned.pose.position_x == pytest.approx(-3.0)
    assert planned.pose.position_y == pytest.approx(-4.0)
    assert planned.pose.yaw == pytest.approx(0.0)


def test_plan_target_home_behavior_is_as_is():
    planner = MovingPlanner(IsaacMap.from_dict(DATA))
    planned = planner.plan_target(
        target_type='home',
        target_id='home',
        frame_id='map',
        position=POSITION,
        orientation=ORIENTATION,
    )
    assert planned.pose.position_x == pytest.approx(-3.0)
    assert planned.pose.position_y == pytest.approx(-4.0)


def test_parking_config_controls_correction():
    data = {
        'waypoints': {
            'shelf_01': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'z': 0.0, 'w': 1.0},
            },
        },
    }
    planner = MovingPlanner(
        IsaacMap.from_dict(data),
        ParkingConfig(parking_distance=0.8, keep_lateral=False),
    )
    planned = planner.plan_target(
        target_type='shelf',
        target_id='shelf_01',
        frame_id='map',
        position=POSITION,
        orientation=ORIENTATION,
    )
    assert planned.pose.position_x == pytest.approx(0.8)
    assert planned.pose.position_y == pytest.approx(0.0)


def test_planner_without_map_keeps_raw_pose():
    planner = MovingPlanner()
    planned = planner.plan_target(
        target_type='shelf',
        target_id='shelf_01',
        frame_id='map',
        position=POSITION,
        orientation=ORIENTATION,
    )
    assert planned.pose.position_x == pytest.approx(-3.0)
    assert planned.pose.position_y == pytest.approx(-4.0)
