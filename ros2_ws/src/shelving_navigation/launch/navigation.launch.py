# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("shelving_navigation")
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    default_map = os.path.join(
    package_share_dir,
    "maps",
    "library_map.yaml",
    )

    default_params = os.path.join(
        package_share_dir,
        "config",
        "nav2.yaml",
    )

    rviz_config_dir = os.path.join(
        package_share_dir,
        "rviz",
        "navigation.rviz",
    )

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=default_map, description="Full path to map file to load"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Omniverse Isaac Sim) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={
                    "namespace": "",
                    "use_namespace": "false",
                    "rviz_config": rviz_config_dir,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),

            # This USD does not publish /clock. Derive it from the Isaac Sim
            # PointCloud timestamp so nodes using use_sim_time can advance.
            # The helper automatically yields if a real /clock publisher appears.
            Node(
                package="shelving_navigation",
                executable="sensor_stamp_to_clock",
                name="sensor_stamp_to_clock",
                condition=IfCondition(use_sim_time),
                parameters=[{
                    "source_topic": "/front_3d_lidar/lidar_points",
                    "clock_topic": "/clock",
                    "use_sim_time": use_sim_time,
                }],
            ),

            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/front_3d_lidar/lidar_points']),
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'front_3d_lidar',
                    'transform_tolerance': 0.01,
                    'min_height': -0.05,
                    'max_height': 0.05,
                    'angle_min': -1.5708,  # -M_PI
                    'angle_max': 1.5708,  # M_PI
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.1,
                    'range_min': 0.05,
                    'range_max': 20.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # 'concurrency_level': 1,
                    'use_sim_time': use_sim_time,
                }],
                name='pointcloud_to_laserscan'
            )
        ]
    )
