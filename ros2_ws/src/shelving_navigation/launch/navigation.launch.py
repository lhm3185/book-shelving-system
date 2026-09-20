"""Launch Nav2 and the shelving navigation action server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the complete navigation launch description."""
    package_share = get_package_share_directory("shelving_navigation")
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    default_map = os.path.join(package_share, "maps", "library_map.yaml")
    default_params = os.path.join(package_share, "config", "nav2.yaml")
    default_rviz = os.path.join(package_share, "rviz", "navigation.rviz")
    default_waypoints = os.path.join(
        package_share, "config", "waypoints.yaml"
    )

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    action_name = LaunchConfiguration("action_name")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    waypoints_path = LaunchConfiguration("waypoints_path")
    frame_id = LaunchConfiguration("frame_id")
    nav2_server_timeout_sec = LaunchConfiguration(
        "nav2_server_timeout_sec"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map file.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Isaac Sim clock when true.",
        ),
        DeclareLaunchArgument(
            "action_name",
            default_value="/navigate_to_target",
            description="Shelving navigation action server name.",
        ),
        DeclareLaunchArgument(
            "nav2_action_name",
            default_value="navigate_to_pose",
            description="Nav2 action server name.",
        ),
        DeclareLaunchArgument(
            "waypoints_path",
            default_value=default_waypoints,
            description="Path to the fixed waypoint YAML file.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Frame of the navigation targets.",
        ),
        DeclareLaunchArgument(
            "nav2_server_timeout_sec",
            default_value="10.0",
            description="Timeout for connecting to Nav2.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, "rviz_launch.py")
            ),
            launch_arguments={
                "namespace": "",
                "use_namespace": "false",
                "rviz_config": default_rviz,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
            }.items(),
        ),
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
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[
                ("cloud_in", "/front_3d_lidar/lidar_points"),
                ("scan", "/scan"),
            ],
            parameters=[{
                "target_frame": "front_3d_lidar",
                "transform_tolerance": 0.01,
                "min_height": -0.05,
                "max_height": 0.05,
                "angle_min": -1.5708,
                "angle_max": 1.5708,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.05,
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="shelving_navigation",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
            parameters=[{
                "action_name": action_name,
                "nav2_action_name": nav2_action_name,
                "waypoints_path": waypoints_path,
                "frame_id": frame_id,
                "nav2_server_timeout_sec": nav2_server_timeout_sec,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
