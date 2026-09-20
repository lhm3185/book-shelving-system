"""Launch the shelving navigation action server.

This node bridges the shelving ``/navigate_to_target`` action used by the
task manager to the Nav2 ``navigate_to_pose`` action server driving the
Isaac Sim Nova Carter AMR.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("shelving_navigation")
    default_waypoints = os.path.join(
        package_share, "config", "waypoints.yaml"
    )

    action_name = LaunchConfiguration("action_name")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    waypoints_path = LaunchConfiguration("waypoints_path")
    frame_id = LaunchConfiguration("frame_id")
    nav2_server_timeout_sec = LaunchConfiguration(
        "nav2_server_timeout_sec"
    )

    navigation_node = Node(
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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "action_name",
            default_value="/navigate_to_target",
            description="Shelving navigation action server name.",
        ),
        DeclareLaunchArgument(
            "nav2_action_name",
            default_value="navigate_to_pose",
            description="Nav2 action server name for navigation.",
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
        navigation_node,
    ])