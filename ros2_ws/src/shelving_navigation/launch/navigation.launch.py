"""Launch Ridgeback-Franka Nav2 and the shelving navigation server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the Ridgeback-Franka navigation launch description."""
    package_share = get_package_share_directory("shelving_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(
        package_share,
        "maps",
        "library_map.yaml",
    )
    default_params = os.path.join(
        package_share,
        "config",
        "nav2_ridgeback_franka.yaml",
    )
    default_lidar_params = os.path.join(
        package_share,
        "config",
        "pointcloud_to_laserscan.yaml",
    )
    default_navigation_params = os.path.join(
        package_share,
        "config",
        "navigation.yaml",
    )
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    navigation_params_file = LaunchConfiguration(
        "navigation_params_file"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    localization_launch = os.path.join(
        nav2_share,
        "launch",
        "localization_launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Occupancy map YAML file.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Ridgeback-Franka Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=default_lidar_params,
            description="PointCloud2 to LaserScan parameter file.",
        ),
        DeclareLaunchArgument(
            "navigation_params_file",
            default_value=default_navigation_params,
            description="Shelving navigation node parameter file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Isaac Sim clock.",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/lidar/points_raw",
            description="Ridgeback LiDAR PointCloud2 topic from Isaac Sim.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="Generated LaserScan topic.",
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[
                lidar_params_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("cloud_in", pointcloud_topic),
                ("scan", scan_topic),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "velocity_smoother",
                    "collision_monitor",
                    "bt_navigator",
                ],
            }],
        ),

        Node(
            package="shelving_navigation",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
            parameters=[
                navigation_params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
