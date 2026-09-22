"""Launch Ridgeback-Franka Nav2 and the shelving navigation server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    default_waypoints = os.path.join(
        package_share,
        "config",
        "waypoints.yaml",
    )
    default_rviz = os.path.join(
        package_share,
        "rviz",
        "navigation.rviz",
    )

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    waypoints_file = LaunchConfiguration("waypoints_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    nav2_launch = os.path.join(
        nav2_share,
        "launch",
        "bringup_launch.py",
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
            "waypoints_file",
            default_value=default_waypoints,
            description="Shelving navigation waypoint file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Isaac Sim clock.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/lidar/points_raw",
            description="Ridgeback LiDAR PointCloud2 topic.",
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
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),

        Node(
            package="shelving_navigation",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
            parameters=[{
                "action_name": "/navigate_to_target",
                "nav2_action_name": "/navigate_to_pose",
                "waypoints_path": waypoints_file,
                "frame_id": "map",
                "nav2_server_timeout_sec": 10.0,
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_rviz),
        ),
    ])