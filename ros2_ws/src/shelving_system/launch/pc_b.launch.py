"""Launch the PC B onboard-computer nodes.

PC B owns:
- Navigation / Nav2
- Perception
- Manipulation
- Robot-related static TF aliases
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the PC B launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    navigation_share = get_package_share_directory(
        "shelving_navigation"
    )
    navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "navigation.launch.py",
    )

    perception_config = os.path.join(
        get_package_share_directory("shelving_perception"),
        "config",
        "perception.yaml",
    )

    manipulation_config = os.path.join(
        get_package_share_directory("shelving_manipulation"),
        "config",
        "manipulation.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the clock published by Isaac Sim on PC A.",
        ),

        # PointCloud 변환, AMCL, map server, Nav2와
        # shelving navigation action server를 실행한다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        # 기존 perception/manipulation 계약에서 사용하는
        # arm_base_link 별칭이다.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="arm_base_link_alias",
            output="screen",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "panda_link0",
                "--child-frame-id", "arm_base_link",
            ],
        ),

        # Isaac이 발행하는 wrist_camera 프레임과
        # perception이 사용하는 optical frame을 연결한다.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="wrist_camera_optical_frame",
            output="screen",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "wrist_camera",
                "--child-frame-id",
                "wrist_camera_optical_frame",
            ],
        ),

        # RGB-D 영상으로 책과 빈 슬롯을 검출한다.
        Node(
            package="shelving_perception",
            executable="vision_manager",
            name="vision_manager",
            output="screen",
            emulate_tty=True,
            parameters=[
                perception_config,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
        ),

        # PlaceBook action을 제공하고 perception/navigation 및
        # PC A의 Isaac manipulation executor와 통신한다.
        Node(
            package="shelving_manipulation",
            executable="manipulation_node",
            name="manipulation_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                manipulation_config,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
