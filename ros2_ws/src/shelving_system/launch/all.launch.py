"""Launch the integrated ROS nodes for the shelving system."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    """Create the ROS nodes for the selected execution mode."""
    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    if mode != "navigation-test":
        raise RuntimeError(
            "Only 'navigation-test' is currently implemented. "
            "The full perception/manipulation mode is not ready."
        )

    navigation_share = get_package_share_directory(
        "shelving_navigation"
    )
    navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "navigation.launch.py",
    )

    return [
        # Nav2, AMCL, LiDAR 변환, navigation_node를 실행한다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        # TrayJob을 받아 전체 작업 순서를 관리한다.
        Node(
            package="shelving_system",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                }
            ],
        ),

        # 테스트 작업을 발행하는 가상 반납기 노드다.
        # 작업 발행은 run.sh가 service를 호출하게 만들 예정이므로
        # 여기서는 자동 발행하지 않는다.
        Node(
            package="shelving_system",
            executable="return_machine_node",
            name="return_machine_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "auto_publish": False,
                }
            ],
        ),

        # 현재 테스트에서는 실제 로봇팔 대신 성공 결과를 반환한다.
        Node(
            package="shelving_system",
            executable="mock_manipulation_server",
            name="mock_manipulation_server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
    ]


def generate_launch_description():
    """Build the integrated launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="navigation-test",
            description=(
                "Execution mode. Currently only "
                "'navigation-test' is available."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Isaac Sim clock.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
