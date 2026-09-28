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

    if mode not in ("navigation-test", "full"):
        raise RuntimeError(
            f"Unsupported mode: {mode}. "
            "Use 'navigation-test' or 'full'."
        )

    navigation_share = get_package_share_directory(
        "shelving_navigation"
    )
    navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "navigation.launch.py",
    )

    nodes = [
        # Nav2, AMCL, LiDAR 변환, navigation_node를 실행한다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        # Isaac의 표준 JSON 메시지를 프로젝트 인터페이스로 변환한다.
        Node(
            package="shelving_system",
            executable="simulation_bridge_node",
            name="simulation_bridge_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "raw_state_topic": (
                        "/simulation/scenario/state"
                    ),
                    "scenario_state_topic": (
                        "/scenario/state"
                    ),
                    # Timeline Stop/Play restores the Isaac articulation.
                    # Publish the matching pose so AMCL does not retain the
                    # previous run's map->odom estimate.
                    "initial_pose_x": -6.086313,
                    "initial_pose_y": 5.546779,
                    "initial_pose_yaw": 0.0,
                }
            ],
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

                    # position_tolerance 0.05m를 고려하여
                    # 실제 약 0.30m 후퇴하도록 0.35m를 요청한다.
                    "shelf_retreat_goal_distance_m": 0.65,
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
                    "publish_on_scenario_restart": True,
                    # 1초간 reset 0속도를 유지하고 AMCL을 다시 맞춘 뒤
                    # 새 작업을 발행한다.
                    "scenario_restart_publish_delay_sec": 2.0,

                    # 통합 시나리오에서는 동일한 shelf_01에
                    # 배치할 책 두 권을 발행한다.
                    "book_ids": [
                        "book_001",
                    ],
                    "rfid_tags": [
                        "rfid_001",
                    ],
                    "classification_codes": [
                        "005.7",
                    ],
                }
            ],
        ),

    ]

    if mode == "navigation-test":
        nodes.append(Node(
            package="shelving_system",
            executable="mock_manipulation_server",
            name="mock_manipulation_server",
            output="screen",
            emulate_tty=True,
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    else:
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
        nodes.extend([
            # 기존 perception 계약은 arm_base_link를 사용한다. Franka의
            # panda_link0와 같은 좌표축인 정적 별칭을 한 곳에서만 제공한다.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="arm_base_link_alias",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "panda_link0",
                    "--child-frame-id", "arm_base_link",
                ],
            ),
            # Isaac의 ROS2PublishTransformTree는 Camera prim의 TF를 이미
            # ROS optical 축(+X 오른쪽, +Y 아래, +Z 전방)으로 발행한다.
            # 이미지 frame_id 별칭만 연결하며 회전을 다시 적용하지 않는다.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="wrist_camera_optical_frame",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0",
                    "--pitch", "0", "--yaw", "0",
                    "--frame-id", "wrist_camera",
                    "--child-frame-id", "wrist_camera_optical_frame",
                ],
            ),
            Node(
                package="shelving_perception",
                executable="vision_manager",
                name="vision_manager",
                output="screen",
                emulate_tty=True,
                parameters=[perception_config, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="shelving_manipulation",
                executable="manipulation_node",
                name="manipulation_node",
                output="screen",
                emulate_tty=True,
                parameters=[manipulation_config, {"use_sim_time": use_sim_time}],
            ),
        ])

    return nodes


def generate_launch_description():
    """Build the integrated launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="navigation-test",
            description=(
                "Execution mode: 'navigation-test' uses a mock arm; "
                "'full' starts production perception and manipulation."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Isaac Sim clock.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
