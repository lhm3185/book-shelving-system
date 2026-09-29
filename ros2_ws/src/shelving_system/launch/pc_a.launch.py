"""Launch the PC A ROS nodes.

PC A owns:
- Isaac Sim bridge
- Task Manager / FSM
- Return machine simulator
- YAML job planning and data management

Isaac Sim itself is started separately with
scripts/isaac/run_standalone.sh.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the PC A launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the clock published by Isaac Sim.",
        ),

        # Isaac 내부의 raw scenario/tray 메시지를
        # 프로젝트 ROS 인터페이스로 변환한다.
        Node(
            package="shelving_system",
            executable="simulation_bridge_node",
            name="simulation_bridge_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "raw_state_topic": "/simulation/scenario/state",
                    "scenario_state_topic": "/scenario/state",

                    # 현재 integration/new_level의 로봇 초기 위치다.
                    "initial_pose_x": -5.591048,
                    "initial_pose_y": -3.852260,
                    "initial_pose_yaw": 0.0,
                }
            ],
        ),

        # 전체 작업 순서와 FSM을 관리한다.
        Node(
            package="shelving_system",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "shelf_retreat_goal_distance_m": 0.65,
                }
            ],
        ),

        # 테스트용 무인반납기다.
        # 자동 발행하지 않고 service 호출로 작업을 시작한다.
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
                    "publish_on_scenario_restart": False,

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

        # A/B 전체 준비 상태를 계산하고
        # 관제 PC의 작업 시작 요청을 처리한다.
        Node(
            package="shelving_system",
            executable="system_supervisor_node",
            name="system_supervisor_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    # Isaac Stop 상태에서도 heartbeat와 timeout이
                    # 계속 동작하도록 시스템 시간을 사용한다.
                    "use_sim_time": False,
                }
            ],
        ),
    ])
