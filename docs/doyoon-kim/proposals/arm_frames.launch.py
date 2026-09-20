"""
팀 계약 프레임을 Franka 프레임에 static TF 로 붙인다.

코드 안의 프레임 이름은 Franka 그대로(panda_link0, panda_hand, right_gripper) 쓰고,
팀 계약 이름(arm_base_link, tool0, gripper_tcp)은 TF 로만 연결한다.
팔이 또 바뀌어도 이 파일만 고치면 된다.

오프셋은 Isaac Sim 5.1.0 에서 실측했다 (2026-09-17, ~/arm/isaac/probe_frames.py):
  panda_hand → right_gripper : z +0.10 m, z축 180° 회전
  panda_link7 → panda_hand   : z +0.107 m (플랜지 위치)

전제: Isaac 쪽이 base_link → ... → panda_hand 까지의 TF 트리를 발행해야 연결된다.
      (2026-09-17 기준 레벨에 ROS2 그래프 없음)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def alias(parent, child, xyz=(0.0, 0.0, 0.0), qxyzw=(0.0, 0.0, 0.0, 1.0)):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'tf_{child}',
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--qx', str(qxyzw[0]), '--qy', str(qxyzw[1]),
            '--qz', str(qxyzw[2]), '--qw', str(qxyzw[3]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        # 팔 기준 좌표계. 비전의 TargetSlot 은 이 프레임으로 들어온다
        alias('panda_link0', 'arm_base_link'),
        # 플랜지
        alias('panda_hand', 'tool0'),
        # 손끝 중심. Lula 의 right_gripper 와 같은 점·자세
        alias('panda_hand', 'gripper_tcp', xyz=(0.0, 0.0, 0.10),
              qxyzw=(0.0, 0.0, 1.0, 0.0)),
    ])
