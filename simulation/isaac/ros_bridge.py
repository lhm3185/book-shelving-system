"""Isaac Sim 의 ROS2 Bridge 공통 처리 (활성화·노드·정리).

실행기(manipulation / navigation) 가 공통으로 쓴다. 토픽 이름은 각 실행기가 정한다.
"""
import os


def enable_bridge(app, updates=10, say=print):
    """isaacsim.ros2.bridge 를 켠다. SimulationApp 을 만든 뒤에 호출해야 한다"""
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    for _ in range(updates):
        app.update()
    say(f"ROS2 Bridge 활성화 (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '기본값 0')}, "
        f"RMW={os.environ.get('RMW_IMPLEMENTATION', '기본값')})")


def make_node(name="isaac_simulation"):
    """Isaac 안에서 도는 rclpy 노드. 시스템 ROS 가 아니라 Isaac 내장 rclpy 를 쓴다"""
    import rclpy

    rclpy.init()
    return rclpy.create_node(name)


def shutdown(node):
    import rclpy

    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
