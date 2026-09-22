"""Isaac Sim 의 ROS2 Bridge 공통 처리 (활성화·노드·정리).

실행기(manipulation / navigation) 가 공통으로 쓴다. 토픽 이름은 각 실행기가 정한다.
"""
import os


def enable_bridge(app, updates=10, say=print):
    """Standalone 실행에 필요한 ROS2·차동구동 확장을 강제로 활성화한다."""
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app

    required_extensions = [
        "isaacsim.robot.wheeled_robots",
        "isaacsim.ros2.bridge",
    ]

    for extension_id in required_extensions:
        enable_extension(extension_id)

    # 확장과 DifferentialController 플러그인이 완전히 등록될 때까지 갱신
    for _ in range(updates):
        app.update()

    manager = omni.kit.app.get_app().get_extension_manager()

    for extension_id in required_extensions:
        if not manager.is_extension_enabled(extension_id):
            raise RuntimeError(
                f"필수 Isaac Sim 확장 활성화 실패: {extension_id}"
            )

    say(
        "Isaac Sim 필수 확장 활성화 완료: "
        + ", ".join(required_extensions)
        + f" (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '기본값 0')}, "
        + f"RMW={os.environ.get('RMW_IMPLEMENTATION', '기본값')})"
    )

def ensure_navigation_graph(stage, say=print):
    """Nova Carter 차동구동 Action Graph를 실행 가능한 상태로 강제 설정한다."""
    graph_path = "/World/Nova_Carter_ROS/differential_drive"
    graph = stage.GetPrimAtPath(graph_path)

    if not graph.IsValid():
        raise RuntimeError(
            f"주행 Action Graph를 찾을 수 없음: {graph_path}"
        )

    if not graph.IsActive():
        graph.SetActive(True)
        say(f"주행 Action Graph 강제 활성화: {graph_path}")

    evaluation_mode = graph.GetAttribute("evaluationMode")
    if evaluation_mode.IsValid():
        evaluation_mode.Set("Automatic")

    required_nodes = [
        "on_playback_tick",
        "ros2_subscribe_twist",
        "differential_controller_01",
        "articulation_controller_01",
    ]

    missing_nodes = []

    for node_name in required_nodes:
        node_path = f"{graph_path}/{node_name}"
        node = stage.GetPrimAtPath(node_path)

        if not node.IsValid():
            missing_nodes.append(node_path)
            continue

        if not node.IsActive():
            node.SetActive(True)
            say(f"주행 그래프 노드 강제 활성화: {node_path}")

    if missing_nodes:
        raise RuntimeError(
            "주행 Action Graph 필수 노드 없음:\n  "
            + "\n  ".join(missing_nodes)
        )

    say(f"주행 Action Graph 준비 완료: {graph_path}")


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
