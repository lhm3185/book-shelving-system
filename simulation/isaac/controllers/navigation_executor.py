"""AMR 주행 실행기 자리 (이동준 담당).

지금은 비어 있다. **인터페이스 경계만 먼저 나눠 둔다** — 나중에 로봇팔 실행기와 같은 파일에
기능을 계속 더하지 않기 위해서다.

붙일 때 지킬 것 (로봇팔 실행기와 같은 방식):
    - `run_simulation.py` 가 만든 rclpy 노드를 받아 자기 토픽만 만든다
    - 매 시뮬 스텝마다 `spin()` 이 불린다. 그 안에서 `world.step()` 을 부르는 쪽은 **하나뿐이어야** 한다
      (지금은 로봇팔 실행기가 부른다. 둘 다 붙이면 루프 쪽에서 한 번만 부르도록 옮길 것)
    - ROS 통신 규약은 FSM 과 합의된 것을 그대로 쓴다 (`/navigate_to_target` 액션)
"""


class NavigationExecutor:
    """AMR 주행 실행기 (미구현)"""

    def __init__(self, scene, node, say, **kwargs):
        raise NotImplementedError("AMR 주행 실행기는 아직 없다 (이동준 담당)")

    def spin(self):
        raise NotImplementedError
