"""Mock navigation action server for integration testing."""

import time

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.node import Node

from shelving_interfaces.action import NavigateToTarget


class MockNavigationServer(Node):
    """Simulate successful AMR navigation."""

    def __init__(self) -> None:
        """Initialize the mock navigation action server."""
        super().__init__("mock_navigation_server")

        self.declare_parameter(
            "action_name",
            "/navigate_to_target",
        )
        self.declare_parameter(
            "step_delay_sec",
            0.5,
        )

        action_name = str(
            self.get_parameter("action_name").value
        )

        self._action_server = ActionServer(
            self,
            NavigateToTarget,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            "Mock navigation server is ready. "
            f"Action name: '{action_name}'."
        )

    def _goal_callback(
        self,
        goal_request: NavigateToTarget.Goal,
    ) -> GoalResponse:
        """Accept valid navigation goals."""
        if not goal_request.job_id.strip():
            self.get_logger().warning(
                "Rejected navigation goal: empty job_id."
            )
            return GoalResponse.REJECT

        if not goal_request.target_id.strip():
            self.get_logger().warning(
                "Rejected navigation goal: empty target_id."
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            "Accepted navigation goal: "
            f"job_id={goal_request.job_id}, "
            f"target_type={goal_request.target_type}, "
            f"target_id={goal_request.target_id}"
        )

        return GoalResponse.ACCEPT

    def _cancel_callback(
        self,
        goal_handle,
    ) -> CancelResponse:
        """Accept navigation cancellation requests."""
        self.get_logger().info(
            "Navigation cancellation requested."
        )
        return CancelResponse.ACCEPT

    def _execute_callback(
        self,
        goal_handle,
    ) -> NavigateToTarget.Result:
        """Simulate navigation feedback and success."""
        request = goal_handle.request
        step_delay = float(
            self.get_parameter("step_delay_sec").value
        )

        remaining_distances = [
            1.0,
            0.5,
            0.1,
            0.0,
        ]

        for step_number, distance in enumerate(
            remaining_distances,
            start=1,
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = NavigateToTarget.Result()
                result.success = False
                result.tolerance_satisfied = False
                result.error_code = 2002
                result.message = "Navigation was canceled."

                return result

            feedback = NavigateToTarget.Feedback()
            feedback.phase = "NAVIGATING"
            feedback.remaining_distance = distance
            feedback.position_error = distance
            feedback.yaw_error = 0.0
            feedback.retry_count = 0

            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                "Navigation feedback: "
                f"step={step_number}, "
                f"remaining_distance={distance:.2f}"
            )

            time.sleep(step_delay)

        goal_handle.succeed()

        result = NavigateToTarget.Result()
        result.success = True
        result.final_pose = request.target_pose
        result.tolerance_satisfied = True
        result.error_code = 0
        result.message = (
            f"Reached target '{request.target_id}'."
        )

        self.get_logger().info(
            "Mock navigation completed: "
            f"job_id={request.job_id}, "
            f"target_id={request.target_id}"
        )

        return result

    def destroy_node(self) -> None:
        """Destroy the action server and node."""
        self._action_server.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the mock navigation action server."""
    rclpy.init(args=args)

    node: MockNavigationServer | None = None

    try:
        node = MockNavigationServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()