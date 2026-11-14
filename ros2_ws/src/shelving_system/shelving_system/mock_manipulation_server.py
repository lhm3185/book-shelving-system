"""Mock book-placement action server."""

import time

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.node import Node

from shelving_interfaces.action import PlaceBook


class MockManipulationServer(Node):
    """Simulate the complete internal book-placement workflow."""

    def __init__(self) -> None:
        """Initialize the mock manipulation server."""
        super().__init__("mock_manipulation_server")

        self.declare_parameter(
            "action_name",
            "/place_book",
        )
        self.declare_parameter(
            "step_delay_sec",
            0.3,
        )

        action_name = str(
            self.get_parameter("action_name").value
        )

        self._action_server = ActionServer(
            self,
            PlaceBook,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            "Mock manipulation server is ready. "
            f"Action name: '{action_name}'."
        )

    def _goal_callback(
        self,
        _goal_request: PlaceBook.Goal,
    ) -> GoalResponse:
        """Accept an empty book-placement command."""
        self.get_logger().info(
            "Accepted book-placement command."
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(
        self,
        _goal_handle,
    ) -> CancelResponse:
        """Accept a placement cancellation request."""
        self.get_logger().info(
            "Book-placement cancellation requested."
        )
        return CancelResponse.ACCEPT

    def _execute_callback(
        self,
        goal_handle,
    ) -> PlaceBook.Result:
        """Simulate perception, grasping, insertion, and verification."""
        step_delay = float(
            self.get_parameter("step_delay_sec").value
        )

        phases = [
            "MOVING_TO_TRAY_VIEW",
            "DETECTING_TRAY_BOOK",
            "MOVING_TO_SHELF_VIEW",
            "DETECTING_EMPTY_SLOT",
            "GRASPING_BOOK",
            "PLANNING_INSERTION",
            "MOVING_TO_PRE_INSERT",
            "INSERTING_BOOK",
            "RELEASING_BOOK",
            "RETREATING",
            "VERIFYING_PLACEMENT",
        ]

        for phase in phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = PlaceBook.Result()
                result.success = False
                result.error_code = 4003
                result.message = (
                    "Book placement was canceled."
                )

                return result

            feedback = PlaceBook.Feedback()
            feedback.phase = phase
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f"Placement feedback: phase={phase}"
            )

            time.sleep(step_delay)

        goal_handle.succeed()

        result = PlaceBook.Result()
        result.success = True
        result.error_code = 0
        result.message = (
            "Mock book placement completed successfully."
        )

        self.get_logger().info(
            "Mock book placement completed."
        )

        return result

    def destroy_node(self) -> None:
        """Destroy the action server and node."""
        self._action_server.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the mock manipulation action server."""
    rclpy.init(args=args)

    node: MockManipulationServer | None = None

    try:
        node = MockManipulationServer()
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