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
    """Simulate grasping and placing a book."""

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
        goal_request: PlaceBook.Goal,
    ) -> GoalResponse:
        """Validate and accept a book-placement goal."""
        if not goal_request.job_id.strip():
            self.get_logger().warning(
                "Rejected placement goal: "
                "job_id is empty."
            )
            return GoalResponse.REJECT

        if not goal_request.book_id.strip():
            self.get_logger().warning(
                "Rejected placement goal: "
                "book_id is empty."
            )
            return GoalResponse.REJECT

        if not goal_request.target_slot.header.frame_id:
            self.get_logger().warning(
                "Rejected placement goal: "
                "target-slot frame_id is empty."
            )
            return GoalResponse.REJECT

        dimensions = {
            "book_width": goal_request.book_width,
            "book_height": goal_request.book_height,
            "book_thickness": goal_request.book_thickness,
            "insertion_speed": goal_request.insertion_speed,
        }

        for field_name, value in dimensions.items():
            if value <= 0.0:
                self.get_logger().warning(
                    "Rejected placement goal: "
                    f"{field_name} must be positive."
                )
                return GoalResponse.REJECT

        if goal_request.target_slot.confidence <= 0.0:
            self.get_logger().warning(
                "Rejected placement goal: "
                "target-slot confidence must be positive."
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            "Accepted placement goal: "
            f"job_id={goal_request.job_id}, "
            f"book_id={goal_request.book_id}, "
            f"target_frame="
            f"{goal_request.target_slot.header.frame_id}"
        )

        return GoalResponse.ACCEPT

    def _cancel_callback(
        self,
        goal_handle,
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
        """Simulate book placement feedback and success."""
        request = goal_handle.request
        step_delay = float(
            self.get_parameter("step_delay_sec").value
        )

        phases = [
            ("DETECTING_BOOK", 0.10),
            ("PLANNING_GRASP", 0.20),
            ("APPROACHING_BOOK", 0.35),
            ("GRASPING", 0.45),
            ("MOVING_TO_PRE_INSERT", 0.60),
            ("INSERTING", 0.75),
            ("RELEASING", 0.85),
            ("RETREATING", 0.93),
            ("VERIFYING", 1.00),
        ]

        current_phase = ""

        for phase, progress in phases:
            current_phase = phase

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = PlaceBook.Result()
                result.success = False
                result.failed_phase = current_phase
                result.placement_verified = False
                result.error_code = 4002
                result.message = (
                    "Book placement was canceled."
                )

                return result

            feedback = PlaceBook.Feedback()
            feedback.phase = phase
            feedback.progress = progress

            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                "Placement feedback: "
                f"phase={phase}, "
                f"progress={progress:.2f}"
            )

            time.sleep(step_delay)

        goal_handle.succeed()

        result = PlaceBook.Result()
        result.success = True
        result.failed_phase = ""
        result.placement_verified = True
        result.error_code = 0
        result.message = (
            f"Book '{request.book_id}' was placed "
            "successfully."
        )

        self.get_logger().info(
            "Mock book placement completed: "
            f"job_id={request.job_id}, "
            f"book_id={request.book_id}"
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