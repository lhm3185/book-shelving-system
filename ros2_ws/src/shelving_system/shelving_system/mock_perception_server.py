"""Mock empty-slot detection action server."""

import time

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.node import Node

from shelving_interfaces.action import DetectTargetSlot


class MockPerceptionServer(Node):
    """Simulate empty shelf-slot detection."""

    def __init__(self) -> None:
        """Initialize the mock perception action server."""
        super().__init__("mock_perception_server")

        self.declare_parameter(
            "action_name",
            "/detect_target_slot",
        )
        self.declare_parameter(
            "target_frame",
            "arm_base_link",
        )
        self.declare_parameter(
            "step_delay_sec",
            0.4,
        )

        action_name = str(
            self.get_parameter("action_name").value
        )

        self._action_server = ActionServer(
            self,
            DetectTargetSlot,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            "Mock perception server is ready. "
            f"Action name: '{action_name}'."
        )

    def _goal_callback(
        self,
        goal_request: DetectTargetSlot.Goal,
    ) -> GoalResponse:
        """Validate and accept an empty-slot detection goal."""
        required_identifiers = {
            "job_id": goal_request.job_id,
            "book_id": goal_request.book_id,
            "shelf_id": goal_request.shelf_id,
        }

        for field_name, value in required_identifiers.items():
            if not value.strip():
                self.get_logger().warning(
                    "Rejected perception goal: "
                    f"{field_name} is empty."
                )
                return GoalResponse.REJECT

        dimensions = {
            "book_width": goal_request.book_width,
            "book_height": goal_request.book_height,
            "book_thickness": goal_request.book_thickness,
            "safety_margin": goal_request.safety_margin,
        }

        for field_name, value in dimensions.items():
            if value <= 0.0:
                self.get_logger().warning(
                    "Rejected perception goal: "
                    f"{field_name} must be positive."
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            "Accepted perception goal: "
            f"job_id={goal_request.job_id}, "
            f"book_id={goal_request.book_id}, "
            f"shelf_id={goal_request.shelf_id}"
        )

        return GoalResponse.ACCEPT

    def _cancel_callback(
        self,
        goal_handle,
    ) -> CancelResponse:
        """Accept a slot-detection cancellation request."""
        self.get_logger().info(
            "Slot detection cancellation requested."
        )
        return CancelResponse.ACCEPT

    def _execute_callback(
        self,
        goal_handle,
    ) -> DetectTargetSlot.Result:
        """Simulate slot-detection feedback and result."""
        request = goal_handle.request
        step_delay = float(
            self.get_parameter("step_delay_sec").value
        )

        phases = [
            ("CAPTURING", 0, 0.0),
            ("DETECTING_SHELF", 1, 0.70),
            ("FINDING_EMPTY_SPACE", 2, 0.85),
            ("CALCULATING_POSE", 2, 0.93),
            ("TRANSFORMING_FRAME", 2, 0.95),
        ]

        for phase, candidate_count, confidence in phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = DetectTargetSlot.Result()
                result.success = False
                result.candidate_count = 0
                result.error_code = 3002
                result.message = (
                    "Slot detection was canceled."
                )

                return result

            feedback = DetectTargetSlot.Feedback()
            feedback.phase = phase
            feedback.candidate_count = candidate_count
            feedback.best_confidence = confidence

            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                "Perception feedback: "
                f"phase={phase}, "
                f"candidate_count={candidate_count}, "
                f"best_confidence={confidence:.2f}"
            )

            time.sleep(step_delay)

        goal_handle.succeed()

        result = DetectTargetSlot.Result()
        result.success = True
        result.candidate_count = 2
        result.error_code = 0
        result.message = (
            f"Detected an empty slot on "
            f"'{request.shelf_id}'."
        )

        target_slot = result.target_slot
        target_slot.header.stamp = (
            self.get_clock().now().to_msg()
        )
        target_slot.header.frame_id = str(
            self.get_parameter("target_frame").value
        )

        # 로봇팔이 검증한 1차 서가 칸 (arm_base_link 기준, 꽂힌 책 AABB 중심)
        target_slot.pose.position.x = -0.3497
        target_slot.pose.position.y = 0.5495
        target_slot.pose.position.z = 0.3399

        # 삽입 방향 yaw +90° (arm_base_link +Y). 단위 쿼터니언은 yaw 0° 라 로봇팔이 M410 으로 거절한다
        target_slot.pose.orientation.x = 0.0
        target_slot.pose.orientation.y = 0.0
        target_slot.pose.orientation.z = 0.7071068
        target_slot.pose.orientation.w = 0.7071068

        target_slot.available_width = max(
            0.08,
            request.book_thickness
            + 2.0 * request.safety_margin,
        )
        target_slot.available_height = max(
            0.30,
            request.book_height
            + request.safety_margin,
        )
        target_slot.insertion_depth = 0.25
        target_slot.pre_insert_offset = 0.05
        target_slot.confidence = 0.95

        self.get_logger().info(
            "Mock slot detection completed: "
            f"job_id={request.job_id}, "
            f"book_id={request.book_id}, "
            f"shelf_id={request.shelf_id}, "
            f"confidence={target_slot.confidence:.2f}"
        )

        return result

    def destroy_node(self) -> None:
        """Destroy the action server and node."""
        self._action_server.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the mock perception action server."""
    rclpy.init(args=args)

    node: MockPerceptionServer | None = None

    try:
        node = MockPerceptionServer()
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