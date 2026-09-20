"""
Bridges shelving navigation goals to Nav2.

The :class:`NavigationNode` implements the ``/navigate_to_target`` action
server that the shelving task manager calls. Each ``NavigateToTarget`` goal
is translated into a Nav2 ``NavigateToPose`` goal and forwarded to the Nav2
action server driving the Isaac Sim Nova Carter AMR. Feedback and result are
mapped back to the shelving action types.
"""

import time
import math
import threading
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from shelving_interfaces.action import NavigateToTarget


class WaypointStore:
    """Load and look up fixed navigation coordinates."""

    def __init__(self, waypoints_path: str) -> None:
        """Load the waypoint YAML file into an internal lookup table."""
        self._waypoints: dict[str, dict] = {}
        self.frame_id = "map"
        self._path = str(waypoints_path)

        path = Path(self._path)
        if not path.exists():
            raise FileNotFoundError(
                f"Waypoint file does not exist: {self._path}"
            )

        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        if isinstance(data, dict):
            self.frame_id = str(data.get("frame_id", "map"))
            waypoints = data.get("waypoints", {})
            if isinstance(waypoints, dict):
                self._waypoints = waypoints

    def lookup(self, key: str) -> dict | None:
        """Return the waypoint dict for a key, or None when missing."""
        if not key:
            return None
        return self._waypoints.get(key)

    def has_waypoint(self, key: str) -> bool:
        """Return True when a fixed waypoint exists for a key."""
        return key in self._waypoints


class NavigationNode(Node):
    """Expose the shelving navigation action and forward goals to Nav2."""

    ERROR_NAVIGATION_SERVER = 2001
    ERROR_NAVIGATION_REJECTED = 2002
    ERROR_NAVIGATION_FAILED = 2003
    ERROR_NAVIGATION_RESULT = 2004

    def __init__(self) -> None:
        """Initialize the navigation action server and Nav2 client."""
        super().__init__("navigation_node")

        default_waypoints = str(
            Path(get_package_share_directory("shelving_navigation"))
            / "config"
            / "waypoints.yaml"
        )

        self.declare_parameter("action_name", "/navigate_to_target")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("waypoints_path", default_waypoints)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("nav2_server_timeout_sec", 10.0)

        self._action_name = str(
            self.get_parameter("action_name").value
        )
        nav2_action_name = str(
            self.get_parameter("nav2_action_name").value
        )
        self._frame_id = str(
            self.get_parameter("frame_id").value
        )
        self._nav2_server_timeout_sec = float(
            self.get_parameter("nav2_server_timeout_sec").value
        )
        self._nav2_action_name = nav2_action_name

        self._callback_group = ReentrantCallbackGroup()

        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            nav2_action_name,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            NavigateToTarget,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._active_goal_handle = None
        self._active_nav2_goal_handle = None
        self._latest_nav2_feedback = None
        self._active_target_pose = None
        self._feedback_lock = threading.Lock()

        self._waypoints = WaypointStore(
            str(self.get_parameter("waypoints_path").value)
        )

        self.get_logger().info(
            "Navigation node is ready. "
            f"Action name: '{self._action_name}', "
            f"Nav2 action name: '{nav2_action_name}', "
            f"frame_id: '{self._frame_id}', "
            f"waypoints: '{self.get_parameter('waypoints_path').value}'."
        )

    def _goal_callback(
        self,
        goal_request: NavigateToTarget.Goal,
    ) -> GoalResponse:
        """Accept navigation goals with valid identifiers."""
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

    async def _execute_callback(
        self,
        goal_handle,
    ) -> NavigateToTarget.Result:
        """Execute one navigation goal through Nav2."""
        request = goal_handle.request

        self._active_goal_handle = goal_handle
        self._latest_nav2_feedback = None

        try:
            self.get_logger().info(
                "Navigating: "
                f"job_id={request.job_id}, "
                f"target_type={request.target_type}, "
                f"target_id={request.target_id}"
            )

            if not self._nav2_client.wait_for_server(
                timeout_sec=self._nav2_server_timeout_sec
            ):
                goal_handle.abort()
                return self._make_result(
                    success=False,
                    error_code=self.ERROR_NAVIGATION_SERVER,
                    message=(
                        "Nav2 action server is not available."
                    ),
                )

            pose = self._resolve_target_pose(request)
            if pose is None:
                goal_handle.abort()
                return self._make_result(
                    success=False,
                    error_code=self.ERROR_NAVIGATION_RESULT,
                    message=(
                        "Could not resolve a target pose for "
                        f"'{request.target_id}'."
                    ),
                )

            self._active_target_pose = pose

            nav2_goal = NavigateToPose.Goal()
            nav2_goal.pose = pose

            send_goal_future = self._nav2_client.send_goal_async(
                nav2_goal,
                feedback_callback=self._handle_nav2_feedback,
            )

            nav2_goal_handle = await send_goal_future

            if nav2_goal_handle is None or not nav2_goal_handle.accepted:
                goal_handle.abort()
                return self._make_result(
                    success=False,
                    error_code=self.ERROR_NAVIGATION_REJECTED,
                    message="Nav2 rejected the navigation goal.",
                )

            self._active_nav2_goal_handle = nav2_goal_handle

            result_future = nav2_goal_handle.get_result_async()

            while rclpy.ok() and not result_future.done():
                if goal_handle.is_cancel_requested:
                    cancel_future = self._nav2_client.cancel_goal_async(
                        nav2_goal_handle
                    )
                    await cancel_future
                    goal_handle.canceled()
                    return self._make_result(
                        success=False,
                        error_code=self.ERROR_NAVIGATION_REJECTED,
                        message="Navigation was canceled.",
                    )
                time.sleep(0.1)

            response = result_future.result()
            nav2_result = response.result

            final_pose = self._take_latest_pose(pose)

            if response.status != GoalStatus.STATUS_SUCCEEDED:
                goal_handle.abort()
                return self._make_result(
                    success=False,
                    final_pose=final_pose,
                    error_code=self.ERROR_NAVIGATION_FAILED,
                    message=(
                        nav2_result.error_msg
                        or "Nav2 navigation failed."
                    ),
                )

            tolerance_satisfied = self._check_tolerance(
                request,
                final_pose,
            )

            goal_handle.succeed()
            return self._make_result(
                success=True,
                final_pose=final_pose,
                tolerance_satisfied=tolerance_satisfied,
                error_code=0,
                message=(
                    f"Reached target '{request.target_id}'."
                ),
            )

        finally:
            self._active_goal_handle = None
            self._active_nav2_goal_handle = None
            self._latest_nav2_feedback = None
            self._active_target_pose = None

    def _handle_nav2_feedback(
        self,
        feedback_message,
    ) -> None:
        """Convert Nav2 feedback into shelving navigation feedback."""
        nav2_feedback = feedback_message.feedback

        with self._feedback_lock:
            self._latest_nav2_feedback = nav2_feedback

        active_goal = self._active_goal_handle
        if active_goal is None:
            return

        feedback = NavigateToTarget.Feedback()
        feedback.phase = "NAVIGATING"
        feedback.remaining_distance = float(
            nav2_feedback.distance_remaining
        )

        position_error, yaw_error = self._compute_errors(
            self._active_target_pose,
            nav2_feedback.current_pose,
        )
        feedback.position_error = float(position_error)
        feedback.yaw_error = float(yaw_error)
        feedback.retry_count = int(
            nav2_feedback.number_of_recoveries
        )

        active_goal.publish_feedback(feedback)

    def _resolve_target_pose(
        self,
        request: NavigateToTarget.Goal,
    ) -> PoseStamped | None:
        """
        Resolve the pose to navigate to.

        A valid pose attached to the action goal takes priority.
        The local waypoint file is used only as a fallback when
        the action goal does not contain a frame_id.
        """
        target_pose = request.target_pose

        if target_pose.header.frame_id:
            return target_pose

        for key in (request.target_id, request.target_type):
            waypoint = self._waypoints.lookup(key)

            if waypoint is None:
                continue

            position = waypoint.get("position", {})
            orientation = waypoint.get("orientation", {})

            pose = PoseStamped()
            pose.header.frame_id = str(
                waypoint.get("frame_id", self._frame_id)
            )
            pose.pose.position.x = float(
                position.get("x", 0.0)
            )
            pose.pose.position.y = float(
                position.get("y", 0.0)
            )
            pose.pose.position.z = float(
                position.get("z", 0.0)
            )
            pose.pose.orientation.x = float(
                orientation.get("x", 0.0)
            )
            pose.pose.orientation.y = float(
                orientation.get("y", 0.0)
            )
            pose.pose.orientation.z = float(
                orientation.get("z", 0.0)
            )
            pose.pose.orientation.w = float(
                orientation.get("w", 1.0)
            )

            return pose

        return None

    def _take_latest_pose(
        self,
        fallback_pose: PoseStamped,
    ) -> PoseStamped:
        """Return the last reported robot pose or a fallback pose."""
        with self._feedback_lock:
            nav2_feedback = self._latest_nav2_feedback

        if nav2_feedback is not None:
            return nav2_feedback.current_pose

        return fallback_pose

    def _check_tolerance(
        self,
        request: NavigateToTarget.Goal,
        final_pose: PoseStamped,
    ) -> bool:
        """Compare the final pose with the requested tolerances."""
        position_error, yaw_error = self._compute_errors(
            self._active_target_pose,
            final_pose,
        )
        return bool(
            position_error <= request.position_tolerance
            and yaw_error <= request.yaw_tolerance
        )

    def _compute_errors(
        self,
        target_pose: PoseStamped,
        current_pose: PoseStamped,
    ) -> tuple[float, float]:
        """Return the euclidean and yaw error between two poses."""
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        position_error = math.hypot(dx, dy)

        target_yaw = self._quaternion_to_yaw(
            target_pose.pose.orientation
        )
        current_yaw = self._quaternion_to_yaw(
            current_pose.pose.orientation
        )
        yaw_error = abs(self._normalize_angle(current_yaw - target_yaw))

        return position_error, yaw_error

    @staticmethod
    def _quaternion_to_yaw(quaternion) -> float:
        """Convert a quaternion into a yaw angle in radians."""
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y ** 2 + quaternion.z ** 2
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap an angle into the [-pi, pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _make_result(
        self,
        *,
        success: bool,
        final_pose: PoseStamped | None = None,
        tolerance_satisfied: bool = False,
        error_code: int,
        message: str,
    ) -> NavigateToTarget.Result:
        """Build a NavigateToTarget result message."""
        result = NavigateToTarget.Result()
        result.success = success
        result.tolerance_satisfied = tolerance_satisfied
        result.error_code = error_code
        result.message = message

        if final_pose is not None:
            result.final_pose = final_pose

        return result

    def destroy_node(self) -> None:
        """Destroy the action server and Nav2 client."""
        self._action_server.destroy()
        self._nav2_client.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the navigation node."""
    rclpy.init(args=args)

    node: NavigationNode | None = None

    try:
        node = NavigationNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()