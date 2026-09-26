"""
Navigation execution logic used by NavigationNode.

This class is not a ROS node. It uses the NavigationNode passed to it
to create Nav2 action clients, a velocity publisher, and a TF listener.
"""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from shelving_interfaces.action import NavigateToTarget
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationController:
    """Execute direct and waypoint navigation through Nav2."""

    ERROR_NAVIGATION_SERVER = 2001
    ERROR_NAVIGATION_REJECTED = 2002
    ERROR_NAVIGATION_FAILED = 2003
    ERROR_NAVIGATION_RESULT = 2004
    ERROR_TRANSFORM = 2005
    ERROR_FINE_ALIGNMENT = 2006
    ERROR_BUSY = 2007

    FINE_ALIGNMENT_TIMEOUT_SEC = 60.0
    CONTROL_PERIOD_SEC = 0.1

    POSITION_GAIN = 0.8
    YAW_GAIN = 1.2

    def __init__(
        self,
        *,
        node,
        callback_group,
        nav2_action_name: str,
        nav2_route_action_name: str,
        nav2_server_timeout_sec: float,
        max_nav2_retries: int,
        nav2_retry_delay_sec: float,
        approach_radius: float,
        position_tolerance: float,
        yaw_tolerance: float,
        fine_alignment_linear_speed: float,
        fine_alignment_angular_speed: float,
        cmd_vel_topic: str,
        global_frame: str,
        robot_base_frame: str,
    ) -> None:
        """Create Nav2 clients, TF listener, and velocity publisher."""

        self._node = node

        self._nav2_server_timeout_sec = nav2_server_timeout_sec
        self._max_nav2_retries = max(0, max_nav2_retries)
        self._nav2_retry_delay_sec = max(
            0.0,
            nav2_retry_delay_sec,
        )
        self._approach_radius = approach_radius
        self._position_tolerance = position_tolerance
        self._yaw_tolerance = yaw_tolerance

        self._fine_alignment_linear_speed = (
            fine_alignment_linear_speed
        )
        self._fine_alignment_angular_speed = (
            fine_alignment_angular_speed
        )

        self._global_frame = global_frame
        self._robot_base_frame = robot_base_frame

        self._navigate_to_pose_client = ActionClient(
            node,
            NavigateToPose,
            nav2_action_name,
            callback_group=callback_group,
        )

        self._navigate_through_poses_client = ActionClient(
            node,
            NavigateThroughPoses,
            nav2_route_action_name,
            callback_group=callback_group,
        )

        self._cmd_vel_publisher = node.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )

        self._state_lock = threading.Lock()
        self._feedback_lock = threading.Lock()

        self._busy = False

        self._active_goal_handle = None
        self._active_nav2_goal_handle = None
        self._active_target_pose = None

        self._latest_pose = None
        self._remaining_distance = 0.0
        self._remaining_poses = 0
        self._number_of_recoveries = 0

        self._total_poses = 0
        self._route_mode = False

    @property
    def is_busy(self) -> bool:
        """Return whether a navigation request is currently executing."""
        with self._state_lock:
            return self._busy

    async def execute(
        self,
        goal_handle,
    ) -> NavigateToTarget.Result:
        """Execute one NavigateToTarget request."""

        with self._state_lock:
            if self._busy:
                goal_handle.abort()
                return self._make_result(
                    success=False,
                    error_code=self.ERROR_BUSY,
                    message=(
                        "Another navigation request is already active."
                    ),
                )

            self._busy = True

        request = goal_handle.request

        self._active_goal_handle = goal_handle
        self._active_target_pose = request.target_pose
        self._latest_pose = None
        self._number_of_recoveries = 0

        self._total_poses = len(request.waypoints) + 1
        self._remaining_poses = self._total_poses
        self._route_mode = bool(request.waypoints)

        try:
            nav2_client, nav2_goal = self._make_nav2_goal(request)

            if not nav2_client.wait_for_server(
                timeout_sec=self._nav2_server_timeout_sec
            ):
                goal_handle.abort()

                return self._make_result(
                    success=False,
                    error_code=self.ERROR_NAVIGATION_SERVER,
                    message=(
                        "The selected Nav2 action server "
                        "is not available."
                    ),
                )

            self._node.get_logger().info(
                "Sending navigation goal to Nav2: "
                f"mode={'route' if self._route_mode else 'direct'}, "
                f"poses={self._total_poses}, "
                "fine_alignment="
                f"{request.enable_fine_alignment}"
            )

            send_goal_future = nav2_client.send_goal_async(
                nav2_goal,
                feedback_callback=self._handle_nav2_feedback,
            )

            nav2_goal_handle = await send_goal_future

            if (
                nav2_goal_handle is None
                or not nav2_goal_handle.accepted
            ):
                goal_handle.abort()

                return self._make_result(
                    success=False,
                    error_code=self.ERROR_NAVIGATION_REJECTED,
                    message="Nav2 rejected the navigation goal.",
                )

            self._active_nav2_goal_handle = nav2_goal_handle

            result_future = nav2_goal_handle.get_result_async()
            early_stop_requested = False

            while rclpy.ok() and not result_future.done():
                if goal_handle.is_cancel_requested:
                    await self._cancel_nav2_goal()
                    self._publish_stop()

                    goal_handle.canceled()

                    return self._make_result(
                        success=False,
                        final_pose=self._get_final_pose(),
                        error_code=self.ERROR_NAVIGATION_REJECTED,
                        message="Navigation was canceled.",
                    )

                if request.enable_fine_alignment:
                    if self._is_final_route_segment():
                        current_pose = self._lookup_robot_pose(
                            log_error=False
                        )

                        if current_pose is not None:
                            position_error, _ = self._compute_errors(
                                request.target_pose,
                                current_pose,
                            )

                            if (
                                position_error
                                <= self._approach_radius
                            ):
                                early_stop_requested = True

                                self._node.get_logger().info(
                                    "Entered approach radius. "
                                    "Canceling Nav2 and starting "
                                    "fine alignment."
                                )

                                await self._cancel_nav2_goal()
                                self._publish_stop()
                                break

                time.sleep(self.CONTROL_PERIOD_SEC)

            if not early_stop_requested:
                if not result_future.done():
                    goal_handle.abort()

                    return self._make_result(
                        success=False,
                        final_pose=self._get_final_pose(),
                        error_code=self.ERROR_NAVIGATION_RESULT,
                        message=(
                            "Nav2 result was not available."
                        ),
                    )

                response = result_future.result()

                if response is None:
                    goal_handle.abort()

                    return self._make_result(
                        success=False,
                        final_pose=self._get_final_pose(),
                        error_code=self.ERROR_NAVIGATION_RESULT,
                        message="Nav2 returned an empty result.",
                    )

                if response.status != GoalStatus.STATUS_SUCCEEDED:
                    nav2_error_code = int(
                        getattr(
                            response.result,
                            "error_code",
                            0,
                        )
                    )
                    error_message = str(
                        getattr(
                            response.result,
                            "error_msg",
                            "",
                        )
                    )

                    final_segment = self._is_final_route_segment()
                    current_pose = self._lookup_robot_pose(
                        log_error=False
                    )
                    position_error = None

                    if current_pose is not None:
                        position_error, _ = self._compute_errors(
                            request.target_pose,
                            current_pose,
                        )

                    # Nav2 결과가 실패로 먼저 끝났더라도 로봇이 이미
                    # 최종 접근 반경 안에 있다면 정밀 정렬을 계속한다.
                    if (
                        request.enable_fine_alignment
                        and final_segment
                        and position_error is not None
                        and position_error <= self._approach_radius
                    ):
                        early_stop_requested = True

                        self._node.get_logger().warning(
                            "Nav2 failed inside the approach radius. "
                            "Continuing with fine alignment: "
                            f"status={response.status}, "
                            f"error_code={nav2_error_code}, "
                            f"position_error={position_error:.3f}, "
                            f"message={error_message or '<empty>'}"
                        )

                        self._publish_stop()

                    else:
                        failure_message = (
                            "Nav2 navigation failed: "
                            f"status={response.status}, "
                            f"error_code={nav2_error_code}, "
                            "position_error="
                            f"{position_error if position_error is not None else 'unknown'}, "
                            f"message={error_message or '<empty>'}"
                        )

                        # 원래 Nav2 goal은 이미 종료된 상태다.
                        self._active_nav2_goal_handle = None

                        if (
                            final_segment
                            and self._max_nav2_retries > 0
                        ):
                            self._node.get_logger().warning(
                                f"{failure_message} "
                                "Retrying only the final target."
                            )

                            retry_state, retry_message = (
                                await self._retry_final_target(
                                    goal_handle,
                                    request,
                                )
                            )

                            if retry_state == "CANCELED":
                                goal_handle.canceled()

                                return self._make_result(
                                    success=False,
                                    final_pose=self._get_final_pose(),
                                    error_code=(
                                        self.ERROR_NAVIGATION_REJECTED
                                    ),
                                    message=retry_message,
                                )

                            if retry_state == "APPROACH_REACHED":
                                early_stop_requested = True

                            elif retry_state == "SUCCEEDED":
                                self._node.get_logger().info(
                                    "Final-target Nav2 retry "
                                    "completed successfully."
                                )

                            else:
                                final_failure_message = (
                                    retry_message
                                    or failure_message
                                )

                                self._node.get_logger().error(
                                    final_failure_message
                                )

                                goal_handle.abort()

                                return self._make_result(
                                    success=False,
                                    final_pose=self._get_final_pose(),
                                    error_code=(
                                        self.ERROR_NAVIGATION_FAILED
                                    ),
                                    message=final_failure_message,
                                )

                        else:
                            self._node.get_logger().error(
                                failure_message
                            )

                            goal_handle.abort()

                            return self._make_result(
                                success=False,
                                final_pose=self._get_final_pose(),
                                error_code=(
                                    self.ERROR_NAVIGATION_FAILED
                                ),
                                message=failure_message,
                            )

            if request.enable_fine_alignment:
                alignment_state, final_pose = (
                    await self._run_fine_alignment(
                        goal_handle,
                        request.target_pose,
                    )
                )

                if alignment_state == "CANCELED":
                    goal_handle.canceled()

                    return self._make_result(
                        success=False,
                        final_pose=final_pose,
                        error_code=self.ERROR_NAVIGATION_REJECTED,
                        message=(
                            "Navigation was canceled during "
                            "fine alignment."
                        ),
                    )

                if alignment_state == "TF_FAILED":
                    goal_handle.abort()

                    return self._make_result(
                        success=False,
                        final_pose=final_pose,
                        error_code=self.ERROR_TRANSFORM,
                        message=(
                            "Could not obtain the robot pose "
                            "from TF during fine alignment."
                        ),
                    )

                if alignment_state != "SUCCEEDED":
                    goal_handle.abort()

                    return self._make_result(
                        success=False,
                        final_pose=final_pose,
                        error_code=self.ERROR_FINE_ALIGNMENT,
                        message=(
                            "Fine alignment did not finish "
                            "within the allowed time."
                        ),
                    )

                goal_handle.succeed()

                return self._make_result(
                    success=True,
                    final_pose=final_pose,
                    tolerance_satisfied=True,
                    error_code=0,
                    message=(
                        f"Reached and aligned with target "
                        f"'{request.target_id}'."
                    ),
                )

            final_pose = self._get_final_pose()

            position_error, yaw_error = self._compute_errors(
                request.target_pose,
                final_pose,
            )

            tolerance_satisfied = bool(
                position_error <= self._position_tolerance
                and yaw_error <= self._yaw_tolerance
            )

            goal_handle.succeed()

            return self._make_result(
                success=True,
                final_pose=final_pose,
                tolerance_satisfied=tolerance_satisfied,
                error_code=0,
                message=f"Reached target '{request.target_id}'.",
            )

        except Exception as error:
            self._node.get_logger().error(
                "Navigation execution failed: "
                f"{type(error).__name__}: {error}"
            )

            await self._cancel_nav2_goal()
            self._publish_stop()

            goal_handle.abort()

            return self._make_result(
                success=False,
                final_pose=self._get_final_pose(),
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=f"Navigation execution failed: {error}",
            )

        finally:
            self._publish_stop()

            self._active_goal_handle = None
            self._active_nav2_goal_handle = None
            self._active_target_pose = None

            self._latest_pose = None
            self._remaining_distance = 0.0
            self._remaining_poses = 0
            self._number_of_recoveries = 0

            self._total_poses = 0
            self._route_mode = False

            with self._state_lock:
                self._busy = False

    def _make_nav2_goal(
        self,
        request: NavigateToTarget.Goal,
    ):
        """Select the Nav2 action based on waypoint presence."""

        if request.waypoints:
            nav2_goal = NavigateThroughPoses.Goal()

            nav2_goal.poses = list(request.waypoints)
            nav2_goal.poses.append(request.target_pose)

            return self._navigate_through_poses_client, nav2_goal

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = request.target_pose

        return self._navigate_to_pose_client, nav2_goal

    def _handle_nav2_feedback(
        self,
        feedback_message,
    ) -> None:
        """Convert Nav2 feedback into shelving feedback."""

        target_pose = self._active_target_pose

        # A final feedback sample can arrive after cancellation or cleanup.
        if target_pose is None:
            return

        nav2_feedback = feedback_message.feedback

        with self._feedback_lock:
            self._latest_pose = nav2_feedback.current_pose
            self._remaining_distance = float(
                nav2_feedback.distance_remaining
            )
            self._number_of_recoveries = int(
                nav2_feedback.number_of_recoveries
            )

            if hasattr(
                nav2_feedback,
                "number_of_poses_remaining",
            ):
                self._remaining_poses = int(
                    nav2_feedback.number_of_poses_remaining
                )
            else:
                self._remaining_poses = 1

        current_index = self._current_waypoint_index()

        position_error, yaw_error = self._compute_errors(
            target_pose,
            nav2_feedback.current_pose,
        )

        self._publish_feedback(
            phase="NAVIGATING",
            current_waypoint_index=current_index,
            remaining_distance=float(
                nav2_feedback.distance_remaining
            ),
            position_error=position_error,
            yaw_error=yaw_error,
        )

    async def _retry_final_target(
        self,
        goal_handle,
        request: NavigateToTarget.Goal,
    ) -> tuple[str, str]:
        """Retry only the final target with NavigateToPose."""

        last_failure_message = (
            "Nav2 final-target retry did not run."
        )

        # 기존 경유점은 다시 보내지 않고 최종 목표만 남긴다.
        with self._feedback_lock:
            self._remaining_poses = 1

        for retry_number in range(
            1,
            self._max_nav2_retries + 1,
        ):
            if goal_handle.is_cancel_requested:
                await self._cancel_nav2_goal()
                self._publish_stop()

                return "CANCELED", "Navigation was canceled."

            current_pose = self._lookup_robot_pose(
                log_error=False
            )

            if (
                request.enable_fine_alignment
                and current_pose is not None
            ):
                position_error, _ = self._compute_errors(
                    request.target_pose,
                    current_pose,
                )

                if position_error <= self._approach_radius:
                    self._node.get_logger().info(
                        "Final target is already inside the "
                        "approach radius before retry. "
                        "Starting fine alignment."
                    )

                    self._publish_stop()

                    return "APPROACH_REACHED", ""

            if self._nav2_retry_delay_sec > 0.0:
                time.sleep(self._nav2_retry_delay_sec)

            if not self._navigate_to_pose_client.wait_for_server(
                timeout_sec=self._nav2_server_timeout_sec
            ):
                last_failure_message = (
                    "NavigateToPose server is unavailable "
                    f"during retry {retry_number}/"
                    f"{self._max_nav2_retries}."
                )

                self._node.get_logger().warning(
                    last_failure_message
                )
                continue

            retry_goal = NavigateToPose.Goal()
            retry_goal.pose = request.target_pose

            self._node.get_logger().warning(
                "Retrying the final Nav2 target: "
                f"retry={retry_number}/"
                f"{self._max_nav2_retries}"
            )

            send_goal_future = (
                self._navigate_to_pose_client.send_goal_async(
                    retry_goal,
                    feedback_callback=self._handle_nav2_feedback,
                )
            )

            retry_goal_handle = await send_goal_future

            if (
                retry_goal_handle is None
                or not retry_goal_handle.accepted
            ):
                last_failure_message = (
                    "Nav2 rejected final-target retry "
                    f"{retry_number}/"
                    f"{self._max_nav2_retries}."
                )

                self._node.get_logger().warning(
                    last_failure_message
                )
                continue

            self._active_nav2_goal_handle = retry_goal_handle
            result_future = retry_goal_handle.get_result_async()

            while rclpy.ok() and not result_future.done():
                if goal_handle.is_cancel_requested:
                    await self._cancel_nav2_goal()
                    self._publish_stop()

                    return "CANCELED", (
                        "Navigation was canceled during retry."
                    )

                if request.enable_fine_alignment:
                    current_pose = self._lookup_robot_pose(
                        log_error=False
                    )

                    if current_pose is not None:
                        position_error, _ = self._compute_errors(
                            request.target_pose,
                            current_pose,
                        )

                        if (
                            position_error
                            <= self._approach_radius
                        ):
                            self._node.get_logger().info(
                                "Entered approach radius during "
                                "final-target retry. Starting "
                                "fine alignment."
                            )

                            await self._cancel_nav2_goal()
                            self._publish_stop()

                            return "APPROACH_REACHED", ""

                time.sleep(self.CONTROL_PERIOD_SEC)

            if not result_future.done():
                await self._cancel_nav2_goal()

                last_failure_message = (
                    "Nav2 retry result was not available: "
                    f"retry={retry_number}/"
                    f"{self._max_nav2_retries}."
                )

                self._node.get_logger().warning(
                    last_failure_message
                )
                continue

            response = result_future.result()
            self._active_nav2_goal_handle = None

            if response is None:
                last_failure_message = (
                    "Nav2 returned an empty retry result: "
                    f"retry={retry_number}/"
                    f"{self._max_nav2_retries}."
                )

                self._node.get_logger().warning(
                    last_failure_message
                )
                continue

            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self._node.get_logger().info(
                    "Final-target retry succeeded: "
                    f"retry={retry_number}/"
                    f"{self._max_nav2_retries}."
                )

                return "SUCCEEDED", ""

            nav2_error_code = int(
                getattr(
                    response.result,
                    "error_code",
                    0,
                )
            )
            error_message = str(
                getattr(
                    response.result,
                    "error_msg",
                    "",
                )
            )

            current_pose = self._lookup_robot_pose(
                log_error=False
            )
            position_error = None

            if current_pose is not None:
                position_error, _ = self._compute_errors(
                    request.target_pose,
                    current_pose,
                )

            if (
                request.enable_fine_alignment
                and position_error is not None
                and position_error <= self._approach_radius
            ):
                self._node.get_logger().warning(
                    "Nav2 retry failed inside the approach "
                    "radius. Continuing with fine alignment: "
                    f"retry={retry_number}/"
                    f"{self._max_nav2_retries}, "
                    f"error_code={nav2_error_code}, "
                    f"position_error={position_error:.3f}"
                )

                self._publish_stop()

                return "APPROACH_REACHED", ""

            last_failure_message = (
                "Final-target Nav2 retry failed: "
                f"retry={retry_number}/"
                f"{self._max_nav2_retries}, "
                f"status={response.status}, "
                f"error_code={nav2_error_code}, "
                "position_error="
                f"{position_error if position_error is not None else 'unknown'}, "
                f"message={error_message or '<empty>'}"
            )

            self._node.get_logger().warning(
                last_failure_message
            )
            self._publish_stop()

        return "FAILED", last_failure_message

    async def _run_fine_alignment(
        self,
        goal_handle,
        target_pose: PoseStamped,
    ) -> tuple[str, PoseStamped]:
        """Move slowly until final position and yaw tolerances are met."""

        start_time = time.monotonic()
        last_pose = self._get_final_pose()
        tf_failure_started = None

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return "CANCELED", last_pose

                current_pose = self._lookup_robot_pose(
                    log_error=False
                )

                if current_pose is None:
                    if tf_failure_started is None:
                        tf_failure_started = time.monotonic()

                    if (
                        time.monotonic() - tf_failure_started
                        >= self.FINE_ALIGNMENT_TIMEOUT_SEC
                    ):
                        return "TF_FAILED", last_pose

                    self._publish_feedback(
                        phase="WAITING_FOR_TF",
                        current_waypoint_index=max(
                            self._total_poses - 1,
                            0,
                        ),
                        remaining_distance=0.0,
                        position_error=0.0,
                        yaw_error=0.0,
                    )

                    time.sleep(self.CONTROL_PERIOD_SEC)
                    continue

                tf_failure_started = None
                last_pose = current_pose

                position_error, yaw_error = self._compute_errors(
                    target_pose,
                    current_pose,
                )

                self._publish_feedback(
                    phase="FINE_ALIGNMENT",
                    current_waypoint_index=max(
                        self._total_poses - 1,
                        0,
                    ),
                    remaining_distance=position_error,
                    position_error=position_error,
                    yaw_error=yaw_error,
                )

                if (
                    position_error <= self._position_tolerance
                    and yaw_error <= self._yaw_tolerance
                ):
                    return "SUCCEEDED", current_pose

                if (
                    time.monotonic() - start_time
                    >= self.FINE_ALIGNMENT_TIMEOUT_SEC
                ):
                    return "TIMEOUT", current_pose

                velocity = self._calculate_alignment_velocity(
                    target_pose,
                    current_pose,
                    position_error,
                    yaw_error,
                )

                self._cmd_vel_publisher.publish(velocity)

                time.sleep(self.CONTROL_PERIOD_SEC)

            return "FAILED", last_pose

        finally:
            self._publish_stop()

    def _calculate_alignment_velocity(
        self,
        target_pose: PoseStamped,
        current_pose: PoseStamped,
        position_error: float,
        yaw_error: float,
    ) -> Twist:
        """Calculate a slow holonomic velocity command."""

        command = Twist()

        current_yaw = self._quaternion_to_yaw(
            current_pose.pose.orientation
        )
        target_yaw = self._quaternion_to_yaw(
            target_pose.pose.orientation
        )

        # 목표 방향이 아직 맞지 않으면 제자리 회전만 한다.
        # 반납기에 가까이 붙으면서 동시에 회전하면 Ridgeback의
        # 사각 footprint가 장애물과 겹쳐 collision monitor가
        # 회전을 차단할 수 있다.
        if yaw_error > self._yaw_tolerance:
            signed_yaw_error = self._normalize_angle(
                target_yaw - current_yaw
            )

            command.angular.z = self._clamp(
                self.YAW_GAIN * signed_yaw_error,
                -self._fine_alignment_angular_speed,
                self._fine_alignment_angular_speed,
            )
            return command

        dx = (
            target_pose.pose.position.x
            - current_pose.pose.position.x
        )
        dy = (
            target_pose.pose.position.y
            - current_pose.pose.position.y
        )

        # map 좌표의 오차를 base_link 좌표로 변환한다.
        error_x_base = (
            math.cos(current_yaw) * dx
            + math.sin(current_yaw) * dy
        )
        error_y_base = (
            -math.sin(current_yaw) * dx
            + math.cos(current_yaw) * dy
        )

        if position_error > self._position_tolerance:
            velocity_x = self.POSITION_GAIN * error_x_base
            velocity_y = self.POSITION_GAIN * error_y_base

            velocity_length = math.hypot(
                velocity_x,
                velocity_y,
            )

            if (
                velocity_length
                > self._fine_alignment_linear_speed
            ):
                scale = (
                    self._fine_alignment_linear_speed
                    / velocity_length
                )
                velocity_x *= scale
                velocity_y *= scale

            command.linear.x = velocity_x
            command.linear.y = velocity_y

        return command

    async def _cancel_nav2_goal(self) -> None:
        """Cancel the currently active Nav2 goal."""

        nav2_goal_handle = self._active_nav2_goal_handle

        if nav2_goal_handle is None:
            return

        try:
            cancel_future = (
                nav2_goal_handle.cancel_goal_async()
            )
            await cancel_future

        except Exception as error:
            self._node.get_logger().warning(
                f"Failed to cancel Nav2 goal: {error}"
            )

        finally:
            self._active_nav2_goal_handle = None

    def _is_final_route_segment(self) -> bool:
        """Return whether Nav2 is heading to the final destination."""

        if not self._route_mode:
            return True

        with self._feedback_lock:
            return self._remaining_poses <= 1

    def _current_waypoint_index(self) -> int:
        """Return a zero-based route progress index."""

        if self._total_poses <= 1:
            return 0

        with self._feedback_lock:
            remaining_poses = self._remaining_poses

        index = self._total_poses - remaining_poses

        return max(
            0,
            min(index, self._total_poses - 1),
        )

    def _lookup_robot_pose(
        self,
        *,
        log_error: bool,
    ) -> PoseStamped | None:
        """Read the current map-to-base pose from TF."""

        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
            )

        except TransformException as error:
            if log_error:
                self._node.get_logger().warning(
                    "Could not look up robot pose: "
                    f"{error}"
                )

            return None

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = transform.header.stamp

        pose.pose.position.x = (
            transform.transform.translation.x
        )
        pose.pose.position.y = (
            transform.transform.translation.y
        )
        pose.pose.position.z = (
            transform.transform.translation.z
        )

        pose.pose.orientation = transform.transform.rotation

        return pose

    def _get_final_pose(self) -> PoseStamped:
        """Return the newest available actual robot pose."""

        tf_pose = self._lookup_robot_pose(log_error=False)

        if tf_pose is not None:
            return tf_pose

        with self._feedback_lock:
            latest_pose = self._latest_pose

        if latest_pose is not None:
            return latest_pose

        if self._active_target_pose is not None:
            return self._active_target_pose

        return PoseStamped()

    def _publish_feedback(
        self,
        *,
        phase: str,
        current_waypoint_index: int,
        remaining_distance: float,
        position_error: float,
        yaw_error: float,
    ) -> None:
        """Publish NavigateToTarget feedback."""

        active_goal = self._active_goal_handle

        if active_goal is None:
            return

        feedback = NavigateToTarget.Feedback()

        feedback.phase = phase
        feedback.current_waypoint_index = (
            current_waypoint_index
        )
        feedback.total_waypoints = self._total_poses

        feedback.remaining_distance = remaining_distance
        feedback.position_error = position_error
        feedback.yaw_error = yaw_error
        feedback.retry_count = self._number_of_recoveries

        active_goal.publish_feedback(feedback)

    def _compute_errors(
        self,
        target_pose: PoseStamped,
        current_pose: PoseStamped,
    ) -> tuple[float, float]:
        """Calculate position and absolute yaw errors."""

        dx = (
            current_pose.pose.position.x
            - target_pose.pose.position.x
        )
        dy = (
            current_pose.pose.position.y
            - target_pose.pose.position.y
        )

        position_error = math.hypot(dx, dy)

        target_yaw = self._quaternion_to_yaw(
            target_pose.pose.orientation
        )
        current_yaw = self._quaternion_to_yaw(
            current_pose.pose.orientation
        )

        yaw_error = abs(
            self._normalize_angle(
                current_yaw - target_yaw
            )
        )

        return position_error, yaw_error

    @staticmethod
    def _quaternion_to_yaw(quaternion) -> float:
        """Convert quaternion orientation to yaw."""

        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )

        cos_yaw = 1.0 - 2.0 * (
            quaternion.y ** 2
            + quaternion.z ** 2
        )

        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize an angle to the [-pi, pi] range."""
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        """Restrict a numeric value to the given range."""
        return max(minimum, min(value, maximum))

    def _publish_stop(self) -> None:
        """Publish a zero velocity command."""
        self._cmd_vel_publisher.publish(Twist())

    def _make_result(
        self,
        *,
        success: bool,
        error_code: int,
        message: str,
        final_pose: PoseStamped | None = None,
        tolerance_satisfied: bool = False,
    ) -> NavigateToTarget.Result:
        """Build a shelving navigation result."""

        result = NavigateToTarget.Result()

        result.success = success
        result.error_code = error_code
        result.message = message
        result.tolerance_satisfied = tolerance_satisfied

        if final_pose is not None:
            result.final_pose = final_pose

        return result

    def destroy(self) -> None:
        """Destroy resources created by the controller."""

        # The ROS signal handler may already have invalidated the context
        # before the launch shutdown reaches this cleanup path.
        if rclpy.ok(context=self._node.context):
            self._publish_stop()

        self._navigate_to_pose_client.destroy()
        self._navigate_through_poses_client.destroy()

        if hasattr(self._tf_listener, "unregister"):
            self._tf_listener.unregister()
