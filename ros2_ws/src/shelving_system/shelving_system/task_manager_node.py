"""Top-level task manager node for the shelving system."""

from pathlib import Path

import math

import rclpy
from ament_index_python.packages import (
    get_package_share_directory,
)
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from shelving_interfaces.action import (
    LoadTray,
    NavigateToTarget,
    PlaceBook,
)
from shelving_interfaces.msg import (
    RobotStatus,
    ScenarioState,
    TrayJob,
)
from shelving_system.job_planner import (
    InvalidTrayJobError,
    JobPlan,
    JobPlanner,
    JobPlanningError,
)
from shelving_system.state_machine import (
    StateMachine,
    SystemState,
)
from shelving_system.yaml_data_manager import (
    ConfigurationError,
    YamlDataManager,
)


class TaskManagerNode(Node):
    """Receive tray jobs and coordinate the shelving workflow."""

    ERROR_JOB_PLANNING = 1001

    ERROR_NAVIGATION_SERVER = 2001
    ERROR_NAVIGATION_REJECTED = 2002
    ERROR_NAVIGATION_FAILED = 2003
    ERROR_NAVIGATION_RESULT = 2004

    ERROR_TRAY_SERVER = 3001
    ERROR_TRAY_REJECTED = 3002
    ERROR_TRAY_TIMEOUT = 3003
    ERROR_TRAY_FAILED = 3004

    ERROR_MANIPULATION_SERVER = 4001
    ERROR_MANIPULATION_REJECTED = 4002
    ERROR_MANIPULATION_FAILED = 4003
    ERROR_MANIPULATION_RESULT = 4004

    def __init__(self) -> None:
        """Initialize the task manager."""
        super().__init__("task_manager_node")

        package_share = Path(get_package_share_directory("shelving_system"))
        default_config_dir = package_share / "config"

        self.declare_parameter("tray_job_topic", "/return_machine/tray_job") 
        self.declare_parameter("status_topic", "/system/state")
        self.declare_parameter(
            "scenario_state_topic",
            "/scenario/state",
        )

        self.declare_parameter("navigation_action", "/navigate_to_target")

        self.declare_parameter('manipulation_action', '/place_book')

        self.declare_parameter(
            "navigation_server_timeout_sec",
            5.0,
        )

        # 위치 허용오차 5cm를 고려하여 65cm 목표를 보내고,
        # 실제로는 약 60cm 책장에서 후퇴하도록 한다.
        self.declare_parameter(
            "shelf_retreat_goal_distance_m",
            0.65,
        )
        self.declare_parameter(
            "global_frame",
            "map",
        )
        self.declare_parameter(
            "robot_base_frame",
            "base_link",
        )
        self.declare_parameter(
            "arm_base_frame",
            "arm_base_link",
        )

        self.declare_parameter(
            "load_tray_action",
            "/load_tray",
        )
        self.declare_parameter(
            "load_tray_server_timeout_sec",
            5.0,
        )

        self.declare_parameter(
            "shelf_map_path",
            str(default_config_dir / "shelf_map.yaml"),
        )
        self.declare_parameter(
            "book_profiles_path",
            str(default_config_dir / "book_profiles.yaml"),
        )
        self.declare_parameter(
            "system_config_path",
            str(default_config_dir / "system.yaml"),
        )

        tray_job_topic = str(
            self.get_parameter("tray_job_topic").value
        )
        status_topic = str(
            self.get_parameter("status_topic").value
        )
        navigation_action = str(
            self.get_parameter("navigation_action").value
        )

        manipulation_action = str(self.get_parameter('manipulation_action').value)

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._status_publisher = self.create_publisher(
            RobotStatus,
            status_topic,
            qos_profile,
        )

        self._tray_job_subscription = (
            self.create_subscription(
                TrayJob,
                tray_job_topic,
                self._handle_tray_job,
                qos_profile,
            )
        )

        scenario_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._scenario_state_subscription = (
            self.create_subscription(
                ScenarioState,
                str(
                    self.get_parameter(
                        "scenario_state_topic"
                    ).value
                ),
                self._handle_scenario_state,
                scenario_qos,
            )
        )

        self._navigation_client = ActionClient(
            self,
            NavigateToTarget,
            navigation_action,
        )
        self._navigation_goal_handle = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )

        self._load_tray_client = ActionClient(
            self,
            LoadTray,
            str(
                self.get_parameter(
                    "load_tray_action"
                ).value
            ),
        )
        self._load_tray_goal_handle = None

        self._manipulation_client = ActionClient(self, PlaceBook, manipulation_action)
        self._manipulation_goal_handle = None

        self._fsm = StateMachine()
        self._active_job_id = ""

        self._current_plan: JobPlan | None = None
        self._current_task_index = 0
        self._completed_book_ids: list[str] = []
        self._accepted_job_ids: set[str] = set()
        self._active_navigation_target_type = ""
        self._active_navigation_target_id = ""
        self._scenario_run_id = ""
        self._scenario_reset_pending = False
        self._action_generation = 0

        self._status_message = "Initializing task manager."

        self._data_manager = YamlDataManager(
            shelf_map_path=self.get_parameter(
                "shelf_map_path"
            ).value,
            book_profiles_path=self.get_parameter(
                "book_profiles_path"
            ).value,
            system_config_path=self.get_parameter(
                "system_config_path"
            ).value,
        )
        self._job_planner = JobPlanner(
            data_manager=self._data_manager
        )

        self._fsm.transition(SystemState.IDLE)
        self._status_message = "Waiting for a tray job."

        self._status_timer = self.create_timer(
            1.0,
            self._publish_status,
        )

        self.get_logger().info(
            "Task manager is ready. "
            f"Listening on '{tray_job_topic}'."
        )
        self.get_logger().info(
            "Navigation action client configured for "
            f"'{navigation_action}'."
        )
        self.get_logger().info(
            "Manipulation action client configured for "
            f"'{manipulation_action}'."
        )

    def _handle_scenario_state(
        self,
        message: ScenarioState,
    ) -> None:
        """Stop 중인 작업을 폐기하고 다음 Play를 준비한다."""
        if message.state in (
            ScenarioState.STOPPED,
            ScenarioState.RESETTING,
        ):
            if not self._scenario_reset_pending:
                self._reset_for_scenario_restart()
            return

        if message.state != ScenarioState.READY:
            return

        run_id = message.run_id.strip()

        if run_id:
            self._scenario_run_id = run_id

        if not self._scenario_reset_pending:
            return

        self._scenario_reset_pending = False
        self._status_message = (
            "Scenario reset completed. Waiting for a tray job."
        )
        self._publish_status()
        self.get_logger().info(
            "Scenario READY received. Task manager is ready "
            f"for a new cycle: run_id={run_id or '<empty>'}"
        )

    def _reset_for_scenario_restart(self) -> None:
        """현재 action과 FSM을 무효화하고 IDLE로 복원한다."""
        previous_state = self._fsm.current_state.name
        previous_job_id = self._active_job_id

        self._scenario_reset_pending = True
        self._action_generation += 1

        self._cancel_active_goal(
            self._navigation_goal_handle,
            "navigation",
        )
        self._cancel_active_goal(
            self._load_tray_goal_handle,
            "load_tray",
        )
        self._cancel_active_goal(
            self._manipulation_goal_handle,
            "manipulation",
        )

        self._navigation_goal_handle = None
        self._load_tray_goal_handle = None
        self._manipulation_goal_handle = None

        self._fsm.reset()
        self._active_job_id = ""
        self._current_plan = None
        self._current_task_index = 0
        self._completed_book_ids.clear()
        self._accepted_job_ids.clear()
        self._active_navigation_target_type = ""
        self._active_navigation_target_id = ""

        self._status_message = (
            "Scenario stopped. Waiting for reset to complete."
        )
        self._publish_status()

        self.get_logger().info(
            "Task manager reset for scenario restart: "
            f"previous_state={previous_state}, "
            f"previous_job_id={previous_job_id or '<none>'}, "
            f"generation={self._action_generation}"
        )

    def _cancel_active_goal(
        self,
        goal_handle,
        label: str,
    ) -> None:
        """현재 ROS action goal을 best-effort로 취소한다."""
        if goal_handle is None:
            return

        try:
            goal_handle.cancel_goal_async()
            self.get_logger().info(
                f"Scenario reset requested {label} goal cancellation."
            )
        except Exception as error:
            self.get_logger().warning(
                f"Failed to request {label} goal cancellation: {error}"
            )

    def _handle_tray_job(
        self,
        message: TrayJob,
    ) -> None:
        """Validate a TrayJob and create its execution plan."""
        job_id = message.job_id.strip()

        self.get_logger().info(
            "Received TrayJob: "
            f"job_id={message.job_id}, "
            f"tray_id={message.tray_id}, "
            f"books={len(message.book_ids)}"
        )

        if job_id in self._accepted_job_ids:
            self.get_logger().warning(
                f"Ignoring duplicate job '{job_id}'."
            )
            return

        if self._fsm.current_state is not SystemState.IDLE:
            self.get_logger().warning(
                "Cannot accept a new job while the system is "
                f"in state '{self._fsm.current_state.name}'."
            )
            return

        self._active_job_id = job_id
        self._fsm.transition(SystemState.PLANNING)
        self._status_message = (
            f"Planning job '{job_id}'."
        )
        self._publish_status()

        try:
            plan = self._job_planner.create_plan(
                job_id=message.job_id,
                tray_id=message.tray_id,
                book_ids=message.book_ids,
                rfid_tags=message.rfid_tags,
                classification_codes=(
                    message.classification_codes
                ),
            )
        except (
            InvalidTrayJobError,
            JobPlanningError,
            ConfigurationError,
        ) as error:
            self._handle_planning_failure(error)
            return

        self._current_plan = plan
        self._current_task_index = 0
        self._completed_book_ids.clear()
        self._active_job_id = plan.job_id
        self._accepted_job_ids.add(plan.job_id)

        self.get_logger().info(
            "Job plan created: "
            f"job_id={plan.job_id}, "
            f"tray_id={plan.tray_id}, "
            f"tasks={len(plan.tasks)}"
        )

        for task_number, task in enumerate(
            plan.tasks,
            start=1,
        ):
            self.get_logger().info(
                f"Task {task_number}: "
                f"book_id={task.book_id}, "
                f"classification={task.classification_code}, "
                f"target_shelf={task.shelf_id}"
            )

        self._fsm.transition(
            SystemState.NAV_TO_RETURN
        )
        self._status_message = (
            "Sending navigation goal to the "
            "return station."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "PLANNING -> NAV_TO_RETURN"
        )

        self._send_navigation_to_return()

    def _build_waypoints(
        self,
        *,
        frame_id: str,
        waypoint_data: list[dict],
    ) -> list[PoseStamped]:
        """Convert YAML waypoint mappings to PoseStamped messages."""
        waypoints: list[PoseStamped] = []
        stamp = self.get_clock().now().to_msg()

        for data in waypoint_data:
            position = data["position"]
            orientation = data["orientation"]

            waypoint = PoseStamped()
            waypoint.header.stamp = stamp
            waypoint.header.frame_id = frame_id

            waypoint.pose.position.x = float(position["x"])
            waypoint.pose.position.y = float(position["y"])
            waypoint.pose.position.z = float(position["z"])

            waypoint.pose.orientation.x = float(
                orientation["x"]
            )
            waypoint.pose.orientation.y = float(
                orientation["y"]
            )
            waypoint.pose.orientation.z = float(
                orientation["z"]
            )
            waypoint.pose.orientation.w = float(
                orientation["w"]
            )

            waypoints.append(waypoint)

        return waypoints

    def _send_navigation_to_return(self) -> None:
        """Send a navigation goal for the return station."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        return_pose = self._current_plan.return_station_pose
        frame_id = str(return_pose["frame_id"])

        waypoints = self._build_waypoints(
            frame_id=frame_id,
            waypoint_data=list(
                return_pose.get("waypoints", [])
            ),
        )

        self._send_navigation_goal(
            target_type="return_station",
            target_id="return_station",
            frame_id=frame_id,
            target_pose=return_pose,
            waypoints=waypoints,
            enable_fine_alignment=True,
        )

    def _send_navigation_to_shelf(self) -> None:
        """Send a navigation goal for the current book's shelf."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        if self._current_task_index >= len(
            self._current_plan.tasks
        ):
            self._fail_navigation(
                error_code=self.ERROR_JOB_PLANNING,
                message="No book task is available.",
            )
            return

        task = self._current_plan.tasks[
            self._current_task_index
        ]

        shelf_data = (
            self._data_manager.get_shelf_for_classification(
                task.classification_code
            )
        )

        waypoints = self._build_waypoints(
            frame_id=task.shelf_frame_id,
            waypoint_data=list(
                shelf_data.get("waypoints", [])
            ),
        )

        self._send_navigation_goal(
            target_type="shelf",
            target_id=task.shelf_id,
            frame_id=task.shelf_frame_id,
            target_pose=task.shelf_observation_pose,
            waypoints=waypoints,
            enable_fine_alignment=True,
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        """Return planar yaw from a quaternion."""
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _send_shelf_retreat(self) -> None:
        """Retreat from the shelf before requesting the home route."""
        global_frame = str(
            self.get_parameter("global_frame").value
        )
        robot_base_frame = str(
            self.get_parameter("robot_base_frame").value
        )
        arm_base_frame = str(
            self.get_parameter("arm_base_frame").value
        )
        retreat_distance = float(
            self.get_parameter(
                "shelf_retreat_goal_distance_m"
            ).value
        )

        # 최대 직접 이동거리 검증은 navigation_controller가 담당한다.
        if retreat_distance <= 0.0:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Shelf retreat distance must be positive: "
                    f"{retreat_distance:.3f}"
                ),
            )
            return

        try:
            base_transform = self._tf_buffer.lookup_transform(
                global_frame,
                robot_base_frame,
                Time(),
            )
            arm_transform = self._tf_buffer.lookup_transform(
                global_frame,
                arm_base_frame,
                Time(),
            )
        except TransformException as error:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Failed to obtain TF for shelf retreat: "
                    f"{error}"
                ),
            )
            return

        base_translation = (
            base_transform.transform.translation
        )
        base_rotation = (
            base_transform.transform.rotation
        )
        arm_rotation = (
            arm_transform.transform.rotation
        )

        arm_yaw = self._yaw_from_quaternion(
            arm_rotation
        )

        # arm_base_link +Y가 책장 방향이므로 -Y가 후퇴 방향이다.
        target_x = (
            base_translation.x
            + math.sin(arm_yaw) * retreat_distance
        )
        target_y = (
            base_translation.y
            - math.cos(arm_yaw) * retreat_distance
        )

        retreat_pose = {
            "position": {
                "x": target_x,
                "y": target_y,
                "z": base_translation.z,
            },
            "orientation": {
                "x": base_rotation.x,
                "y": base_rotation.y,
                "z": base_rotation.z,
                "w": base_rotation.w,
            },
        }

        self._status_message = (
            "Retreating from the shelf before returning home."
        )
        self._publish_status()

        self.get_logger().info(
            "Sending shelf-retreat navigation goal: "
            f"goal_distance={retreat_distance:.2f}m, "
            f"target_map=({target_x:+.3f}, {target_y:+.3f})"
        )

        self._send_navigation_goal(
            target_type="shelf_retreat",
            target_id="shelf_retreat",
            frame_id=global_frame,
            target_pose=retreat_pose,
            waypoints=[],
            enable_fine_alignment=True,
        )

    def _handle_shelf_retreat_arrival(self) -> None:
        """Send the normal home goal after clearing the shelf."""
        if (
            self._fsm.current_state
            is not SystemState.RETURN_HOME
        ):
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Shelf-retreat result received in "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        self.get_logger().info(
            "Shelf retreat completed. "
            "Sending the home navigation goal."
        )

        self._status_message = (
            "Shelf retreat completed. Returning home."
        )
        self._publish_status()

        self._send_navigation_home()

    def _send_navigation_home(self) -> None:
        """Send a navigation goal for the home position."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        home_pose = self._current_plan.home_pose
        frame_id = str(home_pose["frame_id"])

        waypoints = self._build_waypoints(
            frame_id=frame_id,
            waypoint_data=list(
                home_pose.get("waypoints", [])
            ),
        )

        self._send_navigation_goal(
            target_type="home",
            target_id="home",
            frame_id=frame_id,
            target_pose=home_pose,
            waypoints=waypoints,
            enable_fine_alignment=True,
        )

    def _send_navigation_goal(
        self,
        target_type: str,
        target_id: str,
        frame_id: str,
        target_pose: dict,
        waypoints: list[PoseStamped],
        enable_fine_alignment: bool,
    ) -> None:
        """Send one navigation goal to the AMR."""
        timeout_sec = float(
            self.get_parameter(
                "navigation_server_timeout_sec"
            ).value
        )

        self.get_logger().info(
            "Waiting for navigation action server."
        )

        if not self._navigation_client.wait_for_server(
            timeout_sec=timeout_sec
        ):
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_SERVER,
                message=(
                    "Navigation action server is not "
                    "available."
                ),
            )
            return

        goal = NavigateToTarget.Goal()
        goal.job_id = self._active_job_id
        goal.target_type = target_type
        goal.target_id = target_id
        goal.waypoints = list(waypoints)
        goal.enable_fine_alignment = bool(enable_fine_alignment)

        position = target_pose["position"]
        orientation = target_pose["orientation"]

        goal.target_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )
        goal.target_pose.header.frame_id = frame_id

        goal.target_pose.pose.position.x = float(
            position["x"]
        )
        goal.target_pose.pose.position.y = float(
            position["y"]
        )
        goal.target_pose.pose.position.z = float(
            position["z"]
        )

        goal.target_pose.pose.orientation.x = float(
            orientation["x"]
        )
        goal.target_pose.pose.orientation.y = float(
            orientation["y"]
        )
        goal.target_pose.pose.orientation.z = float(
            orientation["z"]
        )
        goal.target_pose.pose.orientation.w = float(
            orientation["w"]
        )

        self._active_navigation_target_type = target_type
        self._active_navigation_target_id = target_id

        self.get_logger().info(
            "Sending navigation goal: "
            f"job_id={goal.job_id}, "
            f"target_type={goal.target_type}, "
            f"target_id={goal.target_id}, "
            f"waypoints={len(goal.waypoints)}, "
            f"fine_alignment={goal.enable_fine_alignment}, "
            f"x={goal.target_pose.pose.position.x:.2f}, "
            f"y={goal.target_pose.pose.position.y:.2f}"
        )

        generation = self._action_generation

        send_goal_future = (
            self._navigation_client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda feedback_message,
                    generation=generation:
                    self._handle_navigation_feedback(
                        feedback_message,
                        generation,
                    )
                ),
            )
        )
        send_goal_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_navigation_goal_response(
                future,
                generation,
            )
        )

    def _handle_navigation_goal_response(
        self,
        future,
        generation: int,
    ) -> None:
        """Handle acceptance or rejection of a goal."""
        try:
            goal_handle = future.result()
        except Exception as error:
            if generation != self._action_generation:
                return
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Failed to send navigation goal: "
                    f"{error}"
                ),
            )
            return

        if generation != self._action_generation:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_REJECTED,
                message=(
                    "Navigation action server rejected "
                    "the goal."
                ),
            )
            return

        self._navigation_goal_handle = goal_handle

        self.get_logger().info(
            "Navigation goal was accepted."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_navigation_result(
                future,
                generation,
            )
        )

    def _handle_navigation_feedback(
        self,
        feedback_message,
        generation: int,
    ) -> None:
        """Handle navigation progress feedback."""
        if generation != self._action_generation:
            return

        feedback = feedback_message.feedback

        if feedback.total_waypoints > 0:
            waypoint_status = (
                f", waypoint="
                f"{feedback.current_waypoint_index + 1}"
                f"/{feedback.total_waypoints}"
            )
        else:
            waypoint_status = ""

        self._status_message = (
            f"Navigating to "
            f"'{self._active_navigation_target_id}': "
            f"phase={feedback.phase}"
            f"{waypoint_status}, "
            f"remaining={feedback.remaining_distance:.2f} m"
        )
        self._publish_status()

        self.get_logger().info(
            "Navigation feedback: "
            f"target_id="
            f"{self._active_navigation_target_id}, "
            f"phase={feedback.phase}"
            f"{waypoint_status}, "
            f"remaining_distance="
            f"{feedback.remaining_distance:.2f}, "
            f"position_error="
            f"{feedback.position_error:.2f}, "
            f"yaw_error={feedback.yaw_error:.2f}"
        )

    def _handle_navigation_result(
        self,
        future,
        generation: int,
    ) -> None:
        """Handle the completed navigation result."""
        if generation != self._action_generation:
            return

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Failed to receive navigation result: "
                    f"{error}"
                ),
            )
            return

        if not (
            result.success
            and result.tolerance_satisfied
        ):
            error_code = int(result.error_code)

            if error_code == 0:
                error_code = self.ERROR_NAVIGATION_FAILED

            self._fail_navigation(
                error_code=error_code,
                message=(
                    result.message
                    or "Navigation failed."
                ),
            )
            return

        self._navigation_goal_handle = None

        self.get_logger().info(
            "Navigation succeeded: "
            f"{result.message}"
        )

        if (
            self._active_navigation_target_type
            == "return_station"
        ):
            self._handle_return_station_arrival()
            return

        if (
            self._active_navigation_target_type
            == "shelf_retreat"
        ):
            self._handle_shelf_retreat_arrival()
            return

        if self._active_navigation_target_type == "shelf":
            self._handle_shelf_arrival()
            return

        if self._active_navigation_target_type == "home":
            self._handle_home_arrival()
            return

        self._fail_navigation(
            error_code=self.ERROR_NAVIGATION_RESULT,
            message=(
                "Navigation completed for an unknown "
                f"target type: "
                f"'{self._active_navigation_target_type}'."
            ),
        )

    def _handle_return_station_arrival(self) -> None:
        """Handle successful arrival at the return station."""
        if (
            self._fsm.current_state
            is not SystemState.NAV_TO_RETURN
        ):
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Return-station result received in "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        self._fsm.transition(
            SystemState.RECEIVE_TRAY
        )
        self._status_message = (
            "Arrived at the return station. "
            "Receiving the tray."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "NAV_TO_RETURN -> RECEIVE_TRAY"
        )

        self._send_load_tray_goal()

    def _send_load_tray_goal(self) -> None:
        """Request tray transfer from the return machine."""
        if self._current_plan is None:
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_FAILED,
                message="Current job plan does not exist.",
            )
            return

        timeout_sec = float(
            self.get_parameter(
                "load_tray_server_timeout_sec"
            ).value
        )

        self.get_logger().info(
            "Waiting for LoadTray action server."
        )

        if not self._load_tray_client.wait_for_server(
            timeout_sec=timeout_sec
        ):
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_SERVER,
                message=(
                    "LoadTray action server is not "
                    "available."
                ),
            )
            return

        goal = LoadTray.Goal()
        goal.job_id = self._current_plan.job_id
        goal.tray_id = self._current_plan.tray_id

        generation = self._action_generation

        send_goal_future = (
            self._load_tray_client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda feedback_message,
                    generation=generation:
                    self._handle_load_tray_feedback(
                        feedback_message,
                        generation,
                    )
                ),
            )
        )
        send_goal_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_load_tray_goal_response(
                future,
                generation,
            )
        )

    def _handle_load_tray_goal_response(
        self,
        future,
        generation: int,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            if generation != self._action_generation:
                return
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_FAILED,
                message=(
                    "Failed to send LoadTray goal: "
                    f"{error}"
                ),
            )
            return

        if generation != self._action_generation:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_REJECTED,
                message="LoadTray goal was rejected.",
            )
            return

        self._load_tray_goal_handle = goal_handle

        self.get_logger().info(
            "LoadTray goal was accepted."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_load_tray_result(
                future,
                generation,
            )
        )

    def _handle_load_tray_feedback(
        self,
        feedback_message,
        generation: int,
    ) -> None:
        if generation != self._action_generation:
            return

        feedback = feedback_message.feedback

        self._status_message = (
            "Receiving tray: "
            f"phase={feedback.phase}, "
            f"progress={feedback.progress:.0%}"
        )
        self._publish_status()

        self.get_logger().info(
            "LoadTray feedback: "
            f"phase={feedback.phase}, "
            f"progress={feedback.progress:.2f}"
        )

    def _handle_load_tray_result(
        self,
        future,
        generation: int,
    ) -> None:
        if generation != self._action_generation:
            return

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_FAILED,
                message=(
                    "Failed to receive LoadTray result: "
                    f"{error}"
                ),
            )
            return

        if not result.success:
            error_code = int(result.error_code)

            if error_code == 0:
                error_code = self.ERROR_TRAY_FAILED

            self._fail_tray_loading(
                error_code=error_code,
                message=(
                    result.message
                    or "Tray loading failed."
                ),
            )
            return

        if (
            self._fsm.current_state
            is not SystemState.RECEIVE_TRAY
        ):
            self._fail_tray_loading(
                error_code=self.ERROR_TRAY_FAILED,
                message=(
                    "LoadTray result received in state "
                    f"'{self._fsm.current_state.name}'."
                ),
            )
            return

        self._load_tray_goal_handle = None

        self.get_logger().info(
            f"Tray loading succeeded: {result.message}"
        )

        self._start_first_book_task()

    def _start_first_book_task(self) -> None:
        """Start processing the first book."""
        self._current_task_index = 0
        self._start_current_book_task()

    def _start_current_book_task(self) -> None:
        """Select the current book and navigate to its shelf."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_JOB_PLANNING,
                message="Current job plan does not exist.",
            )
            return

        if self._current_task_index >= len(
            self._current_plan.tasks
        ):
            self._fail_navigation(
                error_code=self.ERROR_JOB_PLANNING,
                message="No current book task is available.",
            )
            return

        task = self._current_plan.tasks[
            self._current_task_index
        ]

        previous_state = self._fsm.current_state.name

        self._fsm.transition(
            SystemState.SELECT_BOOK
        )
        self._status_message = (
            f"Selected book '{task.book_id}'."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            f"{previous_state} -> SELECT_BOOK"
        )
        self.get_logger().info(
            "Selected book task: "
            f"book_id={task.book_id}, "
            f"task_number="
            f"{self._current_task_index + 1}/"
            f"{len(self._current_plan.tasks)}, "
            f"target_shelf={task.shelf_id}"
        )

        same_shelf_as_previous = False

        if self._current_task_index > 0:
            previous_task = self._current_plan.tasks[
                self._current_task_index - 1
            ]
            same_shelf_as_previous = (
                previous_task.shelf_id == task.shelf_id
                and previous_task.shelf_frame_id
                == task.shelf_frame_id
            )

        if same_shelf_as_previous:
            self._fsm.transition(
                SystemState.PLACE_BOOK
            )
            self._status_message = (
                f"Already at shelf '{task.shelf_id}'. "
                "Starting the next book placement."
            )
            self._publish_status()

            self.get_logger().info(
                "Same shelf as previous task; "
                "skipping redundant shelf navigation: "
                f"shelf_id={task.shelf_id}"
            )
            self.get_logger().info(
                "FSM transition completed: "
                "SELECT_BOOK -> PLACE_BOOK"
            )

            self._send_place_book_goal()
            return

        self._fsm.transition(
            SystemState.NAV_TO_SHELF
        )
        self._status_message = (
            f"Navigating to shelf '{task.shelf_id}'."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "SELECT_BOOK -> NAV_TO_SHELF"
        )

        self._send_navigation_to_shelf()

    def _handle_home_arrival(self) -> None:
        """Complete the job after returning home."""
        if (
            self._fsm.current_state
            is not SystemState.RETURN_HOME
        ):
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Home navigation result received in "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        completed_job_id = self._active_job_id
        completed_count = len(
            self._completed_book_ids
        )

        self._fsm.transition(
            SystemState.COMPLETED
        )
        self._status_message = (
            f"Job '{completed_job_id}' completed."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "RETURN_HOME -> COMPLETED"
        )
        self.get_logger().info(
            "Job completed: "
            f"job_id={completed_job_id}, "
            f"placed_books={completed_count}"
        )

        self._fsm.transition(
            SystemState.IDLE
        )

        self._active_job_id = ""
        self._current_plan = None
        self._current_task_index = 0
        self._completed_book_ids.clear()
        self._active_navigation_target_type = ""
        self._active_navigation_target_id = ""

        self._status_message = (
            "Waiting for a tray job."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "COMPLETED -> IDLE"
        )

    def _handle_shelf_arrival(self) -> None:
        """Handle successful arrival at the target shelf."""
        if (
            self._fsm.current_state
            is not SystemState.NAV_TO_SHELF
        ):
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Shelf navigation result received in "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        self._fsm.transition(SystemState.PLACE_BOOK)
        self._status_message = (
            f"Arrived at shelf "
            f"'{self._active_navigation_target_id}'. "
            "Starting book placement."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "NAV_TO_SHELF -> PLACE_BOOK"
        )

        self._send_place_book_goal()

    def _send_place_book_goal(self) -> None:
        """Send the current book-placement goal."""
        if self._current_plan is None:
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        if self._current_task_index >= len(
            self._current_plan.tasks
        ):
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message="No current book task is available.",
            )
            return

        timeout_sec = self._data_manager.get_timeout(
            "manipulation"
        )

        self.get_logger().info(
            "Waiting for manipulation action server."
        )

        if not self._manipulation_client.wait_for_server(
            timeout_sec=timeout_sec
        ):
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_SERVER,
                message=(
                    "Manipulation action server is not "
                    "available."
                ),
            )
            return

        task = self._current_plan.tasks[
            self._current_task_index
        ]

        goal = PlaceBook.Goal()

        self.get_logger().info(
            "Sending book-placement command: "
            f"book_id={task.book_id}, "
            f"task_index={self._current_task_index}"
        )

        generation = self._action_generation

        send_goal_future = (
            self._manipulation_client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda feedback_message,
                    generation=generation:
                    self._handle_manipulation_feedback(
                        feedback_message,
                        generation,
                    )
                ),
            )
        )
        send_goal_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_manipulation_goal_response(
                future,
                generation,
            )
        )

    def _handle_manipulation_goal_response(
        self,
        future,
        generation: int,
    ) -> None:
        """Handle acceptance of a placement goal."""
        try:
            goal_handle = future.result()
        except Exception as error:
            if generation != self._action_generation:
                return
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message=(
                    "Failed to send manipulation goal: "
                    f"{error}"
                ),
            )
            return

        if generation != self._action_generation:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_manipulation(
                error_code=(
                    self.ERROR_MANIPULATION_REJECTED
                ),
                message=(
                    "Manipulation action server rejected "
                    "the goal."
                ),
            )
            return

        self._manipulation_goal_handle = goal_handle

        self.get_logger().info(
            "Book-placement goal was accepted."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, generation=generation:
            self._handle_manipulation_result(
                future,
                generation,
            )
        )

    def _handle_manipulation_feedback(
        self,
        feedback_message,
        generation: int,
    ) -> None:
        """Handle book-placement phase feedback."""
        if generation != self._action_generation:
            return

        feedback = feedback_message.feedback

        self._status_message = (
            "Placing book: "
            f"phase={feedback.phase}"
        )
        self._publish_status()

        self.get_logger().info(
            "Manipulation feedback: "
            f"phase={feedback.phase}"
        )

    def _handle_manipulation_result(
        self,
        future,
        generation: int,
    ) -> None:
        """Handle the completed book-placement result."""
        if generation != self._action_generation:
            return

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message=(
                    "Failed to receive manipulation result: "
                    f"{error}"
                ),
            )
            return

        if not result.success:
            error_code = int(result.error_code)

            if error_code == 0:
                error_code = self.ERROR_MANIPULATION_FAILED

            failure_description = (
                result.message
                or "Book placement failed."
            )

            self._fail_manipulation(
                error_code=error_code,
                message=failure_description,
            )
            return

        if (
            self._fsm.current_state
            is not SystemState.PLACE_BOOK
        ):
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message=(
                    "Manipulation result received in "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        self._manipulation_goal_handle = None

        task = self._current_plan.tasks[
            self._current_task_index
        ]

        self.get_logger().info(
            "Book placement succeeded: "
            f"book_id={task.book_id}, "
            f"message={result.message}"
        )

        self._fsm.transition(
            SystemState.UPDATE_DATA
        )
        self._status_message = (
            f"Book '{task.book_id}' was placed. "
            "Waiting to update slot data."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "PLACE_BOOK -> UPDATE_DATA"
        )

        self._update_data_and_continue()

    def _update_data_and_continue(self) -> None:
        """Record placement and continue or return home."""
        if self._current_plan is None:
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        if (
            self._fsm.current_state
            is not SystemState.UPDATE_DATA
        ):
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message=(
                    "Data update requested in unexpected "
                    f"state '{self._fsm.current_state.name}'."
                ),
            )
            return

        task = self._current_plan.tasks[
            self._current_task_index
        ]

        if task.book_id not in self._completed_book_ids:
            self._completed_book_ids.append(
                task.book_id
            )

        self.get_logger().info(
            "Placement record updated: "
            f"book_id={task.book_id}, "
            f"shelf_id={task.shelf_id}, "
            f"completed_books="
            f"{len(self._completed_book_ids)}/"
            f"{len(self._current_plan.tasks)}"
        )

        self._fsm.transition(
            SystemState.NEXT_BOOK
        )
        self._status_message = (
            "Checking for another book."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "UPDATE_DATA -> NEXT_BOOK"
        )

        self._current_task_index += 1

        if self._current_task_index < len(
            self._current_plan.tasks
        ):
            self.get_logger().info(
                "Another book remains in the tray."
            )
            self._start_current_book_task()
            return

        self.get_logger().info(
            "All books in the tray were processed."
        )

        self._fsm.transition(
            SystemState.RETURN_HOME
        )
        self._status_message = (
            "All books completed. Returning home."
        )
        self._publish_status()

        self.get_logger().info(
            "FSM transition completed: "
            "NEXT_BOOK -> RETURN_HOME"
        )

        self._send_shelf_retreat()

    def _fail_manipulation(
        self,
        error_code: int,
        message: str,
    ) -> None:
        """Move the FSM to FAILED after manipulation failure."""
        self._fsm.fail(
            error_code=error_code,
            message=message,
        )
        self._status_message = message
        self._publish_status()

        self.get_logger().error(message)

    def _handle_planning_failure(
        self,
        error: Exception,
    ) -> None:
        """Move the FSM to FAILED after planning failure."""
        error_message = str(error)

        self._fsm.fail(
            error_code=self.ERROR_JOB_PLANNING,
            message=error_message,
        )
        self._status_message = error_message
        self._publish_status()

        self.get_logger().error(
            "Job planning failed: "
            f"{error_message}"
        )

    def _fail_tray_loading(
        self,
        *,
        error_code: int,
        message: str,
    ) -> None:
        self._fsm.fail(
            error_code=error_code,
            message=message,
        )
        self._status_message = message
        self._publish_status()
        self.get_logger().error(message)

    def _fail_navigation(
        self,
        error_code: int,
        message: str,
    ) -> None:
        """Move the FSM to FAILED after navigation failure."""
        self._fsm.fail(
            error_code=error_code,
            message=message,
        )
        self._status_message = message
        self._publish_status()

        self.get_logger().error(message)

    def _publish_status(self) -> None:
        """Publish the current task-manager status."""
        now = self.get_clock().now().to_msg()

        message = RobotStatus()
        message.header.stamp = now
        message.component = "task_manager"
        message.state = self._fsm.current_state.name
        message.active_job_id = self._active_job_id
        message.progress = self._calculate_progress()
        message.error_code = self._fsm.error_code
        message.message = self._status_message
        message.heartbeat_time = now

        self._status_publisher.publish(message)

    def _calculate_progress(self) -> float:
        """Return temporary progress for the current state."""
        progress_by_state = {
            SystemState.INITIALIZING: 0.0,
            SystemState.IDLE: 0.0,
            SystemState.PLANNING: 0.05,
            SystemState.NAV_TO_RETURN: 0.10,
            SystemState.RECEIVE_TRAY: 0.20,
            SystemState.SELECT_BOOK: 0.25,
            SystemState.NAV_TO_SHELF: 0.35,
            SystemState.PLACE_BOOK: 0.60,
            SystemState.UPDATE_DATA: 0.75,
            SystemState.NEXT_BOOK: 0.80,
            SystemState.RETURN_HOME: 0.90,
            SystemState.COMPLETED: 1.0,
            SystemState.FAILED: 0.0,
        }

        return progress_by_state.get(
            self._fsm.current_state,
            0.0,
        )

    def destroy_node(self) -> None:
        """Destroy the action client and node."""
        self._navigation_client.destroy()
        self._load_tray_client.destroy()
        self._manipulation_client.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the task manager node."""
    rclpy.init(args=args)

    node: TaskManagerNode | None = None

    try:
        node = TaskManagerNode()
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
