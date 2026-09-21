"""Top-level task manager node for the shelving system."""

from pathlib import Path

import rclpy
from ament_index_python.packages import (
    get_package_share_directory,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from shelving_interfaces.action import (
    NavigateToTarget,
    PlaceBook,
)
from shelving_interfaces.msg import RobotStatus, TrayJob
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

        self.declare_parameter("navigation_action", "/navigate_to_target")

        self.declare_parameter('manipulation_action', '/place_book')

        self.declare_parameter(
            "navigation_server_timeout_sec",
            5.0,
        )
        self.declare_parameter(
            "position_tolerance",
            0.1,
        )
        self.declare_parameter(
            "yaw_tolerance",
            0.1,
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

        self._navigation_client = ActionClient(
            self,
            NavigateToTarget,
            navigation_action,
        )
        self._navigation_goal_handle = None

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

    def _send_navigation_to_return(self) -> None:
        """Send a navigation goal for the return station."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        self._send_navigation_goal(
            target_type="return_station",
            target_id="return_station",
            frame_id=str(self._current_plan.return_station_pose['frame_id']),
            target_pose=self._current_plan.return_station_pose,
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

        self._send_navigation_goal(
            target_type="shelf",
            target_id=task.shelf_id,
            frame_id=task.shelf_frame_id,
            target_pose=task.shelf_observation_pose,
        )

    def _send_navigation_home(self) -> None:
        """Send a navigation goal for the home position."""
        if self._current_plan is None:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message="Current job plan does not exist.",
            )
            return

        self._send_navigation_goal(
            target_type="home",
            target_id="home",
            frame_id=str(
                self._current_plan.home_pose["frame_id"]
            ),
            target_pose=self._current_plan.home_pose,
        )

    def _send_navigation_goal(
        self,
        target_type: str,
        target_id: str,
        frame_id:str,
        target_pose: dict,
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

        goal.position_tolerance = float(
            self.get_parameter(
                "position_tolerance"
            ).value
        )
        goal.yaw_tolerance = float(
            self.get_parameter(
                "yaw_tolerance"
            ).value
        )

        self._active_navigation_target_type = target_type
        self._active_navigation_target_id = target_id

        self.get_logger().info(
            "Sending navigation goal: "
            f"job_id={goal.job_id}, "
            f"target_type={goal.target_type}, "
            f"target_id={goal.target_id}, "
            f"x={goal.target_pose.pose.position.x:.2f}, "
            f"y={goal.target_pose.pose.position.y:.2f}"
        )

        send_goal_future = (
            self._navigation_client.send_goal_async(
                goal,
                feedback_callback=(
                    self._handle_navigation_feedback
                ),
            )
        )
        send_goal_future.add_done_callback(
            self._handle_navigation_goal_response
        )

    def _handle_navigation_goal_response(
        self,
        future,
    ) -> None:
        """Handle acceptance or rejection of a goal."""
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail_navigation(
                error_code=self.ERROR_NAVIGATION_RESULT,
                message=(
                    "Failed to send navigation goal: "
                    f"{error}"
                ),
            )
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
            self._handle_navigation_result
        )

    def _handle_navigation_feedback(
        self,
        feedback_message,
    ) -> None:
        """Handle navigation progress feedback."""
        feedback = feedback_message.feedback

        self._status_message = (
            f"Navigating to "
            f"'{self._active_navigation_target_id}': "
            f"phase={feedback.phase}, "
            f"remaining={feedback.remaining_distance:.2f} m"
        )
        self._publish_status()

        self.get_logger().info(
            "Navigation feedback: "
            f"target_id="
            f"{self._active_navigation_target_id}, "
            f"phase={feedback.phase}, "
            f"remaining_distance="
            f"{feedback.remaining_distance:.2f}, "
            f"position_error="
            f"{feedback.position_error:.2f}, "
            f"yaw_error={feedback.yaw_error:.2f}"
        )

    def _handle_navigation_result(
        self,
        future,
    ) -> None:
        """Handle the completed navigation result."""
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

        send_goal_future = (
            self._manipulation_client.send_goal_async(
                goal,
                feedback_callback=(
                    self._handle_manipulation_feedback
                ),
            )
        )
        send_goal_future.add_done_callback(
            self._handle_manipulation_goal_response
        )

    def _handle_manipulation_goal_response(
        self,
        future,
    ) -> None:
        """Handle acceptance of a placement goal."""
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail_manipulation(
                error_code=self.ERROR_MANIPULATION_RESULT,
                message=(
                    "Failed to send manipulation goal: "
                    f"{error}"
                ),
            )
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
            self._handle_manipulation_result
        )

    def _handle_manipulation_feedback(
        self,
        feedback_message,
    ) -> None:
        """Handle book-placement phase feedback."""
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
    ) -> None:
        """Handle the completed book-placement result."""
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

        self._send_navigation_home()

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