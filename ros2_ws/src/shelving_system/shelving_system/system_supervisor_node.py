"""Supervise the three-PC shelving system runtime."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from shelving_interfaces.action import (
    DetectGraspPoint,
    DetectTargetSlot,
    NavigateToTarget,
    PlaceBook,
)
from shelving_interfaces.msg import (
    RobotStatus,
    ScenarioState,
    SystemStatus,
)


class SystemSupervisorNode(Node):
    """Publish authoritative system state and gate cycle starts."""

    FRESHNESS_TIMEOUT_SEC = 3.0

    def __init__(self) -> None:
        super().__init__("system_supervisor_node")

        self.declare_parameter(
            "scenario_state_topic",
            "/scenario/state",
        )
        self.declare_parameter(
            "task_status_topic",
            "/system/state",
        )
        self.declare_parameter(
            "system_status_topic",
            "/system/status",
        )

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_publisher = self.create_publisher(
            SystemStatus,
            str(
                self.get_parameter(
                    "system_status_topic"
                ).value
            ),
            status_qos,
        )

        self._scenario_state = ScenarioState.UNKNOWN
        self._scenario_run_id = ""
        self._scenario_message = ""

        self._task_status: RobotStatus | None = None
        self._task_status_received_at = 0.0

        self._scan_received_at = 0.0
        self._amcl_received_at = 0.0

        self._bt_navigator_active = False
        self._bt_state_request = None

        self._start_request_in_flight = False
        self._last_start_error = ""

        self.create_subscription(
            ScenarioState,
            str(
                self.get_parameter(
                    "scenario_state_topic"
                ).value
            ),
            self._on_scenario_state,
            status_qos,
        )

        self.create_subscription(
            RobotStatus,
            str(
                self.get_parameter(
                    "task_status_topic"
                ).value
            ),
            self._on_task_status,
            10,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
        )

        self._navigation_client = ActionClient(
            self,
            NavigateToTarget,
            "/navigate_to_target",
        )
        self._manipulation_client = ActionClient(
            self,
            PlaceBook,
            "/place_book",
        )
        self._grasp_client = ActionClient(
            self,
            DetectGraspPoint,
            "/detect_grasp_point",
        )
        self._slot_client = ActionClient(
            self,
            DetectTargetSlot,
            "/detect_target_slot",
        )

        self._return_machine_client = self.create_client(
            Trigger,
            "/return_machine/publish_job",
        )

        self._bt_state_client = self.create_client(
            GetState,
            "/bt_navigator/get_state",
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )

        self._start_service = self.create_service(
            Trigger,
            "/system/start_cycle",
            self._handle_start_cycle,
        )

        self._latest_status = SystemStatus()
        self._status_timer = self.create_timer(
            0.5,
            self._update_and_publish_status,
        )
        self._lifecycle_timer = self.create_timer(
            1.0,
            self._request_bt_navigator_state,
        )

        self.get_logger().info(
            "System supervisor ready. "
            "Start service: /system/start_cycle"
        )

    def _on_scenario_state(
        self,
        message: ScenarioState,
    ) -> None:
        self._scenario_state = message.state
        self._scenario_run_id = message.run_id
        self._scenario_message = message.message

        if message.state in (
            ScenarioState.STOPPED,
            ScenarioState.RESETTING,
        ):
            # 다음 Play에서는 새 initial pose에 대한 AMCL 결과를
            # 다시 받아야 한다.
            self._amcl_received_at = 0.0

        if message.state in (
            ScenarioState.STOPPED,
            ScenarioState.RESETTING,
            ScenarioState.FAILED,
        ):
            self._start_request_in_flight = False

    def _on_task_status(
        self,
        message: RobotStatus,
    ) -> None:
        self._task_status = message
        self._task_status_received_at = time.monotonic()

        if message.state != "IDLE":
            self._start_request_in_flight = False

    def _on_scan(self, _message: LaserScan) -> None:
        self._scan_received_at = time.monotonic()

    def _on_amcl_pose(
        self,
        _message: PoseWithCovarianceStamped,
    ) -> None:
        self._amcl_received_at = time.monotonic()

    def _is_fresh(self, received_at: float) -> bool:
        if received_at <= 0.0:
            return False

        return (
            time.monotonic() - received_at
            <= self.FRESHNESS_TIMEOUT_SEC
        )

    def _request_bt_navigator_state(self) -> None:
        if self._bt_state_request is not None:
            return

        if not self._bt_state_client.service_is_ready():
            self._bt_navigator_active = False
            return

        self._bt_state_request = (
            self._bt_state_client.call_async(
                GetState.Request()
            )
        )
        self._bt_state_request.add_done_callback(
            self._on_bt_navigator_state
        )

    def _on_bt_navigator_state(self, future) -> None:
        self._bt_state_request = None

        try:
            response = future.result()
        except Exception as error:
            self._bt_navigator_active = False
            self.get_logger().warning(
                f"Failed to read bt_navigator state: {error}"
            )
            return

        self._bt_navigator_active = (
            response.current_state.id
            == LifecycleState.PRIMARY_STATE_ACTIVE
        )

    def _robot_tf_is_ready(self) -> bool:
        try:
            return self._tf_buffer.can_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return False

    def _calculate_status(self) -> SystemStatus:
        message = SystemStatus()
        now = self.get_clock().now().to_msg()

        message.header.stamp = now
        message.heartbeat_time = now
        message.run_id = self._scenario_run_id

        simulation_ready = self._scenario_state in (
            ScenarioState.READY,
            ScenarioState.RUNNING,
        )

        task_status_fresh = self._is_fresh(
            self._task_status_received_at
        )
        task_idle = (
            task_status_fresh
            and self._task_status is not None
            and self._task_status.state == "IDLE"
        )

        navigation_ready = (
            self._navigation_client.server_is_ready()
            and self._bt_navigator_active
        )
        manipulation_ready = (
            self._manipulation_client.server_is_ready()
        )
        perception_ready = (
            self._grasp_client.server_is_ready()
            and self._slot_client.server_is_ready()
        )

        scan_ready = self._is_fresh(
            self._scan_received_at
        )
        # AMCL pose는 heartbeat가 아니다. 현재 run에서 한 번이라도
        # 새 위치를 받았다면 localization 준비가 완료된 것이다.
        amcl_ready = self._amcl_received_at > 0.0
        tf_ready = self._robot_tf_is_ready()

        pc_a_ready = (
            simulation_ready
            and task_status_fresh
            and self._return_machine_client.service_is_ready()
        )
        pc_b_ready = (
            navigation_ready
            and manipulation_ready
            and perception_ready
            and scan_ready
            and amcl_ready
            and tf_ready
        )

        message.simulation_ready = simulation_ready
        message.pc_a_ready = pc_a_ready
        message.pc_b_ready = pc_b_ready
        message.navigation_ready = navigation_ready
        message.manipulation_ready = manipulation_ready
        message.perception_ready = perception_ready

        if self._task_status is not None:
            message.phase = self._task_status.state
            message.active_job_id = (
                self._task_status.active_job_id
            )
            message.progress = self._task_status.progress
            message.error_code = (
                self._task_status.error_code
            )
        else:
            message.phase = "INITIALIZING"
            message.active_job_id = ""
            message.progress = 0.0
            message.error_code = 0

        if self._scenario_state == ScenarioState.FAILED:
            message.state = SystemStatus.ERROR
            message.message = (
                self._scenario_message
                or "Isaac scenario failed."
            )
        elif (
            task_status_fresh
            and self._task_status is not None
            and self._task_status.state == "FAILED"
        ):
            message.state = SystemStatus.ERROR
            message.message = self._task_status.message
        elif self._scenario_state == ScenarioState.STOPPED:
            message.state = SystemStatus.STOPPED
            message.message = "Isaac Timeline is stopped."
        elif self._scenario_state == ScenarioState.RESETTING:
            message.state = SystemStatus.RESETTING
            message.message = "System reset is in progress."
        elif not simulation_ready:
            message.state = SystemStatus.WAITING_FOR_SIM
            message.message = "Waiting for Isaac Sim."
        elif (
            self._start_request_in_flight
            or (
                task_status_fresh
                and self._task_status is not None
                and self._task_status.state != "IDLE"
            )
        ):
            message.state = SystemStatus.RUNNING
            message.message = (
                self._task_status.message
                if self._task_status is not None
                else "Starting a new cycle."
            )
        elif not pc_a_ready or not task_idle:
            missing = []

            if not task_status_fresh:
                missing.append("task_manager heartbeat")
            elif not task_idle:
                missing.append(
                    "task_manager state="
                    + self._task_status.state
                )

            if not self._return_machine_client.service_is_ready():
                missing.append("return_machine service")

            message.state = SystemStatus.BOOTING
            message.message = (
                "Waiting for PC A: " + ", ".join(missing)
            )
        elif not pc_b_ready:
            missing = []

            if not navigation_ready:
                missing.append("navigation")
            if not manipulation_ready:
                missing.append("manipulation")
            if not perception_ready:
                missing.append("perception")
            if not scan_ready:
                missing.append("scan")
            if not amcl_ready:
                missing.append("amcl")
            if not tf_ready:
                missing.append("map->base_link TF")

            message.state = SystemStatus.WAITING_FOR_PC_B
            message.message = (
                "Waiting for PC B: " + ", ".join(missing)
            )
        else:
            message.state = SystemStatus.READY
            message.phase = "IDLE"
            message.message = "System is ready for a new cycle."

        if self._last_start_error:
            message.state = SystemStatus.ERROR
            message.message = self._last_start_error

        return message

    def _update_and_publish_status(self) -> None:
        self._latest_status = self._calculate_status()
        self._status_publisher.publish(
            self._latest_status
        )

    def _handle_start_cycle(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # 이전 요청의 일시적인 오류는 새 요청에서 다시 평가한다.
        self._last_start_error = ""
        self._latest_status = self._calculate_status()

        if self._latest_status.state != SystemStatus.READY:
            response.success = False
            response.message = (
                "Start rejected: "
                + self._latest_status.message
            )
            return response

        if not self._return_machine_client.service_is_ready():
            response.success = False
            response.message = (
                "Start rejected: return machine is unavailable."
            )
            return response

        self._start_request_in_flight = True

        future = self._return_machine_client.call_async(
            Trigger.Request()
        )
        future.add_done_callback(
            self._on_return_machine_response
        )

        response.success = True
        response.message = "Cycle start request accepted."
        return response

    def _on_return_machine_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._start_request_in_flight = False
            self._last_start_error = (
                f"Return machine request failed: {error}"
            )
            return

        if not response.success:
            self._start_request_in_flight = False
            self._last_start_error = (
                "Return machine rejected the request: "
                + response.message
            )
            return

        self.get_logger().info(response.message)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SystemSupervisorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
