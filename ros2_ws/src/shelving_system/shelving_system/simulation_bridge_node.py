"""Translate Isaac simulation messages into shelving ROS interfaces."""

from __future__ import annotations

import json
import time
import uuid
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.action import(
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from shelving_interfaces.msg import ScenarioState
from std_msgs.msg import String

from rclpy.clock import Clock, ClockType
from rclpy.task import Future
from shelving_interfaces.action import LoadTray


class SimulationBridgeNode(Node):
    """Bridge standard Isaac messages to project-specific interfaces."""

    RAW_STATE_SCHEMA = "scenario_state/v1"

    STATE_VALUES = {
        "STOPPED": ScenarioState.STOPPED,
        "RESETTING": ScenarioState.RESETTING,
        "READY": ScenarioState.READY,
        "RUNNING": ScenarioState.RUNNING,
        "FAILED": ScenarioState.FAILED,
    }

    def __init__(self) -> None:
        super().__init__("simulation_bridge_node")

        self.declare_parameter(
            "raw_state_topic",
            "/simulation/scenario/state",
        )
        self.declare_parameter(
            "scenario_state_topic",
            "/scenario/state",
        )

        raw_state_topic = str(
            self.get_parameter(
                "raw_state_topic"
            ).value
        )
        scenario_state_topic = str(
            self.get_parameter(
                "scenario_state_topic"
            ).value
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._state_publisher = self.create_publisher(
            ScenarioState,
            scenario_state_topic,
            state_qos,
        )

        self._raw_state_subscription = (
            self.create_subscription(
                String,
                raw_state_topic,
                self._on_raw_state,
                state_qos,
            )
        )

        self.get_logger().info(
            "Simulation bridge ready: "
            f"{raw_state_topic} -> "
            f"{scenario_state_topic}"
        )

        self.declare_parameter(
            "load_tray_action",
            "/load_tray",
        )
        self.declare_parameter(
            "raw_tray_command_topic",
            "/simulation/tray/command",
        )
        self.declare_parameter(
            "raw_tray_state_topic",
            "/simulation/tray/state",
        )
        self.declare_parameter(
            "load_tray_timeout_sec",
            15.0,
        )

        self._raw_tray_command_topic = str(
            self.get_parameter(
                "raw_tray_command_topic"
            ).value
        )
        self._raw_tray_state_topic = str(
            self.get_parameter(
                "raw_tray_state_topic"
            ).value
        )

        self._tray_command_publisher = (
            self.create_publisher(
                String,
                self._raw_tray_command_topic,
                10,
            )
        )
        self._tray_state_subscription = (
            self.create_subscription(
                String,
                self._raw_tray_state_topic,
                self._on_raw_tray_state,
                10,
            )
        )

        self._load_tray_reserved = False
        self._active_load_goal = None
        self._active_load_command_id = ""
        self._load_completion_future = None
        self._load_deadline = 0.0

        self._load_tray_server = ActionServer(
            self,
            LoadTray,
            str(
                self.get_parameter(
                    "load_tray_action"
                ).value
            ),
            execute_callback=(
                self._execute_load_tray
            ),
            goal_callback=(
                self._handle_load_tray_goal
            ),
            cancel_callback=(
                self._handle_load_tray_cancel
            ),
        )

        self._system_clock = Clock(
            clock_type=ClockType.SYSTEM_TIME
        )
        self._load_timeout_timer = self.create_timer(
            0.5,
            self._check_load_tray_timeout,
            clock=self._system_clock,
        )

        self.get_logger().info(
            "LoadTray bridge ready: "
            f"action=/load_tray, "
            f"command={self._raw_tray_command_topic}, "
            f"state={self._raw_tray_state_topic}"
        )

    def _on_raw_state(self, message: String) -> None:
        try:
            payload = self._decode_state(message.data)
        except ValueError as error:
            self.get_logger().error(
                f"Rejected Isaac scenario state: {error}"
            )
            return

        state_name = payload["state"]
        run_id = payload["run_id"]
        state_message = payload["message"]

        ros_message = ScenarioState()
        ros_message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        ros_message.header.frame_id = ""
        ros_message.run_id = run_id
        ros_message.state = self.STATE_VALUES[state_name]
        ros_message.message = state_message

        self._state_publisher.publish(ros_message)

        self.get_logger().info(
            "Scenario state received: "
            f"state={state_name}, "
            f"run_id={run_id or '<none>'}, "
            f"message={state_message}"
        )

    def _handle_load_tray_goal(
        self,
        _goal_request,
    ) -> GoalResponse:
        if self._load_tray_reserved:
            return GoalResponse.REJECT

        self._load_tray_reserved = True
        return GoalResponse.ACCEPT

    def _handle_load_tray_cancel(
        self,
        goal_handle,
    ) -> CancelResponse:
        if goal_handle is not self._active_load_goal:
            return CancelResponse.REJECT

        self._publish_tray_command(
            command="CANCEL",
            command_id=self._active_load_command_id,
            job_id=goal_handle.request.job_id,
            tray_id=goal_handle.request.tray_id,
        )
        return CancelResponse.ACCEPT

    async def _execute_load_tray(
        self,
        goal_handle,
    ):
        command_id = str(uuid.uuid4())
        completion_future = Future()

        self._active_load_goal = goal_handle
        self._active_load_command_id = command_id
        self._load_completion_future = (
            completion_future
        )
        self._load_deadline = (
            time.monotonic()
            + float(
                self.get_parameter(
                    "load_tray_timeout_sec"
                ).value
            )
        )

        self._publish_tray_command(
            command="LOAD",
            command_id=command_id,
            job_id=goal_handle.request.job_id,
            tray_id=goal_handle.request.tray_id,
        )

        self.get_logger().info(
            "LoadTray command sent to Isaac: "
            f"command_id={command_id}, "
            f"job_id={goal_handle.request.job_id}, "
            f"tray_id={goal_handle.request.tray_id}"
        )

        try:
            return await completion_future
        finally:
            self._active_load_goal = None
            self._active_load_command_id = ""
            self._load_completion_future = None
            self._load_deadline = 0.0
            self._load_tray_reserved = False

    def _publish_tray_command(
        self,
        *,
        command: str,
        command_id: str,
        job_id: str,
        tray_id: str,
    ) -> None:
        payload = {
            "schema": "tray_command/v1",
            "command": command,
            "command_id": command_id,
            "job_id": job_id,
            "tray_id": tray_id,
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self._tray_command_publisher.publish(message)

    def _on_raw_tray_state(
        self,
        message: String,
    ) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(
                f"Invalid Isaac tray state: {error}"
            )
            return

        if not isinstance(payload, dict):
            return

        if payload.get("schema") != "tray_state/v1":
            return

        if (
            payload.get("command_id")
            != self._active_load_command_id
        ):
            return

        goal_handle = self._active_load_goal
        completion_future = (
            self._load_completion_future
        )

        if (
            goal_handle is None
            or completion_future is None
            or completion_future.done()
        ):
            return

        status = str(
            payload.get("status", "")
        ).upper()
        progress = float(
            payload.get("progress", 0.0)
        )
        state_message = str(
            payload.get("message", "")
        )

        feedback = LoadTray.Feedback()
        feedback.phase = status
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)

        if status == "MOVING":
            return

        result = LoadTray.Result()
        result.success = status == "SUCCEEDED"
        result.error_code = int(
            payload.get("error_code", 0)
        )
        result.message = state_message

        if status == "SUCCEEDED":
            goal_handle.succeed()
        elif status == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.abort()

        completion_future.set_result(result)

        self.get_logger().info(
            "LoadTray result received from Isaac: "
            f"status={status}, "
            f"message={state_message}"
        )

    def _check_load_tray_timeout(self) -> None:
        if (
            self._active_load_goal is None
            or self._load_completion_future is None
            or self._load_completion_future.done()
        ):
            return

        if time.monotonic() < self._load_deadline:
            return

        self._publish_tray_command(
            command="CANCEL",
            command_id=self._active_load_command_id,
            job_id=(
                self._active_load_goal.request.job_id
            ),
            tray_id=(
                self._active_load_goal.request.tray_id
            ),
        )

        result = LoadTray.Result()
        result.success = False
        result.error_code = 3003
        result.message = "Isaac tray loading timed out."

        self._active_load_goal.abort()
        self._load_completion_future.set_result(
            result
        )

    @classmethod
    def _decode_state(
        cls,
        raw_message: str,
    ) -> dict[str, str]:
        try:
            payload: Any = json.loads(raw_message)
        except json.JSONDecodeError as error:
            raise ValueError(
                f"invalid JSON: {error.msg}"
            ) from error

        if not isinstance(payload, dict):
            raise ValueError(
                "scenario state must be a JSON object"
            )

        schema = payload.get("schema")
        if schema != cls.RAW_STATE_SCHEMA:
            raise ValueError(
                f"unsupported schema: {schema!r}"
            )

        state = payload.get("state")
        if not isinstance(state, str):
            raise ValueError(
                "state must be a string"
            )

        if state not in cls.STATE_VALUES:
            raise ValueError(
                f"unknown state: {state!r}"
            )

        run_id = payload.get("run_id", "")
        if not isinstance(run_id, str):
            raise ValueError(
                "run_id must be a string"
            )

        if state in {"READY", "RUNNING"} and not run_id:
            raise ValueError(
                f"{state} requires a non-empty run_id"
            )

        message = payload.get("message", "")
        if not isinstance(message, str):
            raise ValueError(
                "message must be a string"
            )

        return {
            "run_id": run_id,
            "state": state,
            "message": message,
        }

    def destroy_node(self) -> None:
        self._load_tray_server.destroy()
        super().destroy_node()

def main(args=None) -> None:
    """Run the simulation bridge node."""
    rclpy.init(args=args)

    node = SimulationBridgeNode()

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