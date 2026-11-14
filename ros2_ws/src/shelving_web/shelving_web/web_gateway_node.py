"""ROS 2 to HTTP/WebSocket gateway for the operator dashboard."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path
import threading
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from shelving_interfaces.msg import SystemStatus
from std_srvs.srv import Trigger
import uvicorn
import yaml


class WebGatewayNode(Node):
    """Receive shelving ROS state and expose operator commands."""

    STATE_NAMES = {
        SystemStatus.BOOTING: "BOOTING",
        SystemStatus.WAITING_FOR_SIM: "WAITING_FOR_SIM",
        SystemStatus.WAITING_FOR_PC_B: "WAITING_FOR_PC_B",
        SystemStatus.READY: "READY",
        SystemStatus.RUNNING: "RUNNING",
        SystemStatus.STOPPING: "STOPPING",
        SystemStatus.STOPPED: "STOPPED",
        SystemStatus.RESETTING: "RESETTING",
        SystemStatus.ERROR: "ERROR",
        SystemStatus.EMERGENCY_STOPPED: "EMERGENCY_STOPPED",
    }

    def __init__(self) -> None:
        super().__init__("web_gateway_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter(
            "system_status_topic",
            "/system/status",
        )
        self.declare_parameter(
            "robot_pose_topic",
            "/amcl_pose",
        )
        self.declare_parameter(
            "start_cycle_service",
            "/system/start_cycle",
        )

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._state_lock = threading.Lock()
        self._start_lock = threading.Lock()

        self._latest_status: dict[str, Any] | None = None
        self._latest_pose: dict[str, Any] | None = None
        self._status_received_at = 0.0
        self._pose_received_at = 0.0

        self._status_subscription = self.create_subscription(
            SystemStatus,
            str(
                self.get_parameter(
                    "system_status_topic"
                ).value
            ),
            self._on_system_status,
            status_qos,
        )

        self._pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(
                self.get_parameter(
                    "robot_pose_topic"
                ).value
            ),
            self._on_robot_pose,
            10,
        )

        self._start_client = self.create_client(
            Trigger,
            str(
                self.get_parameter(
                    "start_cycle_service"
                ).value
            ),
        )

        self.get_logger().info(
            "Web gateway ROS interface is ready."
        )

    @staticmethod
    def _stamp_to_dict(stamp) -> dict[str, int]:
        return {
            "sec": int(stamp.sec),
            "nanosec": int(stamp.nanosec),
        }

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _on_system_status(
        self,
        message: SystemStatus,
    ) -> None:
        status = {
            "run_id": message.run_id,
            "state": int(message.state),
            "state_name": self.STATE_NAMES.get(
                int(message.state),
                "UNKNOWN",
            ),
            "phase": message.phase,
            "active_job_id": message.active_job_id,
            "progress": float(message.progress),
            "simulation_ready": bool(
                message.simulation_ready
            ),
            "pc_a_ready": bool(message.pc_a_ready),
            "pc_b_ready": bool(message.pc_b_ready),
            "navigation_ready": bool(
                message.navigation_ready
            ),
            "manipulation_ready": bool(
                message.manipulation_ready
            ),
            "perception_ready": bool(
                message.perception_ready
            ),
            "error_code": int(message.error_code),
            "message": message.message,
            "heartbeat_time": self._stamp_to_dict(
                message.heartbeat_time
            ),
        }

        with self._state_lock:
            self._latest_status = status
            self._status_received_at = time.monotonic()

    def _on_robot_pose(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        pose = message.pose.pose
        orientation = pose.orientation

        robot_pose = {
            "frame_id": message.header.frame_id,
            "stamp": self._stamp_to_dict(
                message.header.stamp
            ),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": self._yaw_from_quaternion(
                orientation
            ),
        }

        with self._state_lock:
            self._latest_pose = robot_pose
            self._pose_received_at = time.monotonic()

    def get_snapshot(self) -> dict[str, Any]:
        """Return a thread-safe dashboard snapshot."""
        now = time.monotonic()

        with self._state_lock:
            status = (
                dict(self._latest_status)
                if self._latest_status is not None
                else None
            )
            pose = (
                dict(self._latest_pose)
                if self._latest_pose is not None
                else None
            )
            status_received_at = self._status_received_at
            pose_received_at = self._pose_received_at

        status_age = (
            now - status_received_at
            if status_received_at > 0.0
            else None
        )
        pose_age = (
            now - pose_received_at
            if pose_received_at > 0.0
            else None
        )

        return {
            "gateway_time": time.time(),
            "ros": {
                "status_connected": (
                    status_age is not None
                    and status_age <= 3.0
                ),
                "pose_connected": pose_age is not None,
                "status_age_sec": status_age,
                "pose_age_sec": pose_age,
                "start_service_ready": (
                    self._start_client.service_is_ready()
                ),
            },
            "status": status,
            "pose": pose,
        }

    def request_start_cycle(
        self,
        timeout_sec: float = 5.0,
    ) -> dict[str, Any]:
        """Call the operator-gated ROS start service."""
        if not self._start_lock.acquire(blocking=False):
            return {
                "ok": False,
                "message": "A start request is already pending.",
                "http_status": 409,
            }

        try:
            if not self._start_client.wait_for_service(
                timeout_sec=1.0
            ):
                return {
                    "ok": False,
                    "message": (
                        "ROS service /system/start_cycle "
                        "is unavailable."
                    ),
                    "http_status": 503,
                }

            future = self._start_client.call_async(
                Trigger.Request()
            )
            completed = threading.Event()
            result_holder: dict[str, Any] = {}

            def on_complete(done_future) -> None:
                try:
                    response = done_future.result()
                    result_holder["response"] = response
                except Exception as error:
                    result_holder["error"] = str(error)
                finally:
                    completed.set()

            future.add_done_callback(on_complete)

            if not completed.wait(timeout=timeout_sec):
                return {
                    "ok": False,
                    "message": (
                        "Timed out waiting for the ROS "
                        "start response."
                    ),
                    "http_status": 504,
                }

            if "error" in result_holder:
                return {
                    "ok": False,
                    "message": result_holder["error"],
                    "http_status": 500,
                }

            response = result_holder["response"]

            self.get_logger().info(
                "Dashboard start request: "
                f"success={response.success}, "
                f"message={response.message}"
            )

            return {
                "ok": bool(response.success),
                "message": response.message,
                "http_status": (
                    200 if response.success else 409
                ),
            }
        finally:
            self._start_lock.release()


def load_map_metadata() -> tuple[Path, dict[str, Any]]:
    """Load the same static map used by Nav2."""
    navigation_share = Path(
        get_package_share_directory(
            "shelving_navigation"
        )
    )
    map_yaml_path = (
        navigation_share
        / "maps"
        / "library_map.yaml"
    )

    with map_yaml_path.open(
        "r",
        encoding="utf-8",
    ) as map_file:
        metadata = yaml.safe_load(map_file)

    map_image_path = (
        map_yaml_path.parent
        / str(metadata["image"])
    )

    return map_image_path, {
        "image_url": "/map.png",
        "resolution": float(metadata["resolution"]),
        "origin": [
            float(value)
            for value in metadata["origin"]
        ],
        "negate": int(metadata["negate"]),
        "occupied_thresh": float(
            metadata["occupied_thresh"]
        ),
        "free_thresh": float(
            metadata["free_thresh"]
        ),
    }


def create_web_application(
    node: WebGatewayNode,
) -> FastAPI:
    """Create the operator dashboard HTTP application."""
    web_share = Path(
        get_package_share_directory("shelving_web")
    )
    static_directory = web_share / "static"
    map_image_path, map_metadata = load_map_metadata()

    application = FastAPI(
        title="Book Shelving Control",
        docs_url="/api/docs",
        redoc_url=None,
    )

    @application.get(
        "/",
        include_in_schema=False,
    )
    async def index():
        return FileResponse(
            static_directory / "index.html"
        )

    @application.get(
        "/app.js",
        include_in_schema=False,
    )
    async def javascript():
        return FileResponse(
            static_directory / "app.js",
            media_type="application/javascript",
        )

    @application.get(
        "/styles.css",
        include_in_schema=False,
    )
    async def stylesheet():
        return FileResponse(
            static_directory / "styles.css",
            media_type="text/css",
        )

    @application.get(
        "/map.png",
        include_in_schema=False,
    )
    async def map_image():
        return FileResponse(
            map_image_path,
            media_type="image/png",
        )

    @application.get("/api/map")
    async def map_information():
        return map_metadata

    @application.get("/api/snapshot")
    async def snapshot():
        return node.get_snapshot()

    @application.get("/api/health")
    async def health():
        snapshot_data = node.get_snapshot()

        return {
            "ok": bool(
                snapshot_data["ros"][
                    "status_connected"
                ]
            ),
            "ros": snapshot_data["ros"],
        }

    @application.post("/api/start")
    async def start_cycle():
        result = await asyncio.to_thread(
            node.request_start_cycle
        )
        http_status = int(
            result.pop("http_status")
        )

        return JSONResponse(
            content=result,
            status_code=http_status,
        )

    @application.websocket("/ws")
    async def dashboard_socket(
        websocket: WebSocket,
    ):
        await websocket.accept()

        try:
            while True:
                await websocket.send_json(
                    node.get_snapshot()
                )
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

    return application


def main(args=None) -> None:
    """Run ROS in a worker thread and HTTP in the main thread."""
    rclpy.init(args=args)

    node = WebGatewayNode()
    executor = MultiThreadedExecutor(
        num_threads=2
    )
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin,
        name="shelving-web-ros",
        daemon=True,
    )
    ros_thread.start()

    host = str(
        node.get_parameter("host").value
    )
    port = int(
        node.get_parameter("port").value
    )

    application = create_web_application(node)

    try:
        uvicorn.run(
            application,
            host=host,
            port=port,
            log_level="info",
        )
    finally:
        executor.shutdown()
        ros_thread.join(timeout=2.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()