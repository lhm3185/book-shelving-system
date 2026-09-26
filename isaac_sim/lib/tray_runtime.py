"""Return-machine tray transfer and reset runtime."""

from __future__ import annotations

import json

import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.rotations import (
    quat_to_rot_matrix,
    rot_matrix_to_quat,
)
from pxr import Gf, UsdPhysics
from std_msgs.msg import String


def _pose_matrix(
    position: np.ndarray,
    orientation: np.ndarray,
) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quat_to_rot_matrix(orientation)
    matrix[:3, 3] = position
    return matrix


def _matrix_pose(
    matrix: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    position = np.asarray(
        matrix[:3, 3],
        dtype=float,
    )
    orientation = rot_matrix_to_quat(
        np.asarray(matrix[:3, :3], dtype=float)
    )
    return position, orientation


def _interpolate_quaternion(
    start: np.ndarray,
    finish: np.ndarray,
    progress: float,
) -> np.ndarray:
    start = np.asarray(start, dtype=float)
    finish = np.asarray(finish, dtype=float)

    if float(np.dot(start, finish)) < 0.0:
        finish = -finish

    orientation = (
        (1.0 - progress) * start
        + progress * finish
    )
    norm = float(np.linalg.norm(orientation))

    if norm < 1.0e-9:
        return start.copy()

    return orientation / norm


class TrayRuntime:
    """Move one tray group onto Ridgeback and restore it on reset."""

    COMMAND_SCHEMA = "tray_command/v1"
    STATE_SCHEMA = "tray_state/v1"

    def __init__(
        self,
        *,
        node,
        stage,
        timeline,
        config: dict,
    ) -> None:
        self._node = node
        self._stage = stage
        self._timeline = timeline

        self._group_path = str(
            config["group_prim_path"]
        )
        self._tray_path = str(
            config["tray_prim_path"]
        )
        self._carrier_path = str(
            config["carrier_prim_path"]
        )

        self._command_topic = str(
            config.get(
                "command_topic",
                "/simulation/tray/command",
            )
        )
        self._state_topic = str(
            config.get(
                "state_topic",
                "/simulation/tray/state",
            )
        )
        self._duration = float(
            config.get("transfer_duration_sec", 3.0)
        )

        self._handoff_position_base = np.asarray(
            config["handoff_position_base"],
            dtype=float,
        )
        self._follow_translation_threshold = float(
            config.get(
                "follow_translation_threshold_m",
                0.002,
            )
        )
        self._follow_rotation_threshold = float(
            config.get(
                "follow_rotation_threshold_rad",
                0.002,
            )
        )

        if self._handoff_position_base.shape != (3,):
            raise ValueError(
                "tray.handoff_position_base must contain 3 values"
            )

        if self._follow_translation_threshold < 0.0:
            raise ValueError(
                "tray.follow_translation_threshold_m "
                "must not be negative"
            )

        if self._follow_rotation_threshold < 0.0:
            raise ValueError(
                "tray.follow_rotation_threshold_rad "
                "must not be negative"
            )

        if self._duration <= 0.0:
            raise ValueError(
                "tray.transfer_duration_sec must be positive"
            )

        group_prim = stage.GetPrimAtPath(
            self._group_path
        )
        tray_prim = stage.GetPrimAtPath(
            self._tray_path
        )
        carrier_prim = stage.GetPrimAtPath(
            self._carrier_path
        )

        if not group_prim.IsValid():
            raise RuntimeError(
                f"Tray group does not exist: {self._group_path}"
            )

        if not tray_prim.IsValid():
            raise RuntimeError(
                f"Tray prim does not exist: {self._tray_path}"
            )

        if not carrier_prim.IsValid():
            raise RuntimeError(
                f"Tray carrier does not exist: "
                f"{self._carrier_path}"
            )

        self._objects: dict[str, SingleXFormPrim] = {}
        self._rigid_apis = {}
        self._initial_states = {}

        for child in group_prim.GetChildren():
            if not child.HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                continue

            path = str(child.GetPath())
            rigid_api = UsdPhysics.RigidBodyAPI(child)
            xform = SingleXFormPrim(path)

            position, orientation = (
                xform.get_world_pose()
            )

            kinematic_value = (
                rigid_api
                .GetKinematicEnabledAttr()
                .Get()
            )

            self._objects[path] = xform
            self._rigid_apis[path] = rigid_api
            self._initial_states[path] = (
                np.asarray(position, dtype=float).copy(),
                np.asarray(orientation, dtype=float).copy(),
                bool(kinematic_value),
            )

        if self._tray_path not in self._objects:
            raise RuntimeError(
                "Configured tray prim is not a top-level "
                "rigid body in the tray group"
            )

        if len(self._objects) < 2:
            raise RuntimeError(
                "Tray group must contain the tray and books"
            )

        self._book_paths = tuple(
            path
            for path in self._objects
            if path != self._tray_path
        )

        self._carrier = SingleXFormPrim(
            self._carrier_path
        )

        self._relative_transforms = {}
        self._source_position = None
        self._source_orientation = None
        self._start_time = 0.0

        self._active_command_id = ""
        self._active_job_id = ""
        self._active_tray_id = ""

        self._loaded = False
        self._loaded_relative_transform = None
        self._last_feedback_progress = -1.0

        self._state_publisher = (
            node.create_publisher(
                String,
                self._state_topic,
                10,
            )
        )
        self._command_subscription = (
            node.create_subscription(
                String,
                self._command_topic,
                self._on_command,
                10,
            )
        )

        self._node.get_logger().info(
            "Tray runtime ready: "
            f"objects={len(self._objects)}, "
            f"tray={self._tray_path}, "
            f"carrier={self._carrier_path}"
        )

    def _capture_relative_transforms(
        self,
    ) -> dict[str, np.ndarray]:
        tray_position, tray_orientation = (
            self._objects[
                self._tray_path
            ].get_world_pose()
        )

        tray_matrix = _pose_matrix(
            np.asarray(tray_position, dtype=float),
            np.asarray(tray_orientation, dtype=float),
        )
        inverse_tray = np.linalg.inv(tray_matrix)

        relative_transforms = {}

        for path, xform in self._objects.items():
            position, orientation = (
                xform.get_world_pose()
            )
            object_matrix = _pose_matrix(
                np.asarray(position, dtype=float),
                np.asarray(orientation, dtype=float),
            )
            relative_transforms[path] = (
                inverse_tray @ object_matrix
            )

        return relative_transforms

    def _carrier_matrix(self) -> np.ndarray:
        carrier_position, carrier_orientation = (
            self._carrier.get_world_pose()
        )
        return _pose_matrix(
            np.asarray(carrier_position, dtype=float),
            np.asarray(carrier_orientation, dtype=float),
        )

    def _capture_loaded_relative_transform(
        self,
    ) -> None:
        tray_position, tray_orientation = (
            self._objects[
                self._tray_path
            ].get_world_pose()
        )
        tray_matrix = _pose_matrix(
            np.asarray(tray_position, dtype=float),
            np.asarray(tray_orientation, dtype=float),
        )
        self._loaded_relative_transform = (
            np.linalg.inv(self._carrier_matrix())
            @ tray_matrix
        )

    def _target_handoff_position(self) -> np.ndarray:
        carrier_matrix = self._carrier_matrix()
        return (
            carrier_matrix[:3, 3]
            + carrier_matrix[:3, :3]
            @ self._handoff_position_base
        )

    def _target_loaded_tray_pose(
        self,
    ) -> tuple[np.ndarray, np.ndarray]:
        if self._loaded_relative_transform is None:
            raise RuntimeError(
                "Loaded tray transform has not been captured"
            )

        return _matrix_pose(
            self._carrier_matrix()
            @ self._loaded_relative_transform
        )

    def _set_group_pose(
        self,
        tray_position: np.ndarray,
        tray_orientation: np.ndarray,
    ) -> None:
        tray_matrix = _pose_matrix(
            tray_position,
            tray_orientation,
        )

        for path, relative in (
            self._relative_transforms.items()
        ):
            object_matrix = tray_matrix @ relative
            position, orientation = _matrix_pose(
                object_matrix
            )
            self._objects[path].set_world_pose(
                position,
                orientation,
            )

    def _set_group_kinematic(
        self,
        enabled: bool,
    ) -> None:
        for rigid_api in self._rigid_apis.values():
            rigid_api.CreateKinematicEnabledAttr().Set(
                bool(enabled)
            )

    def _zero_velocities(self) -> None:
        zero = Gf.Vec3f(0.0, 0.0, 0.0)

        for rigid_api in self._rigid_apis.values():
            rigid_api.CreateVelocityAttr().Set(zero)
            rigid_api.CreateAngularVelocityAttr().Set(
                zero
            )

    def _restore_initial_state(self) -> None:
        self._set_group_kinematic(True)
        self._zero_velocities()

        for path, state in (
            self._initial_states.items()
        ):
            position, orientation, _ = state
            self._objects[path].set_world_pose(
                position,
                orientation,
            )

        for path, state in (
            self._initial_states.items()
        ):
            _, _, was_kinematic = state
            self._rigid_apis[
                path
            ].CreateKinematicEnabledAttr().Set(
                was_kinematic
            )

        self._zero_velocities()
        self._loaded = False
        self._loaded_relative_transform = None

    def _on_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self._node.get_logger().error(
                f"Invalid tray command JSON: {error}"
            )
            return

        if not isinstance(payload, dict):
            return

        if payload.get("schema") != self.COMMAND_SCHEMA:
            return

        command = str(
            payload.get("command", "")
        ).upper()
        command_id = str(
            payload.get("command_id", "")
        )
        job_id = str(payload.get("job_id", ""))
        tray_id = str(payload.get("tray_id", ""))

        if not command_id:
            return

        if command == "CANCEL":
            if command_id != self._active_command_id:
                return

            self._restore_initial_state()
            self._publish_state(
                status="CANCELED",
                progress=0.0,
                message="Tray loading was canceled.",
                error_code=3005,
            )
            self._clear_active_command()
            return

        if command != "LOAD":
            self._publish_state(
                status="FAILED",
                progress=0.0,
                message=f"Unknown tray command: {command}",
                error_code=3004,
                command_id=command_id,
                job_id=job_id,
                tray_id=tray_id,
            )
            return

        if self._active_command_id:
            self._publish_state(
                status="FAILED",
                progress=0.0,
                message="Another tray transfer is active.",
                error_code=3002,
                command_id=command_id,
                job_id=job_id,
                tray_id=tray_id,
            )
            return

        if self._loaded:
            self._publish_state(
                status="SUCCEEDED",
                progress=1.0,
                message="Tray is already loaded.",
                error_code=0,
                command_id=command_id,
                job_id=job_id,
                tray_id=tray_id,
            )
            return

        self._active_command_id = command_id
        self._active_job_id = job_id
        self._active_tray_id = tray_id

        source_position, source_orientation = (
            self._objects[
                self._tray_path
            ].get_world_pose()
        )

        self._source_position = np.asarray(
            source_position,
            dtype=float,
        )
        self._source_orientation = np.asarray(
            source_orientation,
            dtype=float,
        )
        self._relative_transforms = (
            self._capture_relative_transforms()
        )

        self._set_group_kinematic(True)
        self._zero_velocities()

        self._start_time = float(
            self._timeline.get_current_time()
        )
        self._last_feedback_progress = -1.0

        self._publish_state(
            status="MOVING",
            progress=0.0,
            message="Tray transfer started.",
            error_code=0,
        )

        self._node.get_logger().info(
            "Tray transfer started: "
            f"job_id={job_id}, tray_id={tray_id}"
        )

    def update(self) -> None:
        if self._active_command_id:
            current_time = float(
                self._timeline.get_current_time()
            )
            elapsed = max(
                0.0,
                current_time - self._start_time,
            )
            progress = min(
                elapsed / self._duration,
                1.0,
            )

            target_position = self._target_handoff_position()
            target_orientation = self._source_orientation

            # 시작과 끝에서 속도가 0이 되는 smoothstep으로
            # 트레이와 책을 한 rigid group처럼 이동시킨다.
            smooth_progress = (
                progress * progress
                * (3.0 - 2.0 * progress)
            )

            position = (
                (1.0 - smooth_progress)
                * self._source_position
                + smooth_progress * target_position
            )
            orientation = _interpolate_quaternion(
                self._source_orientation,
                target_orientation,
                smooth_progress,
            )

            self._set_group_pose(
                position,
                orientation,
            )

            if progress >= 1.0:
                self._set_group_pose(
                    target_position,
                    target_orientation,
                )
                self._capture_loaded_relative_transform()
                self._loaded = True
                self._publish_state(
                    status="SUCCEEDED",
                    progress=1.0,
                    message=(
                        "Tray and books are loaded "
                        "and following Ridgeback."
                    ),
                    error_code=0,
                )

                self._node.get_logger().info(
                    "Tray loading completed with "
                    "kinematic cargo following."
                )
                self._clear_active_command()
                return

            if (
                progress
                - self._last_feedback_progress
                >= 0.02
            ):
                self._last_feedback_progress = progress
                self._publish_state(
                    status="MOVING",
                    progress=progress,
                    message=(
                        "Tray is moving onto Ridgeback."
                    ),
                    error_code=0,
                )

            return

        if (
            not self._loaded
            or self._loaded_relative_transform is None
        ):
            return

        target_position, target_orientation = (
            self._target_loaded_tray_pose()
        )
        current_position, current_orientation = (
            self._objects[
                self._tray_path
            ].get_world_pose()
        )

        translation_error = float(
            np.linalg.norm(
                target_position
                - np.asarray(current_position, dtype=float)
            )
        )
        current_orientation = np.asarray(
            current_orientation,
            dtype=float,
        )
        orientation_dot = abs(
            float(
                np.dot(
                    target_orientation,
                    current_orientation,
                )
            )
        )
        rotation_error = 2.0 * float(
            np.arccos(np.clip(orientation_dot, 0.0, 1.0))
        )

        if (
            translation_error
            < self._follow_translation_threshold
            and rotation_error
            < self._follow_rotation_threshold
        ):
            return

        self._set_group_pose(
            target_position,
            target_orientation,
        )
        self._zero_velocities()

    def reset(self) -> None:
        if self._active_command_id:
            self._publish_state(
                status="FAILED",
                progress=0.0,
                message=(
                    "Tray loading was interrupted "
                    "by scenario reset."
                ),
                error_code=3006,
            )

        self._clear_active_command()
        self._restore_initial_state()

        self._node.get_logger().info(
            "Tray and tray books restored."
        )

    def _clear_active_command(self) -> None:
        self._active_command_id = ""
        self._active_job_id = ""
        self._active_tray_id = ""
        self._source_position = None
        self._source_orientation = None
        self._last_feedback_progress = -1.0

    def _publish_state(
        self,
        *,
        status: str,
        progress: float,
        message: str,
        error_code: int,
        command_id: str | None = None,
        job_id: str | None = None,
        tray_id: str | None = None,
    ) -> None:
        payload = {
            "schema": self.STATE_SCHEMA,
            "command_id": (
                self._active_command_id
                if command_id is None
                else command_id
            ),
            "job_id": (
                self._active_job_id
                if job_id is None
                else job_id
            ),
            "tray_id": (
                self._active_tray_id
                if tray_id is None
                else tray_id
            ),
            "status": status,
            "progress": float(progress),
            "error_code": int(error_code),
            "message": message,
        }

        ros_message = String()
        ros_message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self._state_publisher.publish(ros_message)

    def close(self) -> None:
        self._node.destroy_subscription(
            self._command_subscription
        )
        self._node.destroy_publisher(
            self._state_publisher
        )
