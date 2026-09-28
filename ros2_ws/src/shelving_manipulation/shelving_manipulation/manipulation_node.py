"""
PlaceBook 액션 서버.

    FSM --PlaceBook--> manipulation_node --작업 명령--> 작업 실행기(Isaac 또는 mock)
                                         <--진행 상태--

executor 파라미터
- mock : 노드 안에서 MockSimExecutor 로 흉내 낸다 (Isaac 없이 FSM 연동 시험)
- sim  : sim_command_topic 으로 JSON 명령을 보내고 sim_state_topic 의 JSON 상태를 따른다.
         Isaac 쪽 실행기, 또는 시험용 sim_mock 노드가 받는다

취소: 시뮬에 취소를 보내고, 시뮬이 안전 동작을 마쳤다고 알릴 때까지 기다린 뒤 결과를 돌려준다 (03 문서 2절).
"""

import math
import os
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from shelving_interfaces.action import (
    DetectGraspPoint,
    DetectTargetSlot,
    NavigateToTarget,
    PlaceBook,
)
from shelving_interfaces.msg import RobotStatus
from std_msgs.msg import String
import yaml

from .book_placer import (
    COMMAND_SWEEP,
    SIM_CANCELLED,
    SIM_FAILED,
    SIM_SUCCEEDED,
    cancel_command,
    choose_gap_any_board,
    decode,
    encode,
    error_name,
    MockSimExecutor,
    Outcome,
    PlaceTracker,
)
from .grasp_planner import (build_place_command, DEFAULT_LIMITS, GraspGoal, parse_profile,
                            parse_tray, PlaceGoal, resolve_book, select_tray_slot, SlotGoal,
                            snap_grasp_to_slot, validate_goal, validate_grasp)

# 진행 중 이 단계 이후에 실패하면 책이 이미 트레이를 떠났다고 본다
#LEFT_TRAY_PHASES = ('MOVING_TO_PRE_INSERT', 'INSERTING', 'RELEASING', 'RETREATING', 'VERIFYING')


class ManipulationNode(Node):

    def __init__(self, **kwargs):
        super().__init__('manipulation_node', **kwargs)
        p = self.declare_parameter
        self.action_name = p('action_name', '/place_book').value
        self.grasp_action_name = p(
            'grasp_action',
            '/detect_grasp_point',
        ).value

        self.slot_action_name = p(
            'slot_action',
            '/detect_target_slot',
        ).value

        self.navigation_action_name = p(
            'navigation_action',
            '/navigate_to_target',
        ).value

        self.navigation_server_timeout_s = float(p(
            'navigation_server_timeout_s',
            10.0,
        ).value)

        self.navigation_result_timeout_s = float(p(
            'navigation_result_timeout_s',
            60.0,
        ).value)

        self.global_frame = p(
            'global_frame',
            'map',
        ).value

        self.robot_base_frame = p(
            'robot_base_frame',
            'base_link',
        ).value

        self.arm_base_frame = p(
            'arm_base_frame',
            'arm_base_link',
        ).value

        # 차체 이동 후 빈칸이 팔 기준 이 x에 오도록 맞춥니다.
        self.alignment_target_x = float(p(
            'alignment_target_x',
            -0.35,
        ).value)

        # 서가 방향(+Y) 정렬 명령의 목표입니다. navigation_node의
        # position_tolerance(현재 0.05 m)를 고려하여 실제 정지 위치가
        # 약 y=0.675 m가 되도록 명령 목표를 0.625 m로 둡니다.
        self.alignment_target_y = float(p(
            'alignment_target_y',
            0.625,
        ).value)

        self.perception_server_timeout_s = float(p(
            'perception_server_timeout_s',
            10.0,
        ).value)

        # 차체 정렬 직후에는 카메라·Depth·TF가 서로 다른 갱신 주기의
        # 첫 프레임을 내보낼 수 있습니다. 그 프레임을 책 검출에 쓰지 않도록
        # 잠시 기다린 뒤 not_before 시각을 기록합니다.
        self.post_alignment_settle_s = max(0.0, float(p(
            'post_alignment_settle_s',
            1.0,
        ).value))

        self.perception_result_timeout_s = float(p(
            'perception_result_timeout_s',
            20.0,
        ).value)

        self.scan_dwell_s = float(p('scan_dwell_s', 1.2).value)
        self.scan_timeout_s = float(p('scan_timeout_s', 180.0).value)
        self.slot_x_snap_to_gap = bool(p('slot_x_snap_to_gap', False).value)
        self.slot_x_snap_max_m = float(p('slot_x_snap_max_m', 0.12).value)


        self.status_topic = p('status_topic', '/robot/status').value
        self.executor_kind = p('executor', 'mock').value
        self.command_topic = p('sim_command_topic', '/manipulation/sim/command').value
        self.state_topic = p('sim_state_topic', '/manipulation/sim/state').value
        profiles_file = p('book_profiles_file', '').value
        self.profile_name = p('book_profile', 'default').value
        self.heartbeat_timeout_s = float(p('heartbeat_timeout_s', 3.0).value)
        self.goal_timeout_s = float(p('goal_timeout_s', 180.0).value)
        self.cancel_timeout_s = float(p('cancel_timeout_s', 30.0).value)
        status_rate = float(p('status_rate_hz', 2.0).value)
        self.loop_period_s = 1.0 / float(p('feedback_rate_hz', 10.0).value)
        self.limits = {}
        for key, default in DEFAULT_LIMITS.items():
            self.limits[key] = p(key, default).value
        mock = MockSimExecutor(
            step_s=float(p('mock_step_s', 0.2).value),
            fail_at=p('mock_fail_at', '').value,
            fail_code=int(p('mock_fail_code', 0).value),
            unverified=bool(p('mock_unverified', False).value),
        )

        if not profiles_file:
            share = get_package_share_directory('shelving_manipulation')
            profiles_file = os.path.join(share, 'config', 'book_profiles.yaml')
        with open(profiles_file, encoding='utf-8') as f:
            profiles = yaml.safe_load(f)
        self.profile = parse_profile(profiles, self.profile_name)
        self.tray_slots, self.tray_assignments = parse_tray(profiles)
#        self.used_slots = set()

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._tracker = None
        self._scan_state = None
        self._scan_events = []
        self._shelf_gaps = None
        self._busy = False
        self._active_perception_goal = None
        self._active_navigation_goal = None
        self._status = ('IDLE', '', 0.0, 0, '')    # state, job, progress, error_code, message

        group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.navigation_client = ActionClient(
            self,
            NavigateToTarget,
            self.navigation_action_name,
            callback_group=group,
        )

        self.grasp_client = ActionClient(self, DetectGraspPoint, self.grasp_action_name, callback_group=group)
        self.slot_client = ActionClient(self, DetectTargetSlot, self.slot_action_name, callback_group=group)
        self.status_pub = self.create_publisher(RobotStatus, self.status_topic, 10)
        self.create_timer(1.0 / status_rate, self._publish_status, callback_group=group)

        if self.executor_kind == 'mock':
            self._mock = mock
            self._send = lambda command: self._mock.handle(command, time.monotonic())
            self.create_timer(0.05, self._poll_mock, callback_group=group)
        elif self.executor_kind == 'sim':
            self._mock = None
            self.command_pub = self.create_publisher(String, self.command_topic, 10)
            self._send = lambda command: self.command_pub.publish(String(data=encode(command)))
            self.create_subscription(String, self.state_topic, self._on_state_msg, 50,
                                     callback_group=group)
        else:
            raise ValueError(f"executor 는 mock 또는 sim: '{self.executor_kind}'")

        self.server = ActionServer(
            self, PlaceBook, self.action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=group,
        )
        self.get_logger().info(
            f"PlaceBook {self.action_name} "
            f"executor={self.executor_kind} "
            f"profile={self.profile_name} "
            f"tray_slots={len(self.tray_slots)} "
            f"grasp_action={self.grasp_action_name} "
            f"slot_action={self.slot_action_name} "
            f"status={self.status_topic}"
        )

    # -------------------------------------------------------------- 시뮬 상태

    def _poll_mock(self):
        self._on_sim_state(self._mock.poll(time.monotonic()))

    def _on_state_msg(self, msg):
        state = decode(msg.data)
        if state is None:
            self.get_logger().warning(f'시뮬 상태 형식 오류: {msg.data[:80]}')
            return
        self._on_sim_state(state)

    def _on_sim_state(self, state):
        """Isaac의 배치 상태와 스윕 상태를 각각 분리하여 저장합니다."""
        wake_required = False

        with self._lock:
            scan_state = self._scan_state

            if (
                scan_state is not None
                and state.get("token")
                == scan_state.get("token")
            ):
                updated_state = dict(state)
                self._scan_state = updated_state

                if (
                    updated_state.get("phase")
                    == "scan_plan"
                    and updated_state.get(
                        "shelf_gaps"
                    )
                ):
                    self._shelf_gaps = [
                        [
                            float(value)
                            for value in gap
                        ]
                        for gap
                        in updated_state[
                            "shelf_gaps"
                        ]
                    ]

                previous_event = (
                    self._scan_events[-1]
                    if self._scan_events
                    else None
                )

                # 같은 phase가 10 Hz로 반복 발행되므로,
                # phase 또는 status가 변한 경우만 대기열에 넣습니다.
                if (
                    previous_event is None
                    or previous_event.get("phase")
                    != updated_state.get("phase")
                    or previous_event.get("status")
                    != updated_state.get("status")
                ):
                    self._scan_events.append(updated_state)

                wake_required = True

            tracker = self._tracker

            if (
                tracker is not None
                and tracker.on_sim_state(
                    state,
                    time.monotonic(),
                )
            ):
                wake_required = True

        if wake_required:
            self._wake.set()

    # -------------------------------------------------------------- 액션

    def _on_goal(self, goal_request):
        """PlaceBook 전체 사이클은 동시에 하나만 허용합니다."""
        with self._lock:
            if self._busy or self._tracker is not None:
                self.get_logger().warning(
                    "PlaceBook goal rejected: another job is running."
                )
                return GoalResponse.REJECT

            self._busy = True

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _wait_for_future(
        self,
        future,
        timeout_s,
        place_goal_handle,
    ):
        """rclpy Future 완료·PlaceBook 취소·시간 초과를 함께 기다립니다."""
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())

        started = time.monotonic()

        while not completed.wait(0.05):
            if place_goal_handle.is_cancel_requested:
                return False, "canceled"

            if time.monotonic() - started > timeout_s:
                return False, "timeout"

        return True, ""

    def _clear_active_perception_goal(self, perception_goal_handle):
        """현재 저장된 perception goal과 같은 경우에만 해제합니다."""
        with self._lock:
            if self._active_perception_goal is perception_goal_handle:
                self._active_perception_goal = None

    def _call_perception_action(
        self,
        client,
        request,
        place_goal_handle,
        action_label,
    ):
        """하나의 perception action을 호출하고 결과를 기다립니다."""
        if not client.wait_for_server(
            timeout_sec=self.perception_server_timeout_s
        ):
            return (
                None,
                411,
                f"{action_label} action server is not available.",
            )

        send_future = client.send_goal_async(request)

        completed, reason = self._wait_for_future(
            send_future,
            self.perception_server_timeout_s,
            place_goal_handle,
        )

        if not completed:
            if reason == "canceled":
                return (
                    None,
                    412,
                    f"{action_label} request was canceled.",
                )

            return (
                None,
                404,
                f"Timed out sending {action_label} goal.",
            )

        try:
            perception_goal_handle = send_future.result()
        except Exception as error:
            return (
                None,
                411,
                f"Failed to send {action_label} goal: {error}",
            )

        if (
            perception_goal_handle is None
            or not perception_goal_handle.accepted
        ):
            return (
                None,
                411,
                f"{action_label} goal was rejected.",
            )

        with self._lock:
            self._active_perception_goal = perception_goal_handle

        result_future = perception_goal_handle.get_result_async()

        completed, reason = self._wait_for_future(
            result_future,
            self.perception_result_timeout_s,
            place_goal_handle,
        )

        if not completed:
            perception_goal_handle.cancel_goal_async()
            self._clear_active_perception_goal(
                perception_goal_handle
            )

            if reason == "canceled":
                return (
                    None,
                    412,
                    f"{action_label} request was canceled.",
                )

            return (
                None,
                404,
                f"Timed out waiting for {action_label} result.",
            )

        try:
            response = result_future.result()
        except Exception as error:
            self._clear_active_perception_goal(
                perception_goal_handle
            )
            return (
                None,
                411,
                f"Failed to receive {action_label} result: {error}",
            )

        self._clear_active_perception_goal(
            perception_goal_handle
        )

        if response is None:
            return (
                None,
                411,
                f"{action_label} returned no response.",
            )

        return response.result, 0, ""

    def _clear_active_navigation_goal(
        self,
        navigation_goal_handle,
    ):
        """현재 Navigation goal과 동일한 경우에만 저장값을 해제합니다."""
        with self._lock:
            if (
                self._active_navigation_goal
                is navigation_goal_handle
            ):
                self._active_navigation_goal = None

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        """Quaternion에서 평면 yaw를 계산합니다."""
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _lookup_transform_with_timeout(
        self,
        target_frame,
        source_frame,
        timeout_s=2.0,
    ):
        """최신 TF가 들어올 때까지 짧게 재시도합니다."""
        deadline = time.monotonic() + float(timeout_s)
        last_error = None

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                )
                return transform, ""
            except TransformException as error:
                last_error = error
                time.sleep(0.05)

        return (
            None,
            (
                f"TF {target_frame} <- {source_frame} "
                f"조회 실패: {last_error}"
            ),
        )

    def _align_base_to_slot(
        self,
        slot_message,
        place_goal_handle,
        job_id,
    ):
        """선택한 빈칸이 팔 작업 범위에 오도록 AMR을 정렬합니다."""
        gap_x = float(
            slot_message.pose.position.x
        )
        gap_y = float(
            slot_message.pose.position.y
        )

        min_x = float(
            self.limits["place_region_min"][0]
        )
        max_x = float(
            self.limits["place_region_max"][0]
        )

        x_needs_alignment = not (
            min_x <= gap_x <= max_x
        )
        y_needs_alignment = (
            gap_y > self.alignment_target_y + 0.05
        )

        # X가 작업 범위 안이고 서가 방향 거리도 충분히 가까우면
        # 차체를 움직이지 않습니다.
        if not x_needs_alignment and not y_needs_alignment:
            self.get_logger().info(
                "선택한 빈칸이 팔 작업 범위 안에 있습니다: "
                f"x={gap_x:+.4f}, "
                f"y={gap_y:+.4f}, "
                f"range={min_x:+.4f}..{max_x:+.4f}"
            )
            return True, 0, ""

        if not (
            min_x
            <= self.alignment_target_x
            <= max_x
        ):
            return (
                False,
                410,
                (
                    "alignment_target_x가 팔 작업 범위 밖입니다: "
                    f"target={self.alignment_target_x:+.4f}, "
                    f"range={min_x:+.4f}..{max_x:+.4f}"
                ),
            )

        base_before, error = (
            self._lookup_transform_with_timeout(
                self.global_frame,
                self.robot_base_frame,
            )
        )

        if base_before is None:
            return False, 410, error

        arm_before, error = (
            self._lookup_transform_with_timeout(
                self.global_frame,
                self.arm_base_frame,
            )
        )

        if arm_before is None:
            return False, 410, error

        base_translation = (
            base_before.transform.translation
        )
        base_rotation = (
            base_before.transform.rotation
        )

        arm_translation = (
            arm_before.transform.translation
        )
        arm_rotation = (
            arm_before.transform.rotation
        )

        arm_yaw_before = self._yaw_from_quaternion(
            arm_rotation
        )

        arm_cos = math.cos(arm_yaw_before)
        arm_sin = math.sin(arm_yaw_before)

        slot_x_before = float(
            slot_message.pose.position.x
        )
        slot_y_before = float(
            slot_message.pose.position.y
        )
        slot_z_before = float(
            slot_message.pose.position.z
        )

        slot_orientation_before = (
            slot_message.pose.orientation
        )
        slot_yaw_before = (
            self._yaw_from_quaternion(
                slot_orientation_before
            )
        )

        # 관측된 빈칸의 물리적 월드 위치를 보존합니다.
        slot_world_x = (
            arm_translation.x
            + arm_cos * slot_x_before
            - arm_sin * slot_y_before
        )
        slot_world_y = (
            arm_translation.y
            + arm_sin * slot_x_before
            + arm_cos * slot_y_before
        )
        slot_world_z = (
            arm_translation.z
            + slot_z_before
        )
        slot_world_yaw = (
            arm_yaw_before
            + slot_yaw_before
        )

        # 로봇이 arm_base_link +X 방향으로 이만큼 이동하면
        # 빈칸의 새 arm 기준 x가 alignment_target_x가 됩니다.
        requested_lateral_move = (
            slot_x_before - self.alignment_target_x
            if x_needs_alignment
            else 0.0
        )

        # arm_base_link +Y가 서가 방향입니다. Nav2 정밀 정렬의 5 cm
        # 성공 허용 오차를 포함해 약 2.5 cm 실제 접근이 일어나도록
        # 현재 빈칸 y에서 0.625 m 목표까지의 차이를 요청합니다.
        requested_forward_move = max(
            0.0,
            slot_y_before - self.alignment_target_y,
        )

        request = NavigateToTarget.Goal()
        request.job_id = f"{job_id}:base_alignment"
        request.target_type = "manipulation_alignment"
        request.target_id = "selected_shelf_gap"
        request.waypoints = []

        request.target_pose.header.frame_id = (
            self.global_frame
        )
        request.target_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        request.target_pose.pose.position.x = (
            base_translation.x
            + arm_cos * requested_lateral_move
            - arm_sin * requested_forward_move
        )
        request.target_pose.pose.position.y = (
            base_translation.y
            + arm_sin * requested_lateral_move
            + arm_cos * requested_forward_move
        )
        request.target_pose.pose.position.z = (
            base_translation.z
        )

        # 횡이동만 수행하도록 현재 차체 방향을 유지합니다.
        request.target_pose.pose.orientation.x = (
            base_rotation.x
        )
        request.target_pose.pose.orientation.y = (
            base_rotation.y
        )
        request.target_pose.pose.orientation.z = (
            base_rotation.z
        )
        request.target_pose.pose.orientation.w = (
            base_rotation.w
        )

        request.enable_fine_alignment = True

        self.get_logger().info(
            "빈칸 접근을 위한 ROS Navigation 요청: "
            f"slot_x={slot_x_before:+.4f}, "
            f"slot_y={slot_y_before:+.4f}, "
            f"target_x={self.alignment_target_x:+.4f}, "
            f"lateral_move={requested_lateral_move:+.4f}m, "
            f"forward_move={requested_forward_move:+.4f}m, "
            "target_map=("
            f"{request.target_pose.pose.position.x:+.4f}, "
            f"{request.target_pose.pose.position.y:+.4f})"
        )

        if not self.navigation_client.wait_for_server(
            timeout_sec=self.navigation_server_timeout_s
        ):
            return (
                False,
                411,
                (
                    f"{self.navigation_action_name} "
                    "action server is not available."
                ),
            )

        send_future = (
            self.navigation_client.send_goal_async(
                request
            )
        )

        completed, reason = self._wait_for_future(
            send_future,
            self.navigation_server_timeout_s,
            place_goal_handle,
        )

        if not completed:
            if reason == "canceled":
                return (
                    False,
                    412,
                    "차체 정렬 요청이 취소되었습니다.",
                )

            return (
                False,
                404,
                "차체 정렬 goal 전송 시간이 초과되었습니다.",
            )

        try:
            navigation_goal_handle = (
                send_future.result()
            )
        except Exception as error:
            return (
                False,
                411,
                f"차체 정렬 goal 전송 실패: {error}",
            )

        if (
            navigation_goal_handle is None
            or not navigation_goal_handle.accepted
        ):
            return (
                False,
                411,
                "Navigation node가 차체 정렬 goal을 거절했습니다.",
            )

        with self._lock:
            self._active_navigation_goal = (
                navigation_goal_handle
            )

        result_future = (
            navigation_goal_handle.get_result_async()
        )

        completed, reason = self._wait_for_future(
            result_future,
            self.navigation_result_timeout_s,
            place_goal_handle,
        )

        if not completed:
            navigation_goal_handle.cancel_goal_async()
            self._clear_active_navigation_goal(
                navigation_goal_handle
            )

            if reason == "canceled":
                return (
                    False,
                    412,
                    "차체 정렬 중 PlaceBook이 취소되었습니다.",
                )

            return (
                False,
                404,
                "차체 정렬 결과 대기 시간이 초과되었습니다.",
            )

        try:
            response = result_future.result()
        except Exception as error:
            self._clear_active_navigation_goal(
                navigation_goal_handle
            )
            return (
                False,
                411,
                f"차체 정렬 결과 수신 실패: {error}",
            )

        self._clear_active_navigation_goal(
            navigation_goal_handle
        )

        if response is None:
            return (
                False,
                411,
                "차체 정렬 결과가 비어 있습니다.",
            )

        navigation_result = response.result

        if (
            navigation_result is None
            or not navigation_result.success
            or not navigation_result.tolerance_satisfied
        ):
            navigation_error = (
                int(navigation_result.error_code)
                if navigation_result is not None
                else 0
            )
            navigation_message = (
                navigation_result.message
                if navigation_result is not None
                else "empty navigation result"
            )

            return (
                False,
                411,
                (
                    "차체 정렬 실패: "
                    f"navigation_error={navigation_error}, "
                    f"message={navigation_message}"
                ),
            )

        arm_after, error = (
            self._lookup_transform_with_timeout(
                self.global_frame,
                self.arm_base_frame,
            )
        )

        if arm_after is None:
            return False, 410, error

        arm_translation_after = (
            arm_after.transform.translation
        )
        arm_rotation_after = (
            arm_after.transform.rotation
        )
        arm_yaw_after = self._yaw_from_quaternion(
            arm_rotation_after
        )

        dx = (
            slot_world_x
            - arm_translation_after.x
        )
        dy = (
            slot_world_y
            - arm_translation_after.y
        )

        after_cos = math.cos(arm_yaw_after)
        after_sin = math.sin(arm_yaw_after)

        # 월드에 고정된 빈칸을 이동 후 arm_base_link로 다시 변환합니다.
        corrected_x = (
            after_cos * dx
            + after_sin * dy
        )
        corrected_y = (
            -after_sin * dx
            + after_cos * dy
        )
        corrected_z = (
            slot_world_z
            - arm_translation_after.z
        )

        corrected_yaw = math.atan2(
            math.sin(
                slot_world_yaw
                - arm_yaw_after
            ),
            math.cos(
                slot_world_yaw
                - arm_yaw_after
            ),
        )

        slot_message.header.frame_id = (
            self.arm_base_frame
        )
        slot_message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        slot_message.pose.position.x = corrected_x
        slot_message.pose.position.y = corrected_y
        slot_message.pose.position.z = corrected_z

        slot_message.pose.orientation.x = 0.0
        slot_message.pose.orientation.y = 0.0
        slot_message.pose.orientation.z = math.sin(
            corrected_yaw * 0.5
        )
        slot_message.pose.orientation.w = math.cos(
            corrected_yaw * 0.5
        )

        self.get_logger().info(
            "차체 정렬 후 빈칸 좌표 재계산: "
            f"x={slot_x_before:+.4f}"
            f"->{corrected_x:+.4f}, "
            f"y={slot_y_before:+.4f}"
            f"->{corrected_y:+.4f}, "
            f"requested_lateral_move="
            f"{requested_lateral_move:+.4f}m, "
            f"requested_forward_move="
            f"{requested_forward_move:+.4f}m"
        )

        if not min_x <= corrected_x <= max_x:
            return (
                False,
                410,
                (
                    "차체 정렬 후에도 빈칸이 팔 작업 범위 밖입니다: "
                    f"x={corrected_x:+.4f}, "
                    f"range={min_x:+.4f}..{max_x:+.4f}"
                ),
            )

        return True, 0, ""

    def _select_slot_from_sweep(
        self,
        slot_results,
        book_thickness,
    ):
        """R&D 정책으로 선반과 실제 빈칸을 선택합니다."""
        if not slot_results:
            return (
                None,
                410,
                "사용 가능한 빈 슬롯을 "
                "검출하지 못했습니다.",
            )

        # R&D 정책: 우선 낮은 단, 같은 단에서는
        # 중심에 가까운 관측을 우선합니다.
        selected_result, observed_floor_z = min(
            slot_results,
            key=lambda item: (
                float(item[1]),
                abs(
                    float(
                        item[0]
                        .target_slot
                        .pose
                        .position
                        .x
                    )
                ),
                -float(
                    item[0]
                    .target_slot
                    .confidence
                ),
            ),
        )

        slot = selected_result.target_slot

        if not self.slot_x_snap_to_gap:
            return slot, 0, ""

        if not self._shelf_gaps:
            return (
                None,
                410,
                "실제 선반 빈칸 목록을 "
                "받지 못했습니다.",
            )

        tagged_gaps = [
            gap
            for gap in self._shelf_gaps
            if len(gap) >= 3
        ]

        if not tagged_gaps:
            return (
                None,
                410,
                "선반 단 정보가 포함된 "
                "빈칸 목록이 없습니다.",
            )

        base_floor_z = min(
            float(gap[2])
            for gap in tagged_gaps
        )

        wanted_relative_z = (
            float(observed_floor_z)
            - base_floor_z
        )

        gap_x, gap_width, selected_relative_z, reason = (
            choose_gap_any_board(
                self._shelf_gaps,
                float(slot.pose.position.x),
                wanted_relative_z,
                float(book_thickness),
                min_clearance=float(
                    self.limits[
                        "side_clearance"
                    ]
                ),
                max_move=(
                    self.slot_x_snap_max_m
                ),
            )
        )

        if gap_x is None:
            return (
                None,
                410,
                reason,
            )

        old_x = float(
            slot.pose.position.x
        )
        old_z = float(
            slot.pose.position.z
        )

        slot.pose.position.x = float(
            gap_x
        )

        if selected_relative_z is not None:
            selected_floor_z = (
                base_floor_z
                + float(
                    selected_relative_z
                )
            )

            # 선반판이 바뀌면 책 중심 높이도
            # 같은 차이만큼 함께 이동시킵니다.
            slot.pose.position.z = (
                old_z
                + selected_floor_z
                - float(observed_floor_z)
            )

        self.get_logger().info(
            "R&D 빈칸 선택: "
            f"x={old_x:+.4f}"
            f"->{slot.pose.position.x:+.4f}, "
            f"z={old_z:+.4f}"
            f"->{slot.pose.position.z:+.4f}, "
            f"gap_width="
            f"{gap_width * 1000:.1f}mm, "
            f"book_thickness="
            f"{float(book_thickness) * 1000:.1f}mm, "
            f"reason={reason}"
        )

        # available_width만 채우면 현재 validate_goal()이
        # available_height=0까지 검사해 실패합니다.
        # 폭 검사는 위에서 끝났으므로 두 필드는 기존 0을 유지합니다.
        return slot, 0, ""

    def _run_shelf_sweep(
        self,
        goal_handle,
        job_id,
        book_width,
        book_height,
        book_thickness,
    ):
        """Isaac 스윕을 실행하고 각 hold에서 빈 슬롯을 검출합니다."""
        token = uuid.uuid4().hex
        scan_job_id = f"{job_id}:scan"

        book_width = float(book_width)
        book_height = float(book_height)
        book_thickness = float(book_thickness)

        with self._lock:
            self._scan_state = {
                "token": token,
                "job_id": scan_job_id,
                "status": "RUNNING",
                "phase": "scan_request",
            }
            self._scan_events.clear()
            self._shelf_gaps = None

        self._wake.clear()

        self._set_status(
            "RUNNING",
            job_id,
            0.15,
            0,
            "SCANNING_SHELF",
        )
        self._feedback(
            goal_handle,
            "SCANNING_SHELF",
            0.15,
        )

        self._send({
            "type": COMMAND_SWEEP,
            "token": token,
            "job_id": scan_job_id,
            "dwell_s": self.scan_dwell_s,
        })

        self.get_logger().info(
            f"서가 스윕 시작: "
            f"job={scan_job_id}, "
            f"dwell={self.scan_dwell_s:.1f}s"
        )

        started_at = time.monotonic()
        cancel_sent_at = None
        cancel_code = 0
        cancel_message = ""

        processed_hold_phases = set()
        slot_results = []

        try:
            while True:
                now = time.monotonic()

                if (
                    goal_handle.is_cancel_requested
                    and cancel_sent_at is None
                ):
                    cancel_sent_at = now
                    cancel_code = 412
                    cancel_message = (
                        "PlaceBook 취소 요청으로 "
                        "서가 스윕을 취소했습니다."
                    )

                    self._send(
                        cancel_command(
                            token,
                            scan_job_id,
                        )
                    )

                if (
                    cancel_sent_at is None
                    and now - started_at
                    > self.scan_timeout_s
                ):
                    cancel_sent_at = now
                    cancel_code = 404
                    cancel_message = (
                        "서가 스윕 제한 시간 "
                        f"{self.scan_timeout_s:.0f}s 초과"
                    )

                    self._send(
                        cancel_command(
                            token,
                            scan_job_id,
                        )
                    )

                self._wake.wait(
                    self.loop_period_s
                )
                self._wake.clear()

                with self._lock:
                    events = list(
                        self._scan_events
                    )
                    self._scan_events.clear()

                    current_state = dict(
                        self._scan_state or {}
                    )

                # perception Action 처리 중 쌓인 phase도 순서대로 처리합니다.
                for event in events:
                    phase = str(
                        event.get("phase", "")
                    )
                    status = str(
                        event.get("status", "")
                    ).upper()

                    if (
                        phase.endswith("_hold")
                        and phase
                        not in processed_hold_phases
                        and cancel_sent_at is None
                    ):
                        processed_hold_phases.add(
                            phase
                        )

                        self.get_logger().info(
                            f"스윕 정지점 {phase}: "
                            "빈 슬롯 검출 요청"
                        )

                        slot_request = (
                            DetectTargetSlot.Goal()
                        )
                        slot_request.book_width = (
                            book_width
                        )
                        slot_request.book_height = (
                            book_height
                        )
                        slot_request.book_thickness = (
                            book_thickness
                        )
                        slot_request.shelf_plane_y = float(
                            event.get("shelf_plane_y", 0.0)
                        )
                        slot_request.shelf_floor_z = float(
                            event.get("shelf_floor_z", 0.0)
                        )
                        slot_request.not_before = (
                            self.get_clock()
                            .now()
                            .to_msg()
                        )

                        self.get_logger().info(
                            f"{phase} 서가 기하: "
                            f"plane_y="
                            f"{slot_request.shelf_plane_y:.4f}, "
                            f"floor_z="
                            f"{slot_request.shelf_floor_z:.4f}"
                        )

                        detection_result, code, message = (
                            self._call_perception_action(
                                self.slot_client,
                                slot_request,
                                goal_handle,
                                (
                                    "DetectTargetSlot "
                                    f"at {phase}"
                                ),
                            )
                        )

                        if code == 412:
                            if cancel_sent_at is None:
                                cancel_sent_at = (
                                    time.monotonic()
                                )
                                cancel_code = 412
                                cancel_message = message

                                self._send(
                                    cancel_command(
                                        token,
                                        scan_job_id,
                                    )
                                )

                            continue

                        if code != 0:
                            self.get_logger().warning(
                                f"{phase} 슬롯 검출 실패: "
                                f"code={code}, "
                                f"{message}"
                            )
                            continue

                        if (
                            detection_result is None
                            or not detection_result.success
                        ):
                            result_code = (
                                int(
                                    detection_result.error_code
                                )
                                if detection_result
                                is not None
                                else 3004
                            )
                            result_message = (
                                detection_result.message
                                if detection_result
                                is not None
                                else "결과 없음"
                            )

                            self.get_logger().info(
                                f"{phase}에서 빈 슬롯 없음: "
                                f"code={result_code}, "
                                f"{result_message}"
                            )
                            continue

                        slot_results.append((
                            detection_result,
                            float(
                                event.get(
                                    "shelf_floor_z",
                                    0.0,
                                )
                            ),
                        ))

                        slot = (
                            detection_result.target_slot
                        )

                        self.get_logger().info(
                            f"{phase} 슬롯 후보: "
                            f"x={slot.pose.position.x:.4f}, "
                            f"y={slot.pose.position.y:.4f}, "
                            f"z={slot.pose.position.z:.4f}, "
                            f"confidence="
                            f"{slot.confidence:.3f}, "
                            f"candidates="
                            f"{detection_result.candidate_count}"
                        )

                        detected_count = len(
                            slot_results
                        )
                        progress = min(
                            0.45,
                            0.15
                            + 0.03
                            * len(
                                processed_hold_phases
                            ),
                        )

                        self._feedback(
                            goal_handle,
                            "DETECTING_SLOT",
                            progress,
                        )
                        self._set_status(
                            "RUNNING",
                            job_id,
                            progress,
                            0,
                            (
                                "DETECTING_SLOT "
                                f"results={detected_count}"
                            ),
                        )

                    if status == SIM_FAILED:
                        return (
                            None,
                            int(
                                event.get(
                                    "error_code",
                                    404,
                                )
                            ),
                            str(
                                event.get(
                                    "message",
                                    "서가 스윕 실패",
                                )
                            ),
                        )

                    if status == SIM_CANCELLED:
                        return (
                            None,
                            cancel_code or 412,
                            cancel_message
                            or str(
                                event.get(
                                    "message",
                                    "서가 스윕 취소",
                                )
                            ),
                        )

                    if status == SIM_SUCCEEDED:
                        if not slot_results:
                            return (
                                None,
                                410,
                                (
                                    "서가 스윕은 완료됐지만 "
                                    "사용 가능한 빈 슬롯을 "
                                    "검출하지 못했습니다."
                                ),
                            )

                        selected_slot, code, message = (
                            self._select_slot_from_sweep(
                                slot_results,
                                book_thickness,
                            )
                        )

                        if code != 0:
                            return (
                                None,
                                code,
                                message,
                            )

                        self.get_logger().info(
                            "서가 스윕 완료: "
                            f"정지점="
                            f"{len(processed_hold_phases)}, "
                            f"성공 관측="
                            f"{len(slot_results)}, "
                            "선택 좌표=("
                            f"{selected_slot.pose.position.x:.4f}, "
                            f"{selected_slot.pose.position.y:.4f}, "
                            f"{selected_slot.pose.position.z:.4f})"
                        )

                        return (
                            selected_slot,
                            0,
                            "",
                        )

                if (
                    cancel_sent_at is not None
                    and time.monotonic()
                    - cancel_sent_at
                    > self.cancel_timeout_s
                ):
                    return (
                        None,
                        cancel_code or 404,
                        (
                            cancel_message
                            or "서가 스윕 취소"
                        )
                        + (
                            f" — {self.cancel_timeout_s:.0f}s "
                            "안에 안전 정지 응답이 없습니다."
                        ),
                    )

                # 이벤트가 유실됐더라도 최신 terminal 상태는 놓치지 않습니다.
                current_status = str(
                    current_state.get(
                        "status",
                        "",
                    )
                ).upper()

                if (
                    current_status == SIM_SUCCEEDED
                    and not events
                ):
                    if not slot_results:
                        return (
                            None,
                            410,
                            (
                                "서가 스윕은 완료됐지만 "
                                "빈 슬롯 결과가 없습니다."
                            ),
                        )

                    selected_slot, code, message = (
                        self._select_slot_from_sweep(
                            slot_results,
                            book_thickness,
                        )
                    )

                    if code != 0:
                        return (
                            None,
                            code,
                            message,
                        )

                    return (
                        selected_slot,
                        0,
                        "",
                    )

        finally:
            with self._lock:
                self._scan_state = None
                self._scan_events.clear()

    def _stamp_age(self, stamp_message):
        """ROS timestamp가 있으면 현재 시각과의 차이를 초로 반환합니다."""
        stamp = Time.from_msg(stamp_message)

        if stamp.nanoseconds <= 0:
            return None

        age_ns = (
            self.get_clock().now().nanoseconds
            - stamp.nanoseconds
        )

        return max(0.0, age_ns / 1_000_000_000.0)

    def _to_grasp(
        self,
        grasp_message,
        book_thickness,
        book_width,
    ):
        """GraspObservation을 ROS 비의존 내부 GraspGoal로 변환합니다."""
        return GraspGoal(
            frame_id=grasp_message.header.frame_id,
            top_center=(
                grasp_message.top_center.x,
                grasp_message.top_center.y,
                grasp_message.top_center.z,
            ),
            spine_yaw=float(grasp_message.spine_yaw),
            thickness=float(book_thickness),
            width=float(book_width),
            confidence=float(grasp_message.confidence),
            age_s=self._stamp_age(
                grasp_message.header.stamp
            ),
        )

    def _to_goal(
        self,
        job_id,
        grasp_message,
        slot_message,
    ):
        """두 perception 결과로 내부 PlaceGoal을 생성합니다."""
        detected_thickness = float(grasp_message.thickness)
        detected_height = float(grasp_message.height)
        detected_width = float(grasp_message.width)

        book_thickness = (
            detected_thickness
            if detected_thickness > 0.0
            else self.profile.thickness
        )
        book_height = (
            detected_height
            if detected_height > 0.0
            else self.profile.height
        )
        book_width = (
            detected_width
            if detected_width > 0.0
            else self.profile.width
        )

        orientation = slot_message.pose.orientation

        slot = SlotGoal(
            frame_id=slot_message.header.frame_id,
            position=(
                slot_message.pose.position.x,
                slot_message.pose.position.y,
                slot_message.pose.position.z,
            ),
            orientation_xyzw=(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
            available_width=float(
                slot_message.available_width
            ),
            available_height=float(
                slot_message.available_height
            ),
            insertion_depth=float(
                slot_message.insertion_depth
            ),
            pre_insert_offset=float(
                slot_message.pre_insert_offset
            ),
            confidence=float(slot_message.confidence),
            age_s=self._stamp_age(
                slot_message.header.stamp
            ),
        )

        grasp = self._to_grasp(
            grasp_message,
            book_thickness,
            book_width,
        )

        return PlaceGoal(
            job_id=job_id,

            # 외부에서 book_id를 받지 않습니다.
            # 기존 tray slot 선택 함수용 내부 식별자만 생성합니다.
            book_id=f"vision:{job_id}",

            slot=slot,
            book_width=book_width,
            book_height=book_height,
            book_thickness=book_thickness,
            insertion_speed=float(
                self.limits["default_insertion_speed"]
            ),
            grasp=grasp,
        )

    def _finish(
        self,
        goal_handle,
        success,
        failed_phase,
        verified,
        code,
        message,
        canceled=False,
        job_id="",
    ):
        result = PlaceBook.Result()
        result.success = bool(success)
        result.failed_phase = failed_phase
        result.placement_verified = bool(verified)
        result.error_code = int(code)
        result.message = message

        if success:
            goal_handle.succeed()
        elif canceled and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        with self._lock:
            self._busy = False
            self._active_perception_goal = None
            self._active_navigation_goal = None

        text = (
            f"job={job_id or 'unassigned'} "
            f"code={code}({error_name(code)}) "
            f"phase={failed_phase} "
            f"{message}"
        )

        if success:
            self.get_logger().info(
                f"PlaceBook 성공 {text}"
            )
        else:
            self.get_logger().warning(
                f"PlaceBook 실패 {text}"
            )

        return result

    def _execute(self, goal_handle):
        """빈 PlaceBook Goal로 perception부터 배치까지 전체 작업을 수행합니다."""
        job_id = (
            "place-"
            + uuid.uuid4().hex[:12]
        )

        tracker = None
        token = None
        tray_slot = None

        try:
            if self.executor_kind != "sim":
                message = (
                    "자동 perception 및 스윕 작업은 "
                    "executor=sim에서만 지원합니다."
                )
                self._set_status(
                    "FAILED",
                    job_id,
                    0.0,
                    411,
                    message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    "SCANNING_SHELF",
                    False,
                    411,
                    message,
                    job_id=job_id,
                )

            # ------------------------------------------------------
            # 1. 선반 스윕 및 빈 슬롯 검출
            # ------------------------------------------------------
            self._set_status(
                "RUNNING",
                job_id,
                0.0,
                0,
                "SCANNING_SHELF",
            )
            self._feedback(
                goal_handle,
                "SCANNING_SHELF",
                0.0,
            )

            slot_message, code, message = (
                self._run_shelf_sweep(
                    goal_handle,
                    job_id,
                    book_width=self.profile.width,
                    book_height=self.profile.height,
                    book_thickness=self.profile.thickness,
                )
            )

            if slot_message is None:
                self._set_status(
                    "FAILED",
                    job_id,
                    0.0,
                    code,
                    message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    "DETECTING_SLOT",
                    False,
                    code,
                    message,
                    canceled=(code == 412),
                    job_id=job_id,
                )

            # start_sweep()가 끝날 때 팔은 q_observe로 복귀합니다.
            # 팔이 안전한 관측 자세에 있을 때 ROS Navigation으로
            # 선택한 빈칸을 팔의 작업 범위 안에 맞춥니다.
            # ------------------------------------------------------
            # 2. 선택한 빈칸에 맞춘 AMR 횡이동
            # ------------------------------------------------------
            self._set_status(
                "RUNNING",
                job_id,
                0.4,
                0,
                "ALIGNING_BASE",
            )
            self._feedback(
                goal_handle,
                "ALIGNING_BASE",
                0.4,
            )

            aligned, code, message = (
                self._align_base_to_slot(
                    slot_message,
                    goal_handle,
                    job_id,
                )
            )

            if not aligned:
                self._set_status(
                    "FAILED",
                    job_id,
                    0.4,
                    code,
                    message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    "ALIGNING_BASE",
                    False,
                    code,
                    message,
                    canceled=(code == 412),
                    job_id=job_id,
                )

            # 차체가 이동한 뒤 트레이 책을 새 자세에서 인식합니다.
            # 트레이는 로봇과 함께 움직이므로 책의 arm 기준 좌표를
            # 이동 완료 후 다시 측정하는 편이 안전합니다.
            # ------------------------------------------------------
            # 3. 트레이 책 검출
            # ------------------------------------------------------
            if self.post_alignment_settle_s > 0.0:
                self.get_logger().info(
                    '차체 정렬 후 카메라·Depth·TF 안정화 대기: '
                    f'{self.post_alignment_settle_s:.1f}s'
                )
                time.sleep(self.post_alignment_settle_s)

            self._set_status(
                "RUNNING",
                job_id,
                0.5,
                0,
                "DETECTING_BOOK",
            )
            self._feedback(
                goal_handle,
                "DETECTING_BOOK",
                0.5,
            )

            grasp_request = (
                DetectGraspPoint.Goal()
            )
            grasp_request.not_before = (
                self.get_clock()
                .now()
                .to_msg()
            )

            grasp_result, code, message = (
                self._call_perception_action(
                    self.grasp_client,
                    grasp_request,
                    goal_handle,
                    "DetectGraspPoint",
                )
            )

            if code != 0:
                self._set_status(
                    "FAILED",
                    job_id,
                    0.5,
                    code,
                    message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    "DETECTING_BOOK",
                    False,
                    code,
                    message,
                    canceled=(code == 412),
                    job_id=job_id,
                )

            if (
                grasp_result is None
                or not grasp_result.success
            ):
                perception_error = (
                    int(grasp_result.error_code)
                    if grasp_result is not None
                    else 3004
                )

                perception_message = (
                    grasp_result.message
                    if grasp_result is not None
                    else "책 검출 결과가 없습니다."
                )

                if (
                    goal_handle.is_cancel_requested
                    or perception_error == 3002
                ):
                    code = 412
                elif perception_error == 3003:
                    # 유효한 프레임은 받았지만 책 후보가 없음
                    code = 411
                else:
                    # 잘못된 좌표·TF·결과 형식
                    code = 410

                self._set_status(
                    "FAILED",
                    job_id,
                    0.5,
                    code,
                    perception_message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    "DETECTING_BOOK",
                    False,
                    code,
                    perception_message,
                    canceled=(code == 412),
                    job_id=job_id,
                )

            grasp_message = (
                grasp_result.grasp
            )

            self.get_logger().info(
                "책 검출 완료: "
                f"candidates="
                f"{grasp_result.candidate_count}, "
                "top_center=("
                f"{grasp_message.top_center.x:.4f}, "
                f"{grasp_message.top_center.y:.4f}, "
                f"{grasp_message.top_center.z:.4f}), "
                f"confidence="
                f"{grasp_message.confidence:.3f}"
            )

            # ------------------------------------------------------
            # 3. perception 메시지를 내부 목표로 변환
            # ------------------------------------------------------
            goal = self._to_goal(
                job_id,
                grasp_message,
                slot_message,
            )

            # 비전은 어느 책인지 고르고, 실제 파지 중심과 책 치수는
            # 위치를 아는 고정 지그인 트레이의 슬롯별 교정값을 사용합니다.
            goal, note = snap_grasp_to_slot(
                goal,
                self.tray_slots,
                self.limits,
            )

            if note:
                self.get_logger().info(
                    note
                )

            book, check = resolve_book(
                goal,
                self.profile,
                self.limits,
            )

            if check.ok and goal.grasp is not None and not note:
                check = type(check)(
                    410,
                    (
                        "비전 관측으로 실제 트레이 슬롯을 "
                        "식별할 수 없습니다."
                    ),
                )

            if check.ok:
                check = validate_goal(
                    goal,
                    book,
                    self.limits,
                )

            if check.ok:
                check = validate_grasp(
                    goal,
                    self.tray_slots,
                    self.limits,
                )

                if not check.ok:
                    self.get_logger().warning(
                        "파지 관측 거절: "
                        f"{check.message}"
                    )

            # ------------------------------------------------------
            # 4. 검출된 책 좌표와 실제 트레이 칸 연결
            # ------------------------------------------------------
            if check.ok:
                if (
                    goal.grasp is None
                    or not self.tray_slots
                ):
                    check = type(check)(
                        411,
                        "파지 좌표 또는 트레이 칸 설정이 없습니다.",
                    )
                else:
                    grasp_x = float(
                        goal.grasp.top_center[0]
                    )

                    tray_slot = min(
                        self.tray_slots,
                        key=lambda slot: abs(
                            float(slot.center[0])
                            - grasp_x
                        ),
                    )

                    # if tray_slot.index in self.used_slots:
                    #     check = type(check)(
                    #         411,
                    #         (
                    #             "비전이 이미 사용한 "
                    #             f"트레이 칸 {tray_slot.index}을 "
                    #             "다시 선택했습니다."
                    #         ),
                    #     )

            if not check.ok:
                phase = (
                    "DETECTING_BOOK"
                    if check.code == 411
                    else "PLANNING_GRASP"
                )

                self._set_status(
                    "FAILED",
                    job_id,
                    0.5,
                    check.code,
                    check.message,
                )
                return self._finish(
                    goal_handle,
                    False,
                    phase,
                    False,
                    check.code,
                    check.message,
                    job_id=job_id,
                )

            # ------------------------------------------------------
            # 5. Isaac 배치 명령 생성 및 발행
            # ------------------------------------------------------
            token = uuid.uuid4().hex

            command = build_place_command(
                token,
                goal,
                book,
                tray_slot,
                self.limits,
            )

            tracker = PlaceTracker(
                token=token,
                job_id=job_id,
                started_at=time.monotonic(),
                heartbeat_timeout_s=(
                    self.heartbeat_timeout_s
                ),
                goal_timeout_s=(
                    self.goal_timeout_s
                ),
            )

            with self._lock:
                self._tracker = tracker

            self.get_logger().info(
                f"작업 {job_id}: "
                f"트레이 칸 {tray_slot.index}, "
                f"파지 {command['pick']['center']} "
                f"({command['pick']['source']}) "
                f"→ 슬롯 {goal.slot.position}, "
                f"삽입 속도 "
                f"{command['insertion_speed']:.3f}m/s"
            )

            self._set_status(
                "RUNNING",
                job_id,
                0.5,
                0,
                "PLANNING_GRASP",
            )
            self._feedback(
                goal_handle,
                "PLANNING_GRASP",
                0.5,
            )

            self._send(command)

            cancel_sent_at = None
            last_phase = None

            try:
                while True:
                    self._wake.wait(
                        self.loop_period_s
                    )
                    self._wake.clear()

                    now = time.monotonic()

                    with self._lock:
                        if (
                            goal_handle
                            .is_cancel_requested
                            and tracker.request_cancel()
                        ):
                            cancel_sent_at = now
                            self._send(
                                cancel_command(
                                    token,
                                    job_id,
                                )
                            )

                        if (
                            tracker.check(now)
                            == "cancel"
                        ):
                            if cancel_sent_at is None:
                                cancel_sent_at = now

                            self._send(
                                cancel_command(
                                    token,
                                    job_id,
                                )
                            )

                        if (
                            cancel_sent_at
                            is not None
                            and tracker.outcome
                            is None
                            and now
                            - cancel_sent_at
                            > self.cancel_timeout_s
                        ):
                            tracker.outcome = Outcome(
                                False,
                                tracker.phase,
                                False,
                                404,
                                (
                                    "취소 후 "
                                    f"{self.cancel_timeout_s:.0f}s "
                                    "안에 안전 정지 보고가 없습니다."
                                ),
                            )

                        outcome = tracker.outcome
                        phase = tracker.phase
                        place_progress = (
                            tracker.progress
                        )

                    overall_progress = (
                        0.5
                        + 0.5
                        * float(place_progress)
                    )

                    if (
                        phase != last_phase
                        or outcome is None
                    ):
                        self._feedback(
                            goal_handle,
                            phase,
                            overall_progress,
                        )
                        self._set_status(
                            "RUNNING",
                            job_id,
                            overall_progress,
                            0,
                            phase,
                        )
                        last_phase = phase

                    if outcome is not None:
                        break

            finally:
                with self._lock:
                    self._tracker = None

            # 책이 트레이를 떠난 단계까지 갔다면
            # 실패했더라도 같은 책을 다시 선택하지 않습니다.
            # if (
            #     outcome.success
            #     or any(
            #         phase_name
            #         in tracker.history
            #         for phase_name
            #         in LEFT_TRAY_PHASES
            #     )
            # ):
            #     self.used_slots.add(
            #         tray_slot.index
            #     )

            final_state = (
                "SUCCEEDED"
                if outcome.success
                else "FAILED"
            )

            self._set_status(
                final_state,
                job_id,
                (
                    1.0
                    if outcome.success
                    else (
                        0.5
                        + 0.5
                        * tracker.progress
                    )
                ),
                outcome.error_code,
                (
                    outcome.message
                    or outcome.failed_phase
                ),
            )

            return self._finish(
                goal_handle,
                outcome.success,
                outcome.failed_phase,
                outcome.placement_verified,
                outcome.error_code,
                outcome.message,
                canceled=(
                    outcome.error_code == 412
                ),
                job_id=job_id,
            )

        except Exception as error:
            self.get_logger().error(
                "PlaceBook 내부 예외: "
                f"{type(error).__name__}: {error}"
            )

            perception_goal = None
            navigation_goal = None
            active_tracker = None

            with self._lock:
                perception_goal = (
                    self._active_perception_goal
                )
                navigation_goal = (
                    self._active_navigation_goal
                )
                active_tracker = self._tracker

                self._active_perception_goal = None
                self._active_navigation_goal = None
                self._tracker = None

            if perception_goal is not None:
                try:
                    perception_goal.cancel_goal_async()
                except Exception:
                    pass

            if navigation_goal is not None:
                try:
                    navigation_goal.cancel_goal_async()
                except Exception:
                    pass

            if (
                active_tracker is not None
                and token is not None
                and active_tracker.request_cancel()
            ):
                try:
                    self._send(
                        cancel_command(
                            token,
                            job_id,
                        )
                    )
                except Exception:
                    pass

            message = (
                "PlaceBook 내부 예외: "
                f"{type(error).__name__}: {error}"
            )

            self._set_status(
                "FAILED",
                job_id,
                0.0,
                404,
                message,
            )

            return self._finish(
                goal_handle,
                False,
                "PLANNING_GRASP",
                False,
                404,
                message,
                job_id=job_id,
            )

    def _feedback(self, goal_handle, phase, progress):
        fb = PlaceBook.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    # -------------------------------------------------------------- 상태

    def _set_status(self, state, job_id, progress, code, message):
        self._status = (state, job_id, float(progress), int(code), message)

    def _publish_status(self):
        state, job_id, progress, code, message = self._status
        msg = RobotStatus()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.limits['frame_id']
        msg.component = 'manipulation'
        msg.state = state
        msg.active_job_id = job_id
        msg.progress = progress
        msg.error_code = code
        msg.message = message
        msg.heartbeat_time = now
        self.status_pub.publish(msg)


class SimMockNode(Node):
    """Isaac 작업 실행기 자리에 붙이는 시험용 노드. executor:=sim 경로를 Isaac 없이 확인한다."""

    def __init__(self, **kwargs):
        super().__init__('manipulation_sim_mock', **kwargs)
        p = self.declare_parameter
        self.mock = MockSimExecutor(
            step_s=float(p('mock_step_s', 0.2).value),
            fail_at=p('mock_fail_at', '').value,
            fail_code=int(p('mock_fail_code', 0).value),
            unverified=bool(p('mock_unverified', False).value),
        )
        state_topic = p('sim_state_topic', '/manipulation/sim/state').value
        self.pub = self.create_publisher(String, state_topic, 10)
        self.create_subscription(String, p('sim_command_topic', '/manipulation/sim/command').value,
                                 self._on_command, 10)
        self.create_timer(0.1, self._tick)

    def _on_command(self, msg):
        command = decode(msg.data)
        if command is not None:
            self.mock.handle(command, time.monotonic())

    def _tick(self):
        self.pub.publish(String(data=encode(self.mock.poll(time.monotonic()))))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def sim_mock_main(args=None):
    rclpy.init(args=args)
    node = SimMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
