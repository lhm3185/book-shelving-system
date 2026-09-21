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

import os
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shelving_interfaces.action import PlaceBook
from shelving_interfaces.msg import RobotStatus
from std_msgs.msg import String
import yaml

from .book_placer import (cancel_command, decode, encode, error_name, MockSimExecutor, Outcome,
                          PlaceTracker)
from .grasp_planner import (build_place_command, DEFAULT_LIMITS, GraspGoal, parse_profile,
                            parse_tray, PlaceGoal, resolve_book, select_tray_slot, SlotGoal,
                            snap_grasp_to_slot, validate_goal, validate_grasp)

# 진행 중 이 단계 이후에 실패하면 책이 이미 트레이를 떠났다고 본다
LEFT_TRAY_PHASES = ('MOVING_TO_PRE_INSERT', 'INSERTING', 'RELEASING', 'RETREATING', 'VERIFYING')


class ManipulationNode(Node):

    def __init__(self, **kwargs):
        super().__init__('manipulation_node', **kwargs)
        p = self.declare_parameter
        self.action_name = p('action_name', '/place_book').value
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
        self.used_slots = set()

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._tracker = None
        self._status = ('IDLE', '', 0.0, 0, '')    # state, job, progress, error_code, message

        group = ReentrantCallbackGroup()
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
            f'PlaceBook {self.action_name} executor={self.executor_kind} '
            f'profile={self.profile_name} tray_slots={len(self.tray_slots)} '
            f'status={self.status_topic}')

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
        with self._lock:
            tracker = self._tracker
            # 다른 작업(이전 작업의 마지막 상태 반복 포함)의 상태는 token 이 달라 무시된다
            if tracker is not None and tracker.on_sim_state(state, time.monotonic()):
                self._wake.set()

    # -------------------------------------------------------------- 액션

    def _on_goal(self, goal_request):
        with self._lock:
            if self._tracker is not None:
                self.get_logger().warning(
                    f'목표 거절: 작업 {self._tracker.job_id} 실행 중 (M411 NOT_READY)')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _to_goal(self, request):
        slot = request.target_slot
        stamp = Time.from_msg(slot.header.stamp)
        age = None
        if stamp.nanoseconds > 0:
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        q = slot.pose.orientation
        return PlaceGoal(
            job_id=request.job_id,
            book_id=request.book_id,
            slot=SlotGoal(
                frame_id=slot.header.frame_id,
                position=(slot.pose.position.x, slot.pose.position.y, slot.pose.position.z),
                orientation_xyzw=(q.x, q.y, q.z, q.w),
                available_width=slot.available_width,
                available_height=slot.available_height,
                insertion_depth=slot.insertion_depth,
                pre_insert_offset=slot.pre_insert_offset,
                confidence=slot.confidence,
                age_s=age,
            ),
            book_width=request.book_width,
            book_height=request.book_height,
            book_thickness=request.book_thickness,
            insertion_speed=request.insertion_speed,
            grasp=self._to_grasp(request),
        )

    def _to_grasp(self, request):
        """
        비전이 준 파지 관측을 값으로 바꾼다 (has_grasp 이 false 면 None).

        None 이면 기존처럼 **설정 파일의 트레이 칸**으로 집는다 — 비전이 늦어도
        흐름이 멈추지 않는다 (계약 §4).
        """
        if not getattr(request, 'has_grasp', False):
            return None
        g = request.grasp
        gstamp = Time.from_msg(g.header.stamp)
        gage = None
        if gstamp.nanoseconds > 0:
            gage = (self.get_clock().now() - gstamp).nanoseconds / 1e9
        return GraspGoal(
            frame_id=g.header.frame_id,
            top_center=(g.top_center.x, g.top_center.y, g.top_center.z),
            spine_yaw=g.spine_yaw,
            thickness=g.thickness,
            width=g.width,
            confidence=g.confidence,
            age_s=gage,
        )

    def _finish(self, goal_handle, success, failed_phase, verified, code, message, canceled=False):
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
        text = (f'job={goal_handle.request.job_id} code={code}({error_name(code)}) '
                f'phase={failed_phase} {message}')
        # rclpy 는 같은 호출 위치에서 심각도를 바꾸면 예외를 던진다 → 호출 위치를 나눈다
        if success:
            self.get_logger().info(f'PlaceBook 성공 {text}')
        else:
            self.get_logger().warning(f'PlaceBook 실패 {text}')
        return result

    def _execute(self, goal_handle):
        request = goal_handle.request
        job_id = request.job_id
        goal = self._to_goal(request)
        self._set_status('RUNNING', job_id, 0.0, 0, 'DETECTING_BOOK')
        self._feedback(goal_handle, 'DETECTING_BOOK', 0.0)

        # 트레이 칸 좌표는 설정값. 비전 관측(grasp)이 오면 **검사를 통과한 것만** 쓴다
        book, check = resolve_book(goal, self.profile, self.limits)
        if check.ok:
            check = validate_goal(goal, book, self.limits)
        if check.ok:
            # **검사 전에** 비전 x 를 칸 중심에 맞춘다 — 트레이는 좌표를 아는 고정 지그다.
            # 비전이 정하는 것은 몇 번 칸인가이고, 그 칸의 x 는 지그가 이미 안다
            goal, note = snap_grasp_to_slot(goal, self.tray_slots)
            if note:
                self.get_logger().info(note)
            # 계약 §5 — 틀린 점을 경계에서 잡는다. 특히 "윗면 높이가 책 규격과 맞는가"
            # 위에서 맞췄어도 이 검사는 그대로 둔다 (마지막 안전선이다)
            check = validate_grasp(goal, self.tray_slots, self.limits)
            if not check.ok:
                self.get_logger().warning(f'파지 관측 거절: {check.message}')
        tray_slot = None
        if check.ok:
            tray_slot, check = select_tray_slot(
                goal.book_id, self.tray_slots, self.tray_assignments, self.used_slots)
        if not check.ok:
            phase = 'DETECTING_BOOK' if check.code == 411 else 'PLANNING_GRASP'
            self._set_status('FAILED', job_id, 0.0, check.code, check.message)
            return self._finish(goal_handle, False, phase, False, check.code, check.message)

        token = uuid.uuid4().hex
        command = build_place_command(token, goal, book, tray_slot, self.limits)
        tracker = PlaceTracker(token=token, job_id=job_id, started_at=time.monotonic(),
                               heartbeat_timeout_s=self.heartbeat_timeout_s,
                               goal_timeout_s=self.goal_timeout_s)
        with self._lock:
            self._tracker = tracker
        self.get_logger().info(
            f'작업 {job_id} book={goal.book_id} 트레이 칸 {tray_slot.index} '
            f"파지 {command['pick']['center']} ({command['pick']['source']}) "
            f'→ {goal.slot.position} '
            f"삽입 속도 {command['insertion_speed']:.3f} m/s")
        self._send(command)

        cancel_sent_at = None
        last_phase = None
        try:
            while True:
                self._wake.wait(self.loop_period_s)
                self._wake.clear()
                now = time.monotonic()
                with self._lock:
                    if goal_handle.is_cancel_requested and tracker.request_cancel():
                        cancel_sent_at = now
                        self._send(cancel_command(token, job_id))
                    if tracker.check(now) == 'cancel':
                        cancel_sent_at = now
                        self._send(cancel_command(token, job_id))
                    if (cancel_sent_at is not None and tracker.outcome is None
                            and now - cancel_sent_at > self.cancel_timeout_s):
                        tracker.outcome = Outcome(
                            False, tracker.phase, False, 404,
                            f'취소 후 {self.cancel_timeout_s:.0f}s 안에 안전 정지 보고 없음')
                    outcome = tracker.outcome
                    phase, progress = tracker.phase, tracker.progress
                if phase != last_phase or outcome is None:
                    self._feedback(goal_handle, phase, progress)
                    self._set_status('RUNNING', job_id, progress, 0, phase)
                    last_phase = phase
                if outcome is not None:
                    break
        finally:
            with self._lock:
                self._tracker = None

        if outcome.success or any(ph in tracker.history for ph in LEFT_TRAY_PHASES):
            self.used_slots.add(tray_slot.index)
        self._set_status('SUCCEEDED' if outcome.success else 'FAILED', job_id, tracker.progress,
                         outcome.error_code, outcome.message or outcome.failed_phase)
        return self._finish(goal_handle, outcome.success, outcome.failed_phase,
                            outcome.placement_verified, outcome.error_code, outcome.message,
                            canceled=outcome.error_code == 412)

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
