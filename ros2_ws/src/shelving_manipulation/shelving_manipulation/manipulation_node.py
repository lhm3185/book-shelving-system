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

from dataclasses import replace
import math
import os
import threading
import time
import traceback
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shelving_interfaces.action import DetectTargetSlot, PlaceBook
from shelving_interfaces.msg import RobotStatus
from std_msgs.msg import Bool, String
import yaml

from .book_placer import (cancel_command, choose_gap_any_board, COMMAND_ROTATE_BASE,
                          decode, encode, error_name, internal_failure, MockSimExecutor,
                          Outcome, PlaceTracker, publish_feedback_safely, SIM_CANCELLED,
                          SIM_FAILED, SIM_SUCCEEDED, stale_gap_reason, steady_book)
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
        self.perception_action = p('perception_action', '/detect_target_slot').value
        self.enable_perception_bridge = bool(p('enable_perception_bridge', False).value)
        self.auto_detect_book = bool(p('auto_detect_book', False).value)
        self.status_topic = p('status_topic', '/robot/status').value
        self.executor_kind = p('executor', 'mock').value
        self.command_topic = p('sim_command_topic', '/manipulation/sim/command').value
        self.state_topic = p('sim_state_topic', '/manipulation/sim/state').value
        profiles_file = p('book_profiles_file', '').value
        self.profile_name = p('book_profile', 'default').value
        self.heartbeat_timeout_s = float(p('heartbeat_timeout_s', 3.0).value)
        self.goal_timeout_s = float(p('goal_timeout_s', 180.0).value)
        self.cancel_timeout_s = float(p('cancel_timeout_s', 30.0).value)
        self.scan_dwell_s = float(p('scan_dwell_s', 1.2).value)
        # 서가 앞면까지 거리 (m): 스캔은 자세 표 기준 0.75, 꽂기는 도착 자리 그대로 0.36
        # 1차 시연 파지 자리(서가 앞면 0.444 m)에서 스캔도 꽂기도 한다 — 앞뒤로 안 움직인다 (2026-09-23).
        # 스윕 레시피(arm_kinematics.sweep_recipe)가 이 거리 기준으로 실측됐다.
        self.scan_standoff_m = float(p('scan_standoff_m', 0.444).value)
        # 빈칸(틈) 관측의 깊이를 **서가 앞면 실측에서 파생**할 것인가. 기본 꺼짐.
        # 틈은 뚫려 있어 깊이 센서가 구멍을 통과한다 — 아래 `_pick_slot` 주석 참조.
        self.slot_y_from_shelf_front = bool(p('slot_y_from_shelf_front', False).value)
        # 빈칸 x 를 **시뮬이 잰 실제 빈칸**에 맞출 것인가. 기본 꺼짐.
        # 비전의 빈칸 x 가 판마다 160 mm 흔들린다 (2026-09-24 다섯 판) — 아래 참조.
        self.slot_x_snap_to_gap = bool(p('slot_x_snap_to_gap', False).value)
        # 가장 가까운 실측 빈칸이 이보다 멀면 **손대지 않는다** (어느 칸인지 알 수 없다)
        self.slot_x_snap_max_m = float(p('slot_x_snap_max_m', 0.12).value)
        self._slot_reject = None     # 맞춤이 거절한 사유 (결과 메시지로 올린다)
        # 스캔 명령: scan_sweep(수평 스윕, 기본) / scan_shelf(예전 자세 표)
        self.scan_command = str(p('scan_command', 'scan_sweep').value)
        # 작업 전 차체 정렬(회전·서가 중앙·거리). 시뮬 실행기(rotate_base)가 있어야 한다 — 코드 기본은 끔이라
        # 가짜 시뮬로 도는 테스트가 30 s 회전 대기에 걸리지 않고, 실제 실행은 manipulation.yaml 에서 켠다.
        self.align_base_before_work = bool(p('align_base_before_work', False).value)
        # 작업 자세(월드 yaw, 도): 서가와 **나란히** 서서 팔 +Y 가 서가 앞면을 보게. level_franka0 의 shelf_01 은
        # x 1.87~3.28, y -2.57~-2.27 (yaw 0) 이고 카트는 y -3.0 에 서므로 yaw 0 이 맞다.
        # 90 은 옆 서가 끝을 봤다 (2026-09-23)
        self.work_yaw_deg = float(p('work_yaw_deg', 0.0).value)
        self.place_standoff_m = float(p('place_standoff_m', 0.444).value)
        self.scan_timeout_s = float(p('scan_timeout_s', 180.0).value)
        self.book_detect_timeout_s = float(p('book_detect_timeout_s', 15.0).value)
        self.slot_center_inset = float(p('slot_center_inset', 0.024).value)
        self.fixed_insertion_depth = float(p('fixed_insertion_depth', 0.30).value)
        self.vision_grasp_min = tuple(float(v) for v in p(
            'vision_grasp_region_min', [-0.55, -0.35, 0.05]).value)
        self.vision_grasp_max = tuple(float(v) for v in p(
            'vision_grasp_region_max', [-0.15, 0.35, 0.30]).value)
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
        self._busy = False
        self._scan_state = None
        self._empty_observations = []
        self._shelf_gaps = None      # 시뮬이 잰 **실제 빈칸** 팔기준 x 구간 [[lo, hi], ...]
        # **빈칸은 배치 한 번이면 낡는다.** 꽂은 책이 그 자리를 채우기 때문이다.
        # 한 판에 사이클 하나만 돌면 안 보이는데, 생중계는 사이클을 여러 번 돈다.
        self._place_count = 0        # 책을 꽂아 본 횟수
        self._shelf_gaps_at = None   # 빈칸을 잰 시점의 `_place_count`
        self._shelf_box = None       # 시뮬이 스캔 계획 때 알려주는 서가 상자(팔 기준). 빈칸은 이 안에서만 인정
        self._book_observations = []
        self._status = ('IDLE', '', 0.0, 0, '')    # state, job, progress, error_code, message
        self._feedback_skipped = 0   # 피드백을 못 보낸 횟수. 조용히 삼키지 않는다

        group = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(RobotStatus, self.status_topic, 10)
        self.detect_pub = self.create_publisher(Bool, '/perception/detect_request', 10)
        self.scan_active_pub = self.create_publisher(Bool, '/perception/slot_scan_active', 10)
        # 작업 서가의 x 범위(팔 기준, 꽂기 자리). 옆 서가와의 틈(x -0.43)을 빈칸으로 잡은 적이 있다 (2026-09-23).
        # level_franka0 shelf_01: 월드 x 1.87~3.28, 팔 베이스 x 2.22 → -0.35~1.06;
        # 책 반폭·팔 도달을 빼서 -0.25~0.45
        # 작업 위치에서 서가 중앙에 맞춰 서므로(executor rotate_base center) 범위는 좌우 대칭 — 서가 반폭 0.7 에서 책 반폭·여유를 뺐다
        # 검증된 삽입 x (1차 시연 고정 칸 -0.51~-0.27 의 가운데). 틈이 다른 x 에 있으면 차체를 옆으로 옮겨 맞춘다
        self.place_x = float(p('place_x', -0.35).value)
        self.shelf_x_min = float(p('shelf_x_min', -0.50).value)
        self.shelf_x_max = float(p('shelf_x_max', 0.50).value)
        self.create_subscription(PointStamped, '/perception/empty_shelf_position',
                                 self._on_empty_slot, 50, callback_group=group)
        self.create_subscription(PointStamped, '/perception/books',
                                 self._on_book, 50, callback_group=group)
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
            goal_callback=self._on_place_goal,
            cancel_callback=self._on_cancel,
            callback_group=group,
        )
        self.perception_server = None
        if self.enable_perception_bridge:
            self.perception_server = ActionServer(
                self, DetectTargetSlot, self.perception_action,
                execute_callback=self._execute_slot_detection,
                goal_callback=self._on_detection_goal,
                cancel_callback=self._on_cancel,
                callback_group=group,
            )
        self.get_logger().info(
            f'PlaceBook {self.action_name} executor={self.executor_kind} '
            f'profile={self.profile_name} tray_slots={len(self.tray_slots)} '
            f'status={self.status_topic} perception_bridge='
            f'{self.enable_perception_bridge}')

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
            if self._scan_state is not None \
                    and state.get('token') == self._scan_state.get('token'):
                self._scan_state = dict(state)
                if state.get('phase') == 'scan_plan' and state.get('shelf_gaps'):
                    self._shelf_gaps = [[float(v) for v in g] for g in state['shelf_gaps']]
                    self._shelf_gaps_at = self._place_count      # 이 시점의 값이다
                if state.get('phase') == 'scan_plan' and state.get('shelf_box'):
                    # 팔 기준 [x0, x1, y_front, y_back, z0, z1]
                    self._shelf_box = [float(v) for v in state['shelf_box']]
                self._wake.set()
            tracker = self._tracker
            # 다른 작업(이전 작업의 마지막 상태 반복 포함)의 상태는 token 이 달라 무시된다
            if tracker is not None and tracker.on_sim_state(state, time.monotonic()):
                self._wake.set()

    # -------------------------------------------------------------- 액션

    def _claim_goal(self, label):
        with self._lock:
            if self._busy or self._tracker is not None:
                active = self._tracker.job_id if self._tracker is not None else 'scan'
                self.get_logger().warning(f'{label} 목표 거절: 작업 {active} 실행 중')
                return False
            self._busy = True
        return True

    def _on_place_goal(self, goal_request):
        if not self._claim_goal('PlaceBook'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_detection_goal(self, goal_request):
        if self.executor_kind != 'sim':
            self.get_logger().warning('빈 슬롯 스캔은 executor=sim 에서만 지원한다')
            return GoalResponse.REJECT
        if not self._claim_goal('DetectTargetSlot'):
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
        # **끝내는 일도 터질 수 있다.** 이미 끝난 목표에 다시 하면 rclpy 가 던지고,
        # 그 예외가 여기서 나가면 `_busy` 를 푸는 아래 줄에 못 닿는다 — 잠긴다.
        if success:
            self._terminate(goal_handle, 'succeed')
        elif canceled and goal_handle.is_cancel_requested:
            self._terminate(goal_handle, 'canceled')
        else:
            self._terminate(goal_handle, 'abort')
        text = (f'job={goal_handle.request.job_id} code={code}({error_name(code)}) '
                f'phase={failed_phase} {message}')
        # rclpy 는 같은 호출 위치에서 심각도를 바꾸면 예외를 던진다 → 호출 위치를 나눈다
        if success:
            self.get_logger().info(f'PlaceBook 성공 {text}')
        else:
            self.get_logger().warning(f'PlaceBook 실패 {text}')
        with self._lock:
            self._busy = False
        return result

    def _on_empty_slot(self, msg):
        with self._lock:
            self._empty_observations.append((time.monotonic(), msg))
            self._empty_observations = self._empty_observations[-100:]

    def _on_book(self, msg):
        with self._lock:
            self._book_observations.append((time.monotonic(), msg))
            self._book_observations = self._book_observations[-100:]

    @staticmethod
    def _inside(point, lower, upper):
        return all(lower[i] <= point[i] <= upper[i] for i in range(3))

    def _detection_feedback(self, goal_handle, phase, count=0, confidence=0.0):
        feedback = DetectTargetSlot.Feedback()
        feedback.phase = phase
        feedback.candidate_count = int(count)
        feedback.best_confidence = float(confidence)
        ok, why = publish_feedback_safely(goal_handle, feedback)
        if not ok:
            self._feedback_skipped += 1
            self.get_logger().warning(f'검출 피드백 못 보냄 ({why}) — 누적 {self._feedback_skipped}회')

    def _scan_for_empty_slots(self, goal_handle, job_id, feedback):
        token = uuid.uuid4().hex
        started = time.monotonic()
        with self._lock:
            self._empty_observations.clear()
            self._scan_state = {'token': token, 'status': 'RUNNING', 'phase': 'scan_request'}
        feedback('SCANNING_SHELF', 0)
        self._set_status('RUNNING', job_id, 0.02, 0, 'SCANNING_SHELF')
        self.scan_active_pub.publish(Bool(data=True))      # 비전: 지금부터 빈칸을 봐라
        self._send({'type': self.scan_command, 'token': token,
                    'job_id': f'{job_id}:scan', 'dwell_s': self.scan_dwell_s})

        while time.monotonic() - started < self.scan_timeout_s:
            if goal_handle.is_cancel_requested:
                self._send(cancel_command(token, f'{job_id}:scan'))
                with self._lock:
                    self._scan_state = None
                return None, 412, '서가 스캔 취소'
            self._wake.wait(self.loop_period_s)
            self._wake.clear()
            with self._lock:
                state = dict(self._scan_state or {})
                count = sum(observed_at >= started
                            for observed_at, _ in self._empty_observations)
            feedback(str(state.get('phase', 'SCANNING_SHELF')), count)
            status = str(state.get('status', '')).upper()
            if status == SIM_SUCCEEDED:
                break
            if status in (SIM_FAILED, SIM_CANCELLED):
                with self._lock:
                    self._scan_state = None
                return None, int(state.get('error_code', 410)), \
                    state.get('message', '서가 스캔 실패')
        else:
            self._send(cancel_command(token, f'{job_id}:scan'))
            with self._lock:
                self._scan_state = None
            return None, 404, f'서가 스캔 {self.scan_timeout_s:.0f}s 시간 초과'

        with self._lock:
            slots = [msg for observed_at, msg in self._empty_observations
                     if observed_at >= started]
            self._scan_state = None
        return slots, 0, ''

    def _select_empty_slot(self, slots, book_width):
        """
        스캔 자리(0.75 m)에서 받은 빈칸 관측 → 꽂을 칸 (팔 기준, 꽂기 자리).

        2026-09-23 두 단 스캔 실측(6자세, 관측 5개):
          · 가운데 판(책 있음)은 책 사이 틈이 잡힌다: (-0.43, 0.85, 0.66) — 앞면 0.75 에서 10 cm 안쪽, 정상.
            그런데 가운데 판(팔 기준 z 0.88)은 운반 자세 IK 가 안 풀린다 (레벨 관절 한계가 Lula 보다 좁다).
          · 아래 판은 책이 하나도 없어 카메라가 서가 **너머**(2.9 m)를 본다 — 비전은 이걸 '빈 곳' 으로 준다.
            이건 잘못이 아니라 **판 전체가 비었다**는 관측이다. 그 판(계약 z 0.3399, 삽입 검증된 높이)에 꽂는다.
        선택 순서: 아래 판(비어 있음 관측) → 가운데 판 틈. x 는 팔이 닿는 폭(-0.5~0.2) 으로 자른다.
        """
        contract_y = 0.5495                       # 1차 계약: 꽂힌 책 중심의 팔 기준 y (Franka)
        lower_z, middle_z = 0.3399, 0.3399 + (1.042 - 0.498)
        # **서가 앞면은 '우리가 선 거리' 가 아니다.** `scan_standoff_m` 은 카메라를
        # 어디 둘지이고, 서가가 실제로 어디 있는지는 스캔이 재서 `_shelf_box` 로
        # 보내 준다 (2026-09-24 실측: 서 있는 거리 0.444, 서가 앞면 0.493).
        # 상자가 오면 그걸 쓰고, 없을 때만 예전 어림을 쓴다.
        front_y = float(self._shelf_box[2]) if self._shelf_box else float(self.scan_standoff_m)
        far_y = front_y + 0.30                          # 서가 깊이(0.30) 너머 = 판이 비어 뒤가 보인 것
        # **서가 안에서만 빈칸을 인정한다** (2026-09-23 지적: 서가 밖·옆 서가 틈·바닥을 빈칸으로 오인). 시뮬이 스캔 계획 때
        # 서가 AABB(팔 기준)를 보내 준다. 없으면 파라미터 x 범위·앞면 거리로만 거른다.
        box = self._shelf_box
        if box:
            bx0, bx1 = box[0] + 0.05, box[1] - 0.05             # 옆판 두께·책 반폭 여유
            by0, by1 = box[2] - 0.03, box[3] + 0.05             # 앞면 조금 앞 ~ 뒤판
            bz0, bz1 = box[4], box[5]
        else:
            bx0, bx1 = self.shelf_x_min, self.shelf_x_max
            by0, by1, bz0, bz1 = 0.30, far_y, 0.15, 1.10
        open_board, gaps = [], []
        for msg in slots:
            if msg.header.frame_id != self.limits['frame_id']:
                continue
            x, y, z = float(msg.point.x), float(msg.point.y), float(msg.point.z)
            if y > far_y and bx0 <= x <= bx1:
                # 판이 비어 뒤가 보인 것(서가 x 폭 안에서만) → 검증된 삽입 x 에 꽂는다
                open_board.append((msg, self.place_x))
                continue
            if not (bx0 <= x <= bx1) or not (by0 <= y <= by1) or not (bz0 <= z <= bz1):
                self.get_logger().info(
                    f'빈칸 관측 버림 (서가 밖): ({x:.3f}, {y:.3f}, {z:.3f}) '
                    f'서가 x {bx0:.2f}~{bx1:.2f} y {by0:.2f}~{by1:.2f} z {bz0:.2f}~{bz1:.2f}')
                continue
            zs = lower_z if abs(z - lower_z) <= abs(z - middle_z) else middle_z
            # **틈 관측의 y(깊이)는 못 믿는다 — 틈은 뚫려 있기 때문이다.**
            #
            # 빈칸 한가운데를 찍으면 깊이 센서가 구멍을 통과해 **뒤판이나 그 너머**를
            # 읽는다. 2026-09-24 실측: 서가 앞면이 0.493 인데 관측 y 가 0.707 로
            # **214 mm 뒤**였고, 거기에 책 깊이 절반을 더해 0.813 이 되어 검증 봉투
            # 상한 0.65 를 넘었다 (410). 같은 판의 다른 관측은 y 2.611(서가 뒤 2 m)
            # 까지 나왔다.
            #
            # x·z 는 흔들리지 않는다 (판마다 1~4 mm). **2D 검출은 맞고 깊이만 틀린다.**
            # 그러니 x·z 는 관측을 쓰고, y 는 **우리가 잰 서가 앞면**에서 파생한다 —
            # 치수를 아는 쪽이 유도한다는 규칙 그대로다.
            #
            # 기본은 꺼 둔다. 켜면 관측 y 를 버리므로, 그 판단은 두 값을 나란히 보고
            # 사람이 한다. 로그에 둘 다 찍는다.
            y_gap = y + book_width / 2.0 + self.slot_center_inset
            if self.slot_y_from_shelf_front and self._shelf_box:
                y_front = front_y + book_width / 2.0 + self.slot_center_inset
                self.get_logger().info(
                    f'빈칸 깊이: 관측 y {y:.3f} → 칸중심 {y_gap:.3f} · '
                    f'앞면 실측 {front_y:.3f} → 칸중심 {y_front:.3f} '
                    f'(차이 {(y - front_y) * 1000:+.0f} mm) '
                    f'— **앞면 쪽을 쓴다** [slot_y_from_shelf_front]')
                y_gap = y_front
            elif self._shelf_box and abs(y - front_y) > 0.05:
                self.get_logger().warning(
                    f'빈칸 관측 y {y:.3f} 가 서가 앞면 {front_y:.3f} 에서 '
                    f'{(y - front_y) * 1000:+.0f} mm 다 — 틈을 통과해 뒤를 읽었을 수 있다 '
                    f'(slot_y_from_shelf_front 로 앞면에서 파생할 수 있다)')
            gaps.append((msg, x, y_gap, zs))
        # **책 사이 틈이 있으면 그것을 먼저 쓴다.** '판 비어 있음' 관측은 틈 사이로 뒤가 보인 것일 수 있어(2026-09-23 19:29:
        # 틈 x -0.05 가 있는데 빈 판으로 판단해 x -0.35 에 꽂다 실패) 틈이 하나도 없을 때만 쓴다.
        if open_board and not gaps:
            # **여기서 바로 돌려주지 않는다.** 예전에는 "판이 비었다" 관측이면 실측 빈칸을
            # 안 보고 아래 판 x -0.35 를 목표로 박았다. 2026-09-25 00:15 실측: 비전 관측이
            # 전부 서가 밖으로 걸러진 판에서 이 분기가 열렸고, 아래 판 실측은 14 / 10 mm
            # 조각뿐인데 35.2 mm 책을 밀어 넣으러 가 손에서 1.1 cm 밀렸다(406). "못 한다"
            # 대신 "꽉 찬 데 밀어 넣는다" 가 됐다 — 401 보다 나쁘다. 그래서 이 관측도
            # 아래 정상 경로(실측 빈칸 스냅 → 들어가는 칸만 → 없으면 거절)를 **똑같이**
            # 탄다. 스냅이 꺼져 있거나 실측 빈칸이 없으면 예전과 같은 자리(-0.35, 계약 y,
            # 아래 판)가 된다.
            msg, x = open_board[0]
            self.get_logger().info(
                f'빈칸 선택: 아래 판이 비어 있음 (서가 너머 관측 {len(open_board)}개) '
                f'→ x {x:.3f}, 계약 y {contract_y}, z {lower_z} 를 **후보로** 두고 '
                f'실측 빈칸에 맞춰 본다')
            advance = float(self.scan_standoff_m) - float(self.place_standoff_m)
            gaps.append((msg, x, contract_y + advance, lower_z))
        if not gaps:
            return None
        gaps.sort(key=lambda c: (c[3], abs(c[1])))
        msg, x, center_y, zs = gaps[0]
        # **빈칸 x 를 시뮬이 잰 실제 빈칸에 맞춘다** (`slot_x_snap_to_gap`, 기본 꺼짐).
        #
        # 왜: 비전의 빈칸 x 가 **판마다 160 mm 흔들린다.** 2026-09-24 다섯 판 실측 —
        # 같은 장면·같은 서가인데 −0.063 · −0.058 · −0.053 · −0.047 · +0.097.
        # 그리고 **겹침이 그 흔들림을 1:1 로 따라간다**(r = 0.997): 검출이 10 mm 틀리면
        # 10 mm 겹친다. 팔·이동·배치는 1.5 mm 안에서 정확하다 — 틀린 건 검출 하나다.
        #
        # 깊이(y)는 서가 앞면 실측으로 끌어왔지만 x 는 끌어올 기준이 없었다. 이제
        # 있다 — 시뮬이 서가 책 AABB 에서 **진짜 빈칸**을 재서 보내 준다(`shelf_gaps`).
        # 비전은 "이 근처에 틈이 있다" 고 말하고, 기하가 "틈은 정확히 여기다" 고 말한다.
        #
        # **멀면 손대지 않는다.** 가장 가까운 실측 빈칸이 `slot_x_snap_max_m` 보다
        # 멀면 어느 칸을 본 것인지 알 수 없다 — 엉뚱한 칸으로 끌어당기면 남의 자리에
        # 꽂는다. 그때는 검출을 그대로 두고 뒤쪽 검사에 맡긴다.
        if self.slot_x_snap_to_gap and self._shelf_gaps:
            stale = stale_gap_reason(self._shelf_gaps_at, self._place_count)
            if stale:
                self._slot_reject = stale
                self.get_logger().warning(f'**빈칸 거절** {stale}')
                return None
            # **들어가는 빈칸만 고른다.** 폭을 재 놓고 안 쓰던 자리다 —
            # 2026-09-24 LIVE2 에서 첫 권을 꽂자 59.9 mm 빈칸이 12.5 / 11.0 두
            # 조각이 됐는데, 다음 판이 **11 mm 조각의 중심이 검출에 제일 가깝다**는
            # 이유로 그리로 끌어와 36.4 mm 책을 꽂았다 (22.5 mm 겹침, 409).
            # **같은 판의 빈칸끼리만 견준다.** 3·4번 선반을 다 재면 빈칸이 여러
            # 판에 걸치는데, 판을 안 가리면 3번에서 본 빈칸을 4번 x 로 끌어당긴다.
            # 판 사이 **상대 높이**로 맞춘다 (`zs` 는 칸 중심, 빈칸 태그는 판 윗면).
            # 그리고 **가리킨 판에 자리가 없으면 다른 판을 본다** — 도윤님 목표가
            # 3·4번 선반에 한 권씩인데, 비전은 두 권째에도 아래 판을 가리킨다.
            # 가리킨 판만 보면 **비어 있는 위 판을 두고 거절한다** (2026-09-24 실측).
            gx, gw, _rel, why = choose_gap_any_board(
                self._shelf_gaps, x, float(zs) - lower_z, self.profile.thickness,
                min_clearance=float(self.limits['side_clearance']),
                max_move=self.slot_x_snap_max_m)
            if gx is None:
                self._slot_reject = why
                self.get_logger().warning(f'**빈칸 거절** {why}')
                return None
            if _rel is not None and abs(_rel - (float(zs) - lower_z)) > 1e-6:
                # **판을 옮겼으면 높이도 같이 옮긴다.** 안 바꾸면 *다른 판 높이에
                # 이 판 x* 라는 있지도 않은 자리가 된다.
                _old = float(zs)
                zs = lower_z + _rel
                self.get_logger().warning(
                    f'{why}  칸 중심 z {_old:+.4f} → {zs:+.4f}')
            self.get_logger().info(
                f'빈칸 x: 검출 {x:+.4f} → 실측 빈칸 중심 {gx:+.4f} '
                f'({(gx - x) * 1000:+.1f} mm, 빈칸 폭 {gw * 1000:.1f} mm, '
                f'책 두께 {self.profile.thickness * 1000:.1f} mm, '
                f'남는 여유 한쪽 {(gw - self.profile.thickness) / 2 * 1000:.1f} mm, '
                f'판 상대높이 {0.0 if _rel is None else _rel:+.3f} m) [slot_x_snap_to_gap]')
            x = gx
        # 스캔과 꽂기가 같은 자리(앞면 0.444 m)라 관측 깊이를 그대로 쓴다: 틈 앞 + 책폭/2 + inset = 꽂힌 책 중심.
        # (전에 스캔만 0.75 m 물러났을 땐 그 차이만큼 차체를 옮겼다 — 이제 scan/place standoff 가 같다.)
        advance = float(self.scan_standoff_m) - float(self.place_standoff_m)
        self.place_standoff_dynamic = float(self.place_standoff_m)
        # **꽂는 x 는 검증된 -0.35 로 고정하고 차체를 옆으로 옮겨 틈을 거기에 둔다.** 틈 x -0.05(정면)에서는
        # 운반 자세(carry_rotate) IK 가 안 풀렸다 (2026-09-23) — 1차 시연이 검증한 x 는 -0.51~-0.27 뿐이다.
        self.place_lateral_dynamic = x - self.place_x
        self.get_logger().info(
            f'빈칸 선택: 책 사이 틈 관측 ({x:.3f}, {float(msg.point.y):.3f}, {float(msg.point.z):.3f}) → '
            f'칸 중심 y {center_y - advance:.3f}, z {zs:.3f} (판 높이로 스냅); 꽂기 전 차체를 옆으로 '
            f'{self.place_lateral_dynamic:+.3f} m 옮겨 틈을 x {self.place_x} 에 둔다')
        return msg, (self.place_x, center_y - advance, zs)

    def _finish_detection(self, goal_handle, result, success):
        if success:
            self._terminate(goal_handle, 'succeed')
        elif goal_handle.is_cancel_requested:
            self._terminate(goal_handle, 'canceled')
        else:
            self._terminate(goal_handle, 'abort')
        with self._lock:
            self._busy = False
        return result

    def _execute_slot_detection(self, goal_handle):
        """빈칸 검출도 같이 감싼다 — 여기서 죽어도 노드가 잠긴다."""
        try:
            return self._execute_slot_detection_inner(goal_handle)
        except Exception as exc:      # noqa: BLE001
            code, _phase, msg = internal_failure(exc, traceback.format_exc())
            self.get_logger().error(msg)
            self._force_release('DetectTargetSlot')
            result = DetectTargetSlot.Result()
            result.success = False
            result.error_code = int(code)
            result.message = msg.splitlines()[0]
            self._terminate(goal_handle, 'abort')
            return result

    def _execute_slot_detection_inner(self, goal_handle):
        request = goal_handle.request
        result = DetectTargetSlot.Result()

        def feedback(phase, count):
            self._detection_feedback(
                goal_handle, phase, count, 1.0 if count else 0.0)
        # **스캔 전에 먼저 베이스를 돌린다.** 주행은 yaw 를 고정한 채 오므로 도착 자세는 출발 자세 그대로다.
        # 스캔 자세 표는 서가가 팔 +Y 에 있다고 가정하는데, 회전 없이 스캔하면 팔이 엉뚱한 쪽으로 펴져
        # 트레이·서가에 끼고 그 반작용으로 카트가 밀려났다 (2026-09-23 실측). PlaceBook 앞의 회전은
        # 이미 돌아 있으면 차이 0 이라 그대로 둔다.
        if self.executor_kind == 'sim' and self.align_base_before_work:
            rotate_result = self._rotate_base_before_work(
                goal_handle, request.job_id, feedback=feedback,
                standoff=self.scan_standoff_m)
            if rotate_result is not None:
                result.error_code, result.message = rotate_result
                self._set_status('FAILED', request.job_id, 0.0, result.error_code, result.message)
                return self._finish_detection(goal_handle, result, False)
        slots, code, message = self._scan_for_empty_slots(
            goal_handle, request.job_id, feedback)
        self.scan_active_pub.publish(Bool(data=False))     # 비전: 빈칸 그만 (책 검출 때 점이 뜨면 안 된다)
        if slots is None:
            result.error_code = code
            result.message = message
            self._set_status('FAILED', request.job_id, 0.0, code, message)
            return self._finish_detection(goal_handle, result, False)

        selected = self._select_empty_slot(slots, float(request.book_width))
        result.candidate_count = len(slots)
        if selected is None:
            result.error_code = 410
            result.message = (getattr(self, '_slot_reject', None)
                              or f'두 층 스캔에서 조작 가능 빈 슬롯 없음 (수신 {len(slots)}개)')
            self._slot_reject = None
            self._set_status('FAILED', request.job_id, 0.0, 410, result.message)
            return self._finish_detection(goal_handle, result, False)

        raw, center = selected
        # **스캔은 0.75 m 물러나서 하고 꽂기는 0.36 m 로 다시 붙어서 한다.** 빈칸 좌표는 물러난 자리의
        # 팔 기준이므로, 붙은 뒤의 팔 기준으로 +Y 를 그만큼 당겨 둔다 (순수 평행이동, x·z 는 그대로).
        slot = result.target_slot
        slot.header = raw.header
        slot.pose.position.x, slot.pose.position.y, slot.pose.position.z = center
        slot.pose.orientation.z = math.sqrt(0.5)
        slot.pose.orientation.w = math.sqrt(0.5)
        slot.available_width = 0.0
        slot.available_height = 0.0
        slot.insertion_depth = self.fixed_insertion_depth
        slot.pre_insert_offset = 0.05
        slot.confidence = 1.0
        result.success = True
        result.error_code = 0
        result.message = '두 선반 층 스캔에서 빈 슬롯 중심을 선택했다.'
        self._detection_feedback(goal_handle, 'TRANSFORMING_FRAME', len(slots), 1.0)
        self._set_status('SUCCEEDED', request.job_id, 1.0, 0, 'EMPTY_SLOT_READY')
        self.get_logger().info(
            f'빈 슬롯 스캔 완료: {tuple(round(v, 4) for v in center)}, '
            f'삽입깊이 {self.fixed_insertion_depth:.3f}m')
        return self._finish_detection(goal_handle, result, True)

    def _detect_book_for_place(self, goal_handle, goal):
        with self._lock:
            self._book_observations.clear()
        started = time.monotonic()
        last_trigger = 0.0
        self._feedback(goal_handle, 'DETECTING_BOOK', 0.18)
        while time.monotonic() - started < self.book_detect_timeout_s:
            now = time.monotonic()
            if now - last_trigger >= 0.5:
                self.detect_pub.publish(Bool(data=True))
                last_trigger = now
            if goal_handle.is_cancel_requested:
                return None, 412, '책 검출 취소'
            self._wake.wait(min(self.loop_period_s, 0.1))
            self._wake.clear()
            with self._lock:
                recent = [msg for observed_at, msg in self._book_observations
                          if observed_at >= started
                          and msg.header.frame_id == self.limits['frame_id']]
            if recent:
                # **마지막 프레임 하나를 쓰지 않는다.** 2026-09-24 실측: 한 판 안에서
                # 같은 책의 x 가 50 mm 튀고, 프레임마다 **다른 책**이 뽑히기도 한다.
                # 어느 프레임이 마지막이냐가 결과를 정하고 있었다 (41 mm 어긋나 411).
                _bx, _by, _bz, _why = steady_book(
                    [(m.point.x, m.point.y, m.point.z) for m in recent])
                if _bx is None:
                    self.get_logger().warning(f'책 관측이 안 정해진다: {_why} — 더 본다')
                    continue
                self.get_logger().info(f'책 좌표 정함 ({_bx:+.4f}, {_by:+.4f}, {_bz:+.4f}) — {_why}')
                book_top = (float(_bx), float(_by), float(_bz))
                if not self._inside(book_top, self.vision_grasp_min,
                                    self.vision_grasp_max):
                    return None, 410, \
                        f'비전 책 좌표 {tuple(round(v, 3) for v in book_top)}가 안전 범위 밖'
                grasp = GraspGoal(
                    frame_id=self.limits['frame_id'], top_center=book_top,
                    spine_yaw=0.0, thickness=goal.book_thickness,
                    width=goal.book_width, confidence=1.0, age_s=0.0)
                return replace(goal, grasp=grasp), 0, ''
        return None, 411, \
            f'책 관측 자세에서 {self.book_detect_timeout_s:.0f}s 동안 책 좌표 없음'

    def _auto_perception(self, goal_handle, goal):
        """
        두 선반 층 스캔 뒤 빈 슬롯과 책 좌표를 채운다.

        task_manager는 비전과 통신하지 않는다. 이 함수가 Isaac 실행기에 스캔을
        요청하고, 실행기가 각 정지 자세에서 비전을 트리거한다. 스캔 종료 후 q_home
        (책 관측 자세)에서 책 검출을 별도로 요청한다.
        """
        if self.executor_kind != 'sim':
            return None, 411, '자동 비전 스캔은 executor=sim 에서만 지원한다'

        token = uuid.uuid4().hex
        started = time.monotonic()
        with self._lock:
            self._empty_observations.clear()
            self._scan_state = {'token': token, 'status': 'RUNNING', 'phase': 'scan_request'}
        self._feedback(goal_handle, 'SCANNING_SHELF', 0.02)
        self._set_status('RUNNING', goal.job_id, 0.02, 0, 'SCANNING_SHELF')
        # 서가가 둘이면 어느 서가를 재는지 시뮬이 알아야 한다 — 계약의 shelf_id 를 그대로 싣는다.
        # 꽂기 명령에도 같은 값을 실어 재는 서가와 꽂는 서가가 갈리지 않게 한다.
        self._shelf_id = str(getattr(goal, 'shelf_id', '') or '')
        self._send({'type': self.scan_command, 'token': token,
                    'job_id': f'{goal.job_id}:scan', 'dwell_s': self.scan_dwell_s,
                    'shelf_id': self._shelf_id})

        while time.monotonic() - started < self.scan_timeout_s:
            if goal_handle.is_cancel_requested:
                self._send(cancel_command(token, f'{goal.job_id}:scan'))
                with self._lock:
                    self._scan_state = None
                return None, 412, '서가 스캔 취소'
            self._wake.wait(self.loop_period_s)
            self._wake.clear()
            with self._lock:
                state = dict(self._scan_state or {})
            status = str(state.get('status', '')).upper()
            phase = str(state.get('phase', 'SCANNING_SHELF'))
            self._feedback(goal_handle, phase, 0.10)
            if status == SIM_SUCCEEDED:
                break
            if status in (SIM_FAILED, SIM_CANCELLED):
                with self._lock:
                    self._scan_state = None
                return None, int(state.get('error_code', 410)), \
                    state.get('message', '서가 스캔 실패')
        else:
            self._send(cancel_command(token, f'{goal.job_id}:scan'))
            with self._lock:
                self._scan_state = None
            return None, 404, f'서가 스캔 {self.scan_timeout_s:.0f}s 시간 초과'

        with self._lock:
            slots = [msg for observed_at, msg in self._empty_observations
                     if observed_at >= started]
            self._scan_state = None

        slot_candidates = []
        for msg in slots:
            if msg.header.frame_id != self.limits['frame_id']:
                continue
            # 비전 토픽은 서가 전면의 빈칸 중심이다. 실제 배치 목표는 꽂힌 책의
            # AABB 중심이므로 책 깊이 절반과 작은 안쪽 여유만 더한다.
            center = (float(msg.point.x),
                      float(msg.point.y) + goal.book_width / 2.0 + self.slot_center_inset,
                      float(msg.point.z))
            if self._inside(center, self.limits['place_region_min'],
                            self.limits['place_region_max']):
                slot_candidates.append((msg, center))
        if not slot_candidates:
            return None, 410, f'두 층 스캔에서 조작 가능 빈 슬롯 없음 (수신 {len(slots)}개)'
        slot_msg, slot_center = slot_candidates[-1]

        with self._lock:
            self._book_observations.clear()
        detect_started = time.monotonic()
        last_trigger = 0.0
        book_msg = None
        self._feedback(goal_handle, 'DETECTING_BOOK', 0.18)
        while time.monotonic() - detect_started < self.book_detect_timeout_s:
            now = time.monotonic()
            if now - last_trigger >= 0.5:
                self.detect_pub.publish(Bool(data=True))
                last_trigger = now
            if goal_handle.is_cancel_requested:
                return None, 412, '책 검출 취소'
            self._wake.wait(min(self.loop_period_s, 0.1))
            self._wake.clear()
            with self._lock:
                recent = [msg for observed_at, msg in self._book_observations
                          if observed_at >= detect_started
                          and msg.header.frame_id == self.limits['frame_id']]
            if recent:
                book_msg = recent[-1]
                break
        if book_msg is None:
            return None, 411, f'책 관측 자세에서 {self.book_detect_timeout_s:.0f}s 동안 책 좌표 없음'

        book_top = (float(book_msg.point.x), float(book_msg.point.y), float(book_msg.point.z))
        if not self._inside(book_top, self.vision_grasp_min, self.vision_grasp_max):
            return None, 410, f'비전 책 좌표 {tuple(round(v, 3) for v in book_top)}가 안전 범위 밖'

        slot = SlotGoal(
            frame_id=self.limits['frame_id'], position=slot_center,
            orientation_xyzw=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
            available_width=0.0, available_height=0.0,
            insertion_depth=self.fixed_insertion_depth, pre_insert_offset=0.05,
            confidence=1.0, age_s=0.0)
        grasp = GraspGoal(
            frame_id=self.limits['frame_id'], top_center=book_top, spine_yaw=0.0,
            thickness=goal.book_thickness, width=goal.book_width,
            confidence=1.0, age_s=0.0)
        self.get_logger().info(
            f'비전 연동 완료: 빈 슬롯 {tuple(round(v, 4) for v in slot_center)}, '
            f'책 윗면 {tuple(round(v, 4) for v in book_top)}, 삽입깊이 {self.fixed_insertion_depth:.3f}m')
        return replace(goal, slot=slot, grasp=grasp), 0, ''

    def _rotate_base_before_work(self, goal_handle, job_id, feedback=None,
                                 standoff=None, lateral=None):
        """
        작업 전에 차체를 작업 자세로 돌리고 서가까지 거리·옆자리를 맞춘다.

        성공하면 `None` 을 돌려준다 (호출자가 계속 진행한다). 실패하면
        `(error_code, message)` 다.

        **"90도" 가 아니다.** 목표 각은 `work_yaw_deg` 이고 이 레벨에서는 0 이다 —
        서가와 나란히 서서 팔 +Y 가 앞면을 보게 한다. 90 은 옆 서가 끝을 봤다
        (2026-09-23).
        """
        import uuid as _uuid

        token = _uuid.uuid4().hex
        rotate_cmd = {
            'type': COMMAND_ROTATE_BASE,
            'token': token,
            'job_id': f'{job_id}:rotate',
            'yaw': float(self.work_yaw_deg),
        }
        if standoff is not None:
            rotate_cmd['standoff'] = float(standoff)
        if lateral is not None:
            rotate_cmd['lateral'] = float(lateral)      # 서가 AABB 중앙 대신 이만큼 옆으로

        self.get_logger().info(
            f'베이스 회전 명령 전송 (목표 yaw {self.work_yaw_deg:+.0f}°, 서가 거리 {standoff})')
        # PlaceBook 피드백 헬퍼는 DetectTargetSlot 핸들에 쓰면 TypeError 가 난다 — 호출자가 맞는 피드백을 준다
        if feedback is not None:
            feedback('ROTATING_BASE', 0)
        else:
            self._feedback(goal_handle, 'ROTATING_BASE', 0.0)
        self._set_status('RUNNING', job_id, 0.0, 0, 'ROTATING_BASE')

        with self._lock:
            self._scan_state = {'token': token, 'status': 'RUNNING', 'phase': 'rotate_base'}

        self._send(rotate_cmd)

        # 회전 완료 대기 (최대 10초)
        import time
        started = time.monotonic()
        timeout = 30.0
        while time.monotonic() - started < timeout:
            if goal_handle.is_cancel_requested:
                self._send(cancel_command(token, f'{job_id}:rotate'))
                with self._lock:
                    self._scan_state = None
                return 412, '베이스 회전 취소'
            self._wake.wait(min(self.loop_period_s, 0.1))
            self._wake.clear()
            with self._lock:
                state = dict(self._scan_state or {})
            status = str(state.get('status', '')).upper()
            if status == SIM_SUCCEEDED:
                self.get_logger().info('베이스 회전 완료')
                with self._lock:
                    self._scan_state = None
                return None
            if status in (SIM_FAILED, SIM_CANCELLED):
                with self._lock:
                    self._scan_state = None
                return int(state.get('error_code', 410)), \
                    state.get('message', '베이스 회전 실패')

        # 타임아웃
        self._send(cancel_command(token, f'{job_id}:rotate'))
        with self._lock:
            self._scan_state = None
        return 404, f'베이스 회전 {timeout:.0f}s 시간 초과'

    def _force_release(self, label):
        """
        **무슨 일이 나도 다음 요청은 받는다.** 잡아 둔 것을 전부 푼다.

        `_busy` 를 푸는 곳이 정상 종료 경로에만 있었다. 예외가 실행 콜백 밖으로
        나가면 영원히 잠긴다 — 2026-09-24 에 그래서 6 사이클이 통째로 거절됐다.
        """
        with self._lock:
            self._busy = False
            self._tracker = None
            self._scan_state = None
        self.get_logger().warning(f'{label}: 잡아 둔 상태를 풀었다 — 다음 요청을 받는다')

    def _terminate(self, goal_handle, how):
        """
        목표를 끝낸다. **이미 끝난 목표에 다시 하면 rclpy 가 던진다**.

        그 예외가 우리를 죽이면 안 된다 — 고치려던 병이 바로 그것이다.
        """
        try:
            getattr(goal_handle, how)()
        except Exception as exc:      # noqa: BLE001
            self.get_logger().warning(f'목표 종료({how}) 실패 {type(exc).__name__}: {exc}')

    def _execute(self, goal_handle):
        """**예외가 나도 노드는 산다.** 작업 하나를 잃는 것과 노드를 잃는 것은 다르다."""
        try:
            return self._execute_place(goal_handle)
        except Exception as exc:      # noqa: BLE001 - 노드를 잠그지 않는 것이 먼저다
            code, phase, msg = internal_failure(exc, traceback.format_exc())
            self.get_logger().error(msg)
            self._force_release('PlaceBook')
            result = PlaceBook.Result()
            result.success = False
            result.failed_phase = phase
            result.placement_verified = False
            result.error_code = int(code)
            result.message = msg.splitlines()[0]
            self._set_status('FAILED', getattr(goal_handle.request, 'job_id', ''),
                             0.0, int(code), result.message)
            self._terminate(goal_handle, 'abort')
            return result

    def _execute_place(self, goal_handle):
        request = goal_handle.request
        job_id = request.job_id
        goal = self._to_goal(request)
        vision_owned = not goal.slot.frame_id
        continuous_grasp = vision_owned

        # **작업 시작 전에 franka base를 90도 회전시킨다.**
        # 카트가 서가 앞에 서면 franka 팔이 책장을 정면으로 보게 되는데,
        # 수평 삽입을 위해 base를 90도 회전시켜야 한다.
        # task_manager 가 빈칸(frame_id 있음)을 넘겨줄 때도 **꽂기 거리로 다시 붙어야** 한다 — 스캔은 0.75 m 물러나서
        # 했다. vision_owned 조건 때문에 안 붙어서 허공에 꽂을 뻔했다 (2026-09-23). yaw 는 이미 맞아 차이 0.
        if self.executor_kind == 'sim' and self.align_base_before_work:
            rotate_result = self._rotate_base_before_work(
                goal_handle, job_id,
                standoff=getattr(self, 'place_standoff_dynamic', None) or self.place_standoff_m,
                lateral=getattr(self, 'place_lateral_dynamic', None))
            if rotate_result is not None:
                code, message = rotate_result
                self._set_status('FAILED', job_id, 0.0, code, message)
                return self._finish(goal_handle, False, 'ROTATING_BASE', False,
                                    code, message, canceled=code == 412)

        if vision_owned:
            goal, code, message = self._auto_perception(goal_handle, goal)
            if goal is None:
                self._set_status('FAILED', job_id, 0.0, code, message)
                return self._finish(goal_handle, False, 'DETECTING_BOOK', False,
                                    code, message, canceled=code == 412)
        elif goal.grasp is None and self.auto_detect_book:
            goal, code, message = self._detect_book_for_place(goal_handle, goal)
            if goal is None:
                self._set_status('FAILED', job_id, 0.0, code, message)
                return self._finish(goal_handle, False, 'DETECTING_BOOK', False,
                                    code, message, canceled=code == 412)
            continuous_grasp = True
        self._set_status('RUNNING', job_id, 0.0, 0, 'DETECTING_BOOK')
        self._feedback(goal_handle, 'DETECTING_BOOK', 0.0)

        # 트레이 칸 좌표는 설정값. 비전 관측(grasp)이 오면 **검사를 통과한 것만** 쓴다
        book, check = resolve_book(goal, self.profile, self.limits)
        if check.ok:
            check = validate_goal(goal, book, self.limits)
        if check.ok:
            # **검사 전에** 비전 x 를 칸 중심에 맞춘다 — 트레이는 좌표를 아는 고정 지그다.
            # 비전이 정하는 것은 몇 번 칸인가이고, 그 칸의 x 는 지그가 이미 안다
            goal, note = ((goal, '') if continuous_grasp else
                          snap_grasp_to_slot(goal, self.tray_slots, self.limits))
            if note:
                self.get_logger().info(note)
            # 계약 §5 — 틀린 점을 경계에서 잡는다. 특히 "윗면 높이가 책 규격과 맞는가"
            # 위에서 맞췄어도 이 검사는 그대로 둔다 (마지막 안전선이다)
            check = validate_grasp(
                goal, [] if continuous_grasp else self.tray_slots, self.limits)
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
        command['shelf_id'] = getattr(self, '_shelf_id', '')
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
        # **꽂아 본 판은 빈칸 실측을 낡게 만든다.** 성공했든 아니든 책이 서가 쪽으로
        # 갔으면 그 자리는 더 이상 우리가 잰 그 빈칸이 아니다.
        if any(ph in tracker.history for ph in LEFT_TRAY_PHASES):
            self._place_count += 1
            self.get_logger().info(
                f'빈칸 실측이 낡았다 (배치 {self._place_count}회째) — 다음 판은 다시 스캔해야 한다. '
                f'트레이 쓴 칸 {sorted(self.used_slots)} / 전체 {len(self.tray_slots)}')
        self._set_status('SUCCEEDED' if outcome.success else 'FAILED', job_id, tracker.progress,
                         outcome.error_code, outcome.message or outcome.failed_phase)
        return self._finish(goal_handle, outcome.success, outcome.failed_phase,
                            outcome.placement_verified, outcome.error_code, outcome.message,
                            canceled=outcome.error_code == 412)

    def _feedback(self, goal_handle, phase, progress):
        # **피드백이 작업을 죽이지 않게 한다.** 목표가 이미 끝난 뒤에 보내면 rclpy 가
        # RCLError 를 던지고, 그 예외가 `_execute` 밖으로 튀어 실행 스레드를 끝낸다
        # (2026-09-24 노드 사망 1/16). 장식 때문에 책을 쥔 채로 죽을 수는 없다.
        fb = PlaceBook.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        ok, why = publish_feedback_safely(goal_handle, fb)
        if not ok:
            self._feedback_skipped += 1
            self.get_logger().warning(f'피드백 못 보냄 ({why}) — 누적 {self._feedback_skipped}회')

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
