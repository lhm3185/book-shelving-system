"""Bridge the ROS navigation action to Isaac's JSON navigation executor."""

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shelving_interfaces.action import NavigateToTarget
from std_msgs.msg import String
import yaml


def _yaw_from_quaternion(q):
	return math.atan2(
		2.0 * (q.w * q.z + q.x * q.y),
		1.0 - 2.0 * (q.y * q.y + q.z * q.z),
	)


class NavManager(Node):
	"""Expose NavigateToTarget and forward goals to Isaac Sim."""

	def __init__(self):
		super().__init__('nav_manager')
		self.action_name = self.declare_parameter(
			'target_action', '/navigate_to_target').value
		self.command_topic = self.declare_parameter(
			'command_topic', '/navigation/sim/command').value
		self.state_topic = self.declare_parameter(
			'state_topic', '/navigation/sim/state').value
		self.timeout_s = float(self.declare_parameter(
			'navigation_timeout_sec', 300.0).value)
		self.waypoints_file = self.declare_parameter(
			'waypoints_file', '').value
		self.patrol_route_name = self.declare_parameter(
			'patrol_route', 'library_loop').value
		self.patrol_speed = float(self.declare_parameter(
			'patrol_speed', 0.5).value)
		# 서가 도착 뒤 제자리에서 맞출 월드 yaw(도). 음수 큰 값(-999)이면 안 돌린다
		_wy = float(self.declare_parameter('work_yaw_deg', 0.0).value)
		self.work_yaw_deg = None if _wy < -900 else _wy
		self.patrol_route = self._load_patrol_route()
		self._lock = threading.Lock()
		self._wake = threading.Event()
		self._state = {}
		self._last_home = None
		self._last_home_root = None
		self._active = False
		group = ReentrantCallbackGroup()
		self._command_pub = self.create_publisher(String, self.command_topic, 10)
		self.create_subscription(String, self.state_topic, self._on_state, 50,
								 callback_group=group)
		self._server = ActionServer(
			self, NavigateToTarget, self.action_name,
			execute_callback=self._execute,
			goal_callback=self._goal_callback,
			cancel_callback=lambda _goal: CancelResponse.ACCEPT,
			callback_group=group,
		)
		self.get_logger().info(
			f'Navigation bridge ready: {self.action_name} -> '
			f'{self.command_topic}; patrol={self.patrol_route_name} '
			f'points={len(self.patrol_route)}')

	def _load_patrol_route(self):
		path = self.waypoints_file
		if not path:
			path = str(Path(get_package_share_directory(
				'shelving_navigation')) / 'config' / 'waypoints.yaml')
		with open(path, encoding='utf-8') as stream:
			data = yaml.safe_load(stream) or {}
		waypoints = data.get('waypoints', {})
		self._waypoints = waypoints
		route_names = data.get('routes', {})
		if self.patrol_route_name not in route_names:
			raise ValueError(
				f'patrol route not found: {self.patrol_route_name}')
		route = []
		for name in route_names[self.patrol_route_name]:
			position = waypoints[name]['position']
			route.append([
				float(position['x']),
				float(position['y']),
			])
		return route

	def _goal_callback(self, request):
		if not request.job_id.strip() or not request.target_id.strip():
			return GoalResponse.REJECT
		with self._lock:
			if self._active:
				return GoalResponse.REJECT
			self._active = True
		return GoalResponse.ACCEPT

	def _on_state(self, message):
		try:
			state = json.loads(message.data)
		except (TypeError, ValueError):
			self.get_logger().warning('Invalid Isaac navigation state JSON')
			return
		with self._lock:
			self._state = state
			if state.get('home'):
				self._last_home = state['home']
			# **복귀 목표는 루트 자리다.** `home` 은 팔 베이스라 여기로 주행하면
			# 로봇이 arm_offset 만큼(이 레벨 y 0.30 m) 못 미쳐 선다 — 데크가
			# 트레이에서 그만큼 떨어져 트레이를 다시 못 받는다 (2026-09-24).
			if state.get('home_root'):
				self._last_home_root = state['home_root']
		self._wake.set()

	def _publish(self, command):
		self._command_pub.publish(String(data=json.dumps(command)))

	@staticmethod
	def _yaw_of(waypoint):
		"""waypoints.yaml 한 점의 orientation(사원수) → 월드 yaw(도). 없으면 None."""
		q = (waypoint or {}).get('orientation')
		if not q:
			return None
		x, y, z, w = (float(q.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
		return math.degrees(math.atan2(2.0 * (w * z + x * y),
									   1.0 - 2.0 * (y * y + z * z)))

	def _command_for_goal(self, request):
		if request.target_type == 'shelf':
			return {
				'type': 'patrol',
				'speed': self.patrol_speed,
				'route': self.patrol_route,
			}
		# 작업이 끝나면 **시작점으로 돌아간다** — waypoints.yaml 의 home 좌표로 goto (2026-09-23).
		# 전에는 home 도 '이미 만족' 으로 넘겨 카트가 서가 앞에 남았다.
		if request.target_type == 'home':
			# 시뮬이 알려준 실제 출발 자리를 먼저 쓰고, 없으면 waypoints.yaml 의 home
			with self._lock:
				# 루트 자리를 먼저 쓴다. 옛 시뮬은 안 실어 보내므로 없으면 팔 베이스로
				# 물러서는데, 그때는 arm_offset 만큼 어긋난다는 것을 알려 둔다.
				sim_home = (self._last_home_root or None)
				if sim_home is None and self._last_home:
					sim_home = self._last_home
					self.get_logger().warn(
						'시뮬이 home_root 를 안 보낸다 — 팔 베이스 자리로 복귀한다. '
						'루트와 arm_offset 만큼 어긋날 수 있다')
			wp = (self._waypoints or {}).get('home', {})
			home = wp.get('position')
			# **방향까지 되돌린다.** 자리만 맞추면 서가를 볼 때의 yaw 로 선 채 끝나서,
			# 처음 트레이를 받던 자세와 어긋난다 (2026-09-24 도윤님 지적: 이 레벨은
			# 출발 yaw +90°, 작업 yaw 0° 라 90° 틀어진 채 복귀했다).
			# 시뮬이 실어 보내는 출발 yaw 를 먼저 쓰고, 없으면 waypoints 의 사원수에서 낸다.
			yaw = float(sim_home[2]) if sim_home and len(sim_home) > 2 else self._yaw_of(wp)
			if sim_home:
				cmd = {'type': 'goto', 'x': float(sim_home[0]), 'y': float(sim_home[1])}
				if yaw is not None:
					cmd['yaw_deg'] = yaw
				return cmd
			if home:
				cmd = {'type': 'goto', 'x': float(home['x']), 'y': float(home['y'])}
				if yaw is not None:
					cmd['yaw_deg'] = yaw
				return cmd
		# The simulator already delivers the tray before the ROS job starts.
		# The verified patrol ends at the work position, which is also the
		# manipulation/home position for this simulation scene.
		return None

	def _run_command(self, goal_handle, request, command, label):
		"""시뮬 주행 명령 하나를 보내고 끝을 기다린다 → (성공, 결과 or None)."""
		with self._lock:
			self._state = {}
		self._wake.clear()
		self._publish(command)
		started = time.monotonic()
		while time.monotonic() - started < self.timeout_s:
			if goal_handle.is_cancel_requested:
				self._publish({'type': 'cancel'})
				goal_handle.canceled()
				return False, self._result(request, False, 2002, 'Navigation canceled.')
			self._wake.wait(0.2)
			self._wake.clear()
			with self._lock:
				state = dict(self._state)
			status = str(state.get('status', '')).lower()
			feedback = NavigateToTarget.Feedback()
			feedback.phase = f"{label}:{str(state.get('status', 'NAVIGATING')).upper()}"
			feedback.remaining_distance = float(state.get('remaining', 0.0))
			feedback.position_error = feedback.remaining_distance
			feedback.yaw_error = 0.0
			goal_handle.publish_feedback(feedback)
			if status == 'succeeded':
				return True, None
			if status == 'failed':
				goal_handle.abort()
				return False, self._result(request, False, 2003, state.get('message', 'Navigation failed.'))
		self._publish({'type': 'cancel'})
		goal_handle.abort()
		return False, self._result(request, False, 2004, 'Navigation timed out.')

	def _execute(self, goal_handle):
		request = goal_handle.request
		command = self._command_for_goal(request)
		try:
			if command is None:
				goal_handle.succeed()
				return self._result(request, True, 0, 'Navigation waypoint is already satisfied in simulation.')
			ok, result = self._run_command(goal_handle, request, command, command.get('type', ''))
			if not ok:
				return result
			# **1차 시연 방식**: 한 바퀴 돌아 파지 자리에 서면 시뮬 주행기가 제자리에서 yaw 를 맞춘다(3 s, 트레이 동반).
			# 검증된 파지 자리는 (2.535, -3.019) yaw 0° — 팔 +Y 가 서가 앞면을 본다 (2026-09-22 실측).
			if request.target_type == 'shelf' and self.work_yaw_deg is not None:
				last = self.patrol_route[-1]
				ok, result = self._run_command(goal_handle, request, {
					'type': 'goto', 'x': float(last[0]), 'y': float(last[1]),
					'speed': self.patrol_speed, 'yaw_deg': float(self.work_yaw_deg)}, 'turn')
				if not ok:
					return result
			goal_handle.succeed()
			return self._result(request, True, 0, 'Navigation completed.')
		finally:
			with self._lock:
				self._active = False

	@staticmethod
	def _result(request, success, error_code, message):
		result = NavigateToTarget.Result()
		result.success = success
		result.final_pose = request.target_pose
		result.tolerance_satisfied = success
		result.error_code = error_code
		result.message = message
		return result


def main(args=None):
	rclpy.init(args=args)
	node = NavManager()
	executor = MultiThreadedExecutor(num_threads=4)
	executor.add_node(node)
	try:
		executor.spin()
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
