"""
PlaceBook 액션 서버 통합시험 — 같은 프로세스에서 서버·클라이언트를 띄운다 (Isaac 없이).

실패 입력(잘못된 목표, 파지 실패, 취소, 실행 중 재요청, 실행기 미연결)을 포함한다.
"""

import math
import os
from pathlib import Path
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
import pytest

os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from shelving_interfaces.action import PlaceBook  # noqa: E402
from shelving_interfaces.msg import RobotStatus  # noqa: E402

from shelving_manipulation.manipulation_node import ManipulationNode, SimMockNode  # noqa: E402

PROFILES = str(Path(__file__).resolve().parents[1] / 'config' / 'book_profiles.yaml')
VERIFIED = (-0.3497, 0.5495, 0.3399)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, with_sim_mock=False, **params):
        suffix = uuid.uuid4().hex[:8]
        self.action = f'/test_place_book_{suffix}'
        self.status_topic = f'/test_status_{suffix}'
        base = {
            'action_name': self.action, 'status_topic': self.status_topic,
            'book_profiles_file': PROFILES, 'mock_step_s': 0.02, 'status_rate_hz': 20.0,
            'sim_command_topic': f'/test_cmd_{suffix}', 'sim_state_topic': f'/test_state_{suffix}',
        }
        base.update(params)
        overrides = [Parameter(k, value=v) for k, v in base.items()]
        self.server = ManipulationNode(parameter_overrides=overrides)
        self.client_node = Node(f'test_client_{suffix}')
        self.client = ActionClient(self.client_node, PlaceBook, self.action)
        self.statuses = []
        self.client_node.create_subscription(RobotStatus, self.status_topic,
                                             lambda m: self.statuses.append(m.state), 10)
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.server)
        self.executor.add_node(self.client_node)
        self.sim = None
        if with_sim_mock:
            self.sim = SimMockNode(parameter_overrides=[
                Parameter('sim_command_topic', value=base['sim_command_topic']),
                Parameter('sim_state_topic', value=base['sim_state_topic']),
                Parameter('mock_step_s', value=0.02)])
            self.executor.add_node(self.sim)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_server(timeout_sec=5.0)

    def goal(self, frame='arm_base_link', position=VERIFIED, job='job1', book='book1'):
        g = PlaceBook.Goal()
        g.job_id, g.book_id = job, book
        g.target_slot.header.frame_id = frame
        pos = g.target_slot.pose.position
        pos.x, pos.y, pos.z = position
        g.target_slot.pose.orientation.z = math.sin(math.pi / 4)
        g.target_slot.pose.orientation.w = math.cos(math.pi / 4)
        g.target_slot.confidence = 1.0
        return g

    def send(self, goal, feedback=None):
        done = threading.Event()
        box = {}

        def on_goal(fut):
            box['handle'] = fut.result()
            if not box['handle'].accepted:
                done.set()
                return
            box['handle'].get_result_async().add_done_callback(on_result)

        def on_result(fut):
            box['result'] = fut.result()
            done.set()

        def on_feedback(msg):
            feedback.append(msg.feedback.phase)

        callback = on_feedback if feedback is not None else None
        self.client.send_goal_async(goal, feedback_callback=callback).add_done_callback(on_goal)
        return box, done

    def close(self):
        self.executor.shutdown()
        self.server.destroy_node()
        self.client_node.destroy_node()
        if self.sim:
            self.sim.destroy_node()


@pytest.fixture
def harness_factory():
    made = []

    def make(**kw):
        h = Harness(**kw)
        made.append(h)
        return h
    yield make
    for h in made:
        h.close()


def test_success_with_feedback_and_status(harness_factory):
    h = harness_factory(mock_step_s=0.1)       # 폴링(20Hz)보다 길게 해서 모든 단계를 본다
    phases = []
    box, done = h.send(h.goal(), phases)
    assert done.wait(15.0)
    r = box['result']
    assert r.status == GoalStatus.STATUS_SUCCEEDED
    assert r.result.success and r.result.placement_verified and r.result.error_code == 0
    for phase in ('DETECTING_BOOK', 'PLANNING_GRASP', 'GRASPING', 'INSERTING', 'VERIFYING'):
        assert phase in phases
    time.sleep(0.2)
    assert 'RUNNING' in h.statuses and h.statuses[-1] == 'SUCCEEDED'
    assert h.server.used_slots == {0}


def test_wrong_frame_rejected_with_410(harness_factory):
    h = harness_factory()
    box, done = h.send(h.goal(frame='camera_color_optical_frame'))
    assert done.wait(10.0)
    r = box['result']
    assert r.status == GoalStatus.STATUS_ABORTED
    assert r.result.error_code == 410 and r.result.failed_phase == 'PLANNING_GRASP'
    assert 'frame_id' in r.result.message
    assert h.server.used_slots == set()


def test_grasp_failure_keeps_book_in_tray(harness_factory):
    h = harness_factory(mock_fail_at='lift', mock_fail_code=405)
    box, done = h.send(h.goal())
    assert done.wait(10.0)
    r = box['result']
    assert r.status == GoalStatus.STATUS_ABORTED
    assert r.result.error_code == 405 and r.result.failed_phase == 'GRASPING'
    assert h.server.used_slots == set()      # 책이 트레이를 떠나지 않았으니 같은 칸을 다시 쓴다


def test_insert_failure_marks_slot_used(harness_factory):
    h = harness_factory(mock_fail_at='wedge', mock_fail_code=407)
    box, done = h.send(h.goal())
    assert done.wait(10.0)
    assert box['result'].result.error_code == 407
    assert h.server.used_slots == {0}


def test_cancel_returns_412_after_safe_stop(harness_factory):
    h = harness_factory(mock_step_s=0.3)
    box, done = h.send(h.goal())
    deadline = time.time() + 5
    while 'handle' not in box and time.time() < deadline:
        time.sleep(0.02)
    time.sleep(0.5)
    box['handle'].cancel_goal_async()
    assert done.wait(10.0)
    r = box['result']
    assert r.status == GoalStatus.STATUS_CANCELED
    assert r.result.error_code == 412 and not r.result.success


def test_second_goal_rejected_while_running(harness_factory):
    h = harness_factory(mock_step_s=0.2)
    box1, done1 = h.send(h.goal(job='a'))
    time.sleep(0.4)
    box2, done2 = h.send(h.goal(job='b'))
    assert done2.wait(5.0)
    assert not box2['handle'].accepted
    assert done1.wait(15.0) and box1['result'].result.success


def test_tray_empty_is_411(harness_factory):
    h = harness_factory()
    h.server.used_slots.update(range(6))
    box, done = h.send(h.goal())
    assert done.wait(10.0)
    assert box['result'].result.error_code == 411
    assert box['result'].result.failed_phase == 'DETECTING_BOOK'


def test_sim_executor_over_topics(harness_factory):
    h = harness_factory(with_sim_mock=True, executor='sim')
    box, done = h.send(h.goal())
    assert done.wait(20.0)
    assert box['result'].result.success, box['result'].result.message


def test_sim_executor_missing_is_411(harness_factory):
    h = harness_factory(executor='sim', heartbeat_timeout_s=0.5)
    box, done = h.send(h.goal())
    assert done.wait(10.0)
    r = box['result'].result
    assert r.error_code == 411 and '미연결' in r.message


def test_success_then_failure_on_same_node(harness_factory):
    """성공 뒤 실패 (CLI 에서 발견: 같은 줄에서 로그 심각도를 바꾸면 rclpy 가 예외를 던져 결과가 비었다)."""
    h = harness_factory()
    box, done = h.send(h.goal(job='ok'))
    assert done.wait(10.0) and box['result'].result.success
    box, done = h.send(h.goal(job='bad', frame='base_link'))
    assert done.wait(10.0)
    r = box['result'].result
    assert r.error_code == 410 and 'frame_id' in r.message
    box, done = h.send(h.goal(job='ok2'))
    assert done.wait(10.0) and box['result'].result.success
