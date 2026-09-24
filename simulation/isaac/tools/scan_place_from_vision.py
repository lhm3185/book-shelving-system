"""조작기 스캔 → 비전 좌표 → PlaceBook을 한 번에 검증한다.

비전은 task_manager와 통신하지 않는다. 이 도구가 조작 실행기에 ``scan_shelf``를
보내면 실행기가 각 정지 자세에서 ``/perception/detect_request``를 발행한다.
비전이 발행한 책 윗면 중심과 빈 책장 전면 좌표를 모아 PlaceBook goal로 전달한다.
"""

import argparse
import json
import math
import sys
import time
import uuid

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shelving_interfaces.action import PlaceBook
from std_msgs.msg import Bool, String


BOOK = {'thickness': 0.035, 'height': 0.24, 'width': 0.18}
SLOT_CENTER_INSET = 0.024
PLACE_MIN = (-0.56, 0.45, 0.25)
PLACE_MAX = (-0.22, 0.65, 0.55)


def _inside(point):
    return all(lo <= value <= hi for value, lo, hi in zip(point, PLACE_MIN, PLACE_MAX))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dwell', type=float, default=1.2)
    parser.add_argument('--scan-timeout', type=float, default=180.0)
    parser.add_argument('--place-timeout', type=float, default=180.0)
    parser.add_argument('--book-id', default='vision_book')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = Node('scan_place_from_vision')
    books = []
    empty_fronts = []
    states = []
    node.create_subscription(PointStamped, '/perception/books', books.append, 20)
    node.create_subscription(
        PointStamped, '/perception/empty_shelf_position', empty_fronts.append, 20)
    node.create_subscription(
        String, '/manipulation/sim/state', lambda message: states.append(message.data), 50)
    scan_pub = node.create_publisher(String, '/manipulation/sim/command', 10)
    trigger_pub = node.create_publisher(Bool, '/perception/detect_request', 10)

    started = time.monotonic()
    while scan_pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - started > 15.0:
            print('조작 실행기가 보이지 않는다: /manipulation/sim/command 구독자를 확인할 것')
            return 2

    token = uuid.uuid4().hex
    scan_pub.publish(String(data=json.dumps({
        'type': 'scan_shelf', 'token': token, 'job_id': 'vision_scan',
        'dwell_s': args.dwell,
    })))
    print(f'조작기 스캔 시작: dwell={args.dwell:.1f}s')

    scan_done = False
    started = time.monotonic()
    while time.monotonic() - started < args.scan_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        while states:
            state = json.loads(states.pop(0))
            if state.get('token') != token:
                continue
            status = str(state.get('status', '')).lower()
            phase = state.get('phase', '')
            if phase:
                print(f'  scan {status}: {phase}')
            if status == 'succeeded':
                scan_done = True
                break
            if status in ('failed', 'cancelled'):
                print(f"스캔 실패: {state.get('message', '')}")
                return 1
        if scan_done:
            break
    if not scan_done:
        print('스캔 시간 초과')
        return 1

    # 스캔이 끝나면 실행기가 책 검출 자세(q_home)로 돌아온다. 스캔 중 서가 책이
    # 섞이지 않도록 기존 관측을 버리고, 이 자세에서 트레이 책을 새로 검출한다.
    books.clear()
    started = time.monotonic()
    while not books and time.monotonic() - started < 15.0:
        trigger_pub.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.3)
    if not books:
        print('스캔 후 책 검출 자세에서 비전 책 좌표를 받지 못했다')
        return 2
    book = books[-1]
    print('책 윗면 중심: '
          f'({book.point.x:+.4f}, {book.point.y:+.4f}, {book.point.z:+.4f}) '
          f'frame={book.header.frame_id}')

    candidates = []
    for front in empty_fronts:
        if front.header.frame_id != 'arm_base_link':
            continue
        center = (
            front.point.x,
            front.point.y + BOOK['width'] * 0.5 + SLOT_CENTER_INSET,
            front.point.z,
        )
        if _inside(center):
            candidates.append((front, center))
    if not candidates:
        print(f'조작 가능 범위의 빈 슬롯이 없다 (수신 {len(empty_fronts)}개)')
        return 2

    # 같은 빈칸이 여러 자세에서 보이면 가장 최근 관측을 사용한다.
    front, center = candidates[-1]
    print('빈 슬롯: 책장 전면 '
          f'({front.point.x:+.4f}, {front.point.y:+.4f}, {front.point.z:+.4f})')
    print('삽입 완료 책 중심: '
          f'({center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}), '
          '고정 삽입 깊이=0.3000m')
    if args.dry_run:
        print('--dry-run: PlaceBook은 보내지 않는다')
        return 0

    client = ActionClient(node, PlaceBook, '/place_book')
    if not client.wait_for_server(timeout_sec=15.0):
        print('/place_book 액션 서버가 없다')
        return 2

    goal = PlaceBook.Goal()
    goal.job_id = f'vision_place_{int(time.time())}'
    goal.book_id = args.book_id
    goal.target_slot.header = front.header
    goal.target_slot.pose.position.x = center[0]
    goal.target_slot.pose.position.y = center[1]
    goal.target_slot.pose.position.z = center[2]
    goal.target_slot.pose.orientation.z = math.sqrt(0.5)
    goal.target_slot.pose.orientation.w = math.sqrt(0.5)
    goal.target_slot.insertion_depth = 0.30
    goal.target_slot.pre_insert_offset = 0.05
    goal.target_slot.confidence = 1.0
    goal.book_width = BOOK['width']
    goal.book_height = BOOK['height']
    goal.book_thickness = BOOK['thickness']
    goal.insertion_speed = 0.03
    goal.has_grasp = True
    goal.grasp.header = book.header
    goal.grasp.top_center = book.point
    goal.grasp.spine_yaw = 0.0
    goal.grasp.thickness = BOOK['thickness']
    goal.grasp.width = BOOK['width']
    goal.grasp.confidence = 1.0

    future = client.send_goal_async(
        goal, feedback_callback=lambda feedback: print(
            f'  place: {feedback.feedback.phase} {feedback.feedback.progress:.0%}'))
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        print('PlaceBook goal이 거절됐다')
        return 2
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.place_timeout)
    wrapped = result_future.result()
    if wrapped is None:
        print('PlaceBook 결과 시간 초과')
        return 1
    result = wrapped.result
    print(f'결과: success={result.success}, verified={result.placement_verified}, '
          f'code={result.error_code}, phase={result.failed_phase}')
    print(result.message)
    return 0 if result.success and result.placement_verified else 1


if __name__ == '__main__':
    sys.exit(main())
