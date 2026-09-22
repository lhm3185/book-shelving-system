"""비전이 본 책을 그대로 집게 한다 — `/perception/books` → `PlaceBook`.

## 무엇을 하나

1. `/perception/detect_request` 로 검출을 요청한다
2. `/perception/books` 로 오는 **신뢰도 1등 책의 좌표**를 받는다
   (비전이 `publish_best_only=true` 면 오는 것이 곧 1등이다)
3. 그 좌표를 `PlaceBook.grasp.top_center` 에 실어 보낸다
4. 로봇팔이 계약 검사(5절)를 통과시킨 뒤 집어서 꽂는다

**비전 좌표는 "보이는 윗면 중심"이다.** 로봇팔이 책 폭의 절반을 빼 AABB 중심으로
바꾼다 — 유도는 치수를 아는 쪽이 한다 (`grasp_planner.grasp_pick_center`).

    ROS_DOMAIN_ID=129 python3 pick_from_vision.py
    ROS_DOMAIN_ID=129 python3 pick_from_vision.py --goal-x -0.4297 --dry-run

`--dry-run` 은 **좌표만 확인하고 보내지 않는다.** 처음 볼 때 쓰면 좋다.
"""
import argparse
import os
import sys

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shelving_interfaces.action import PlaceBook
from std_msgs.msg import Bool

# 검증된 Franka 꽂을 좌표 (팔 기준). test_contract_coords.py 가 지키는 값이다
FRANKA_SLOTS = [-0.3497, -0.4297, -0.5097, -0.2697]
BOOK = {'thickness': 0.0353, 'height': 0.2374, 'width': 0.1631}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float, default=float(os.environ.get('SIM_GOAL_X', FRANKA_SLOTS[0])), help='꽂을 칸 x (팔 기준)')
    ap.add_argument('--goal-y', type=float, default=float(os.environ.get('SIM_GOAL_Y', 0.5495)))  # 야간: SIM_GOAL_Y (기본 불변)
    ap.add_argument('--goal-z', type=float, default=float(os.environ.get('SIM_GOAL_Z', 0.3399)))
    ap.add_argument('--book-id', default='book_0')
    ap.add_argument('--wait', type=float, default=20.0, help='비전 좌표를 기다릴 시간(초)')
    ap.add_argument('--dry-run', action='store_true', help='좌표만 보고 보내지 않는다')
    a = ap.parse_args()

    rclpy.init()
    node = Node('pick_from_vision')
    seen = []
    node.create_subscription(PointStamped, '/perception/books', seen.append, 10)
    trigger = node.create_publisher(Bool, '/perception/detect_request', 10)

    # 검출 요청을 반복해 보낸다 — 비전은 요청이 있을 때만 검출한다.
    # 한 번만 보내면 디스커버리가 늦어 놓치는 일이 있다 (2026-09-21 실측).
    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < a.wait:
        trigger.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.3)
        if seen:
            break
    if not seen:
        print(f'**비전 좌표가 {a.wait:.0f}초 안에 오지 않았다.** '
              f'vision_manager 가 떠 있는지, 도메인이 같은지 볼 것')
        return 2

    msg = seen[-1]
    p = msg.point
    print(f'비전 좌표(윗면 중심)  frame={msg.header.frame_id}  '
          f'({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f})')
    print(f'→ 로봇팔이 쓸 AABB 중심  ({p.x:+.4f}, {p.y:+.4f}, '
          f'{p.z - BOOK["width"] / 2:+.4f})   (책 폭 {BOOK["width"]} 의 절반을 뺀 값)')
    print(f'꽂을 곳  ({a.goal_x:+.4f}, {a.goal_y:+.4f}, {a.goal_z:+.4f})')
    if a.dry_run:
        print('--dry-run: 보내지 않는다')
        return 0

    client = ActionClient(node, PlaceBook, '/place_book')
    if not client.wait_for_server(timeout_sec=10.0):
        print('**/place_book 액션 서버가 없다** — manipulation_node 를 볼 것')
        return 2

    goal = PlaceBook.Goal()
    goal.job_id = f'vision_{msg.header.stamp.sec}'
    goal.book_id = a.book_id
    goal.target_slot.header.frame_id = 'arm_base_link'
    goal.target_slot.pose.position.x = a.goal_x
    goal.target_slot.pose.position.y = a.goal_y
    goal.target_slot.pose.position.z = a.goal_z
    goal.target_slot.pose.orientation.z = 0.7071068      # yaw +90° (서가 방향)
    goal.target_slot.pose.orientation.w = 0.7071068
    goal.target_slot.confidence = 1.0
    goal.book_thickness = BOOK['thickness']
    goal.book_height = BOOK['height']
    goal.book_width = BOOK['width']
    # **비전 관측을 그대로 싣는다.** 로봇팔이 검사한 뒤 쓴다 (계약 5절)
    goal.has_grasp = True
    goal.grasp.header = msg.header
    goal.grasp.top_center = p
    goal.grasp.thickness = BOOK['thickness']
    goal.grasp.width = BOOK['width']
    goal.grasp.confidence = 1.0

    print('보냄 — 진행 상황:')
    send = client.send_goal_async(
        goal, feedback_callback=lambda f: print(f'  {f.feedback.phase}'))
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('**목표가 거절됐다**')
        return 2
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=180.0)
    res = result_future.result()
    if res is None:
        print('**결과를 못 받았다 (시간 초과)**')
        return 2
    r = res.result
    print(f"결과: success={r.success} code={r.error_code} "
          f"phase={r.failed_phase} verified={r.placement_verified}")
    print(f"  {r.message}")
    return 0 if r.success else 1


if __name__ == '__main__':
    sys.exit(main())
