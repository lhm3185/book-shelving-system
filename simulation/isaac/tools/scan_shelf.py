"""서가를 훑어 빈칸을 찾게 한다 — 팔만 움직이고 베이스는 그대로 둔다.

    ROS_DOMAIN_ID=130 python3 scan_shelf.py                 # 기본 자세 9개
    ROS_DOMAIN_ID=130 python3 scan_shelf.py --dwell 1.5     # 자세마다 더 오래 멈춘다

각 자세에서 멈추는 동안 비전이 찍는다. 움직이는 중에 찍으면 깊이가 흐려진다.

**베이스를 움직이지 않는 이유**: 비전이 기억한 빈칸 좌표를 `arm_base_link` 그대로
쓰기 위해서다. 베이스가 움직이면 그 사이에 좌표계가 틀어진다
(2026-09-21 실측: 주행 뒤 팔 베이스가 8.1 cm·4.37° 밀렸다).
"""
import argparse
import json
import sys
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dwell", type=float, default=1.0, help="자세마다 멈추는 시간 (초)")
    ap.add_argument("--command-topic", default="/manipulation/sim/command")
    ap.add_argument("--state-topic", default="/manipulation/sim/state")
    ap.add_argument("--timeout", type=float, default=180.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node("scan_shelf_client")
    pub = node.create_publisher(String, a.command_topic, 10)
    seen = []
    node.create_subscription(String, a.state_topic, lambda m: seen.append(m.data), 20)

    # 구독자가 붙을 때까지 기다린다 — 바로 보내면 디스커버리에서 놓친다
    start = node.get_clock().now()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - start).nanoseconds / 1e9 > 15.0:
            print("**시뮬이 안 보인다** — Isaac 이 떠 있는지, 도메인이 같은지 볼 것")
            return 2

    token = uuid.uuid4().hex
    pub.publish(String(data=json.dumps(
        {"type": "scan_shelf", "token": token, "job_id": "scan", "dwell_s": a.dwell})))
    print(f"스캔 명령 보냄 (dwell {a.dwell}초)")

    start = node.get_clock().now()
    last = None
    while (node.get_clock().now() - start).nanoseconds / 1e9 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        while seen:
            st = json.loads(seen.pop(0))
            if st.get("token") != token:
                continue
            if st.get("phase") == "scan_plan":
                print("계획한 자세:")
                for b in st.get("boards", []):
                    mark = "꽂을 수 있음" if b.get("reachable") else "**스캔만** (팔이 못 닿는 판)"
                    print(f"   {b['name']:18s} 선반판 {b['board_z']:.3f}  {mark}")
                continue
            key = (st.get("status"), st.get("phase"))
            if key != last:
                last = key
                print(f"  {st.get('status'):10s} {st.get('phase', '')}  {st.get('message', '')}")
            if st.get("status") in ("succeeded", "failed", "cancelled"):
                ok = st["status"] == "succeeded"
                print("**스캔 끝**" if ok else f"**스캔 실패** {st.get('message', '')}")
                return 0 if ok else 1
    print("시간 초과")
    return 1


if __name__ == "__main__":
    sys.exit(main())
