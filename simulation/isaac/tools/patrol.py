"""도서관 테두리를 따라 한 바퀴 돌게 한다 — `waypoints.yaml` 의 경로를 보낸다.

    ROS_DOMAIN_ID=129 python3 patrol.py                    # library_loop 한 바퀴
    ROS_DOMAIN_ID=129 python3 patrol.py --speed 0.8        # 빠르게
    ROS_DOMAIN_ID=129 python3 patrol.py --route library_loop --dry-run

**1단계는 회전 없이 간다** — 모든 점의 yaw 가 0 이고, 몸통을 돌리지 않고 평행이동만 한다.
Ridgeback 은 전방향이라 그렇게 갈 수 있고, 로봇팔·트레이가 같은 방향을 유지하므로
돌아온 뒤 바로 파지할 수 있다.
"""
import argparse
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
WAYPOINTS = os.path.join(REPO, "ros2_ws/src/shelving_navigation/config/waypoints.yaml")


def load_route(path, name):
    """waypoints.yaml 에서 경로를 읽어 [(x, y), ...] 로 돌려준다."""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    routes = data.get("routes") or {}
    if name not in routes:
        raise SystemExit(f"경로 '{name}' 가 없다. 있는 것: {sorted(routes)}")
    points = data.get("waypoints") or {}
    out = []
    for wp in routes[name]:
        if wp not in points:
            raise SystemExit(f"waypoint '{wp}' 가 지도에 없다")
        pos = points[wp].get("position") or {}
        out.append((wp, float(pos.get("x", 0.0)), float(pos.get("y", 0.0))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default="library_loop")
    ap.add_argument("--speed", type=float, default=0.5, help="m/s")
    ap.add_argument("--waypoints", default=WAYPOINTS)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--dry-run", action="store_true", help="보내지 않고 경로만 보여준다")
    a = ap.parse_args()

    route = load_route(a.waypoints, a.route)
    pts = [(x, y) for _, x, y in route]
    total = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
    print(f"경로 '{a.route}' — {len(route)}개 점, 총 {total:.2f} m, {a.speed} m/s "
          f"(약 {total / max(a.speed, 1e-3):.0f}초)")
    for (name, x, y) in route:
        print(f"   {name:16s} ({x:+8.3f}, {y:+8.3f})")
    if a.dry_run:
        print("--dry-run: 보내지 않는다")
        return 0

    rclpy.init()
    node = Node("patrol_client")
    pub = node.create_publisher(String, "/navigation/sim/command", 10)
    seen = []
    node.create_subscription(String, "/navigation/sim/state", lambda m: seen.append(m.data), 20)

    # 발행자가 붙을 때까지 기다린다 — 바로 보내면 디스커버리에서 놓친다
    start = node.get_clock().now()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - start).nanoseconds / 1e9 > 15.0:
            print("**시뮬이 안 보인다** — Isaac 이 떠 있는지, 도메인이 같은지 볼 것")
            return 2

    pub.publish(String(data=json.dumps(
        {"type": "patrol", "speed": a.speed, "route": [[x, y] for _, x, y in route]})))
    print("보냄 — 진행 상황:")

    start = node.get_clock().now()
    last = None
    while (node.get_clock().now() - start).nanoseconds / 1e9 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        while seen:
            st = json.loads(seen.pop(0))
            key = (st.get("status"), st.get("leg"), st.get("message"))
            if key != last:
                last = key
                pos = st.get("pose", [0, 0])
                extra = f" 남은 {st['remaining']:.2f}m" if "remaining" in st else ""
                print(f"  {st.get('status'):10s} {st.get('leg')}/{st.get('legs')} "
                      f"({pos[0]:+.2f}, {pos[1]:+.2f}){extra}  {st.get('message', '')}")
            if st.get("status") in ("succeeded", "failed"):
                return 0 if st["status"] == "succeeded" else 1
    print("시간 초과")
    return 1


if __name__ == "__main__":
    sys.exit(main())
