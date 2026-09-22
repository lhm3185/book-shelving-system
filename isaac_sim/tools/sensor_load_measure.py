"""센서 부하 측정 — PlaceBook 1회 동안 /rgb·/point_cloud 수신 주기를 단계별로 기록한다.

    python3 sensor_load_measure.py --x -0.3497      (manipulation_node executor:=sim 이 떠 있어야 함)
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from shelving_interfaces.action import PlaceBook
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument("--x", type=float, default=-0.3497)
ap.add_argument("--idle", type=float, default=8.0)
args = ap.parse_args()

rclpy.init()
n = Node("sensor_load_measure")
ev = {"rgb": [], "cloud": []}
phase = {"now": "idle", "log": []}
n.create_subscription(Image, "/rgb", lambda m: ev["rgb"].append((time.time(), phase["now"])), qos_profile_sensor_data)
n.create_subscription(PointCloud2, "/point_cloud", lambda m: ev["cloud"].append((time.time(), phase["now"])), qos_profile_sensor_data)


def on_state(m):
    try:
        d = json.loads(m.data)
    except ValueError:
        return
    if d.get("status") == "RUNNING" and d.get("phase") and d["phase"] != phase["now"]:
        phase["now"] = d["phase"]; phase["log"].append((time.time(), d["phase"]))
    if d.get("status") in ("SUCCEEDED", "FAILED", "CANCELLED") and phase["now"] != "done":
        phase["now"] = "done"; phase["log"].append((time.time(), "done")); phase["final"] = d


n.create_subscription(String, "/manipulation/sim/state", on_state, 50)
t0 = time.time()
while time.time() - t0 < args.idle:
    rclpy.spin_once(n, timeout_sec=0.05)
cli = ActionClient(n, PlaceBook, "/place_book"); cli.wait_for_server(timeout_sec=10)
g = PlaceBook.Goal(); g.job_id = f"load_{int(time.time())}"; g.book_id = "b"
g.target_slot.header.frame_id = "arm_base_link"
p = g.target_slot.pose.position; p.x, p.y, p.z = args.x, 0.5495, 0.3399
g.target_slot.pose.orientation.z = math.sin(math.pi / 4); g.target_slot.pose.orientation.w = math.cos(math.pi / 4)
t_goal = time.time()
fut = cli.send_goal_async(g); rclpy.spin_until_future_complete(n, fut)
res = fut.result().get_result_async(); rclpy.spin_until_future_complete(n, res, timeout_sec=300)
t_done = time.time()
t_end = t_done + 4
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
r = res.result().result
print(f"결과 success={r.success} code={r.error_code} 소요(goal→result) {t_done - t_goal:.1f}s")
fin = phase.get("final", {})
print(f"시뮬 스텝 {fin.get('sim_steps')}  렌더 스텝 {fin.get('render_steps')}")


def rate(key, a, b):
    k = [t for t, _ in ev[key] if a <= t < b]
    return len(k) / max(b - a, 1e-6)


carry = next((t for t, ph in phase["log"] if ph == "carry_rotate"), None)
print(f"/rgb        대기 {rate('rgb', t0, t_goal):5.1f} Hz | 작업 전반(~파지) {rate('rgb', t_goal, carry or t_done):5.1f} Hz | 파지 뒤 {rate('rgb', carry or t_done, t_done):5.1f} Hz | 작업 후 {rate('rgb', t_done + 1, t_end):5.1f} Hz")
print(f"/point_cloud 대기 {rate('cloud', t0, t_goal):5.1f} Hz | 작업 전반(~파지) {rate('cloud', t_goal, carry or t_done):5.1f} Hz | 파지 뒤 {rate('cloud', carry or t_done, t_done):5.1f} Hz | 작업 후 {rate('cloud', t_done + 1, t_end):5.1f} Hz")
print(f"파지 확인(carry_rotate 시작) goal 후 {carry - t_goal:.1f}s" if carry else "carry_rotate 못 봄")
rclpy.shutdown()
