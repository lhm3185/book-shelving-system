"""AMR 라이다 점검 — franka_camera.usd 의 RTX 라이다가 스캔맵에 쓸 만한 데이터를 내는지.

확인: /point_cloud 프레임·크기·필드·주기·점 분포, 라이다 프레임이 TF 에 있는지, odom 수신, 위에서 본 점 그림.
    python3 lidar_check.py --seconds 10 --out /tmp/b1_demo
"""
import argparse
import math
import os
import time

import cv2
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

ap = argparse.ArgumentParser()
ap.add_argument("--topic", default="/point_cloud")
ap.add_argument("--seconds", type=float, default=10.0)
ap.add_argument("--out", default="/tmp/b1_demo")
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

rclpy.init()
n = Node("lidar_check")
clouds, stamps = [], []
tf_pairs = {"/tf": set(), "/tf_static": set(), "/World/ridgeback_franka/tf": set()}
odom = []


def on_cloud(m):
    stamps.append(time.time())
    if len(clouds) < 3 or len(stamps) % 10 == 0:
        clouds.append(m)


n.create_subscription(PointCloud2, args.topic, on_cloud, qos_profile_sensor_data)
n.create_subscription(PointCloud2, args.topic, lambda m: None, 10)   # reliable 발행자도 받게
for t in tf_pairs:
    n.create_subscription(TFMessage, t, lambda m, t=t: tf_pairs[t].update((x.header.frame_id, x.child_frame_id) for x in m.transforms), 50)
n.create_subscription(Odometry, "/World/ridgeback_franka/odom", lambda m: odom.append(m), 10)
t0 = time.time()
while time.time() - t0 < args.seconds:
    rclpy.spin_once(n, timeout_sec=0.05)

print(f"== {args.topic}: {len(stamps)} 개 / {args.seconds:.0f}s → {len(stamps) / args.seconds:.1f} Hz")
if not clouds:
    print("수신 없음"); rclpy.shutdown(); raise SystemExit
m = clouds[-1]
print(f"frame_id '{m.header.frame_id}', width {m.width}, height {m.height}, fields {[f.name for f in m.fields]}, stamp {m.header.stamp.sec}.{m.header.stamp.nanosec:09d}")
pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=False)], float)
finite = np.isfinite(pts).all(1)
p = pts[finite]
r = np.linalg.norm(p[:, :2], axis=1) if len(p) else np.array([])
print(f"점 {len(pts)}개, 유한 {finite.sum()}개")
if len(p):
    print(f"x [{p[:, 0].min():.2f}, {p[:, 0].max():.2f}]  y [{p[:, 1].min():.2f}, {p[:, 1].max():.2f}]  z [{p[:, 2].min():.3f}, {p[:, 2].max():.3f}]  (라이다 프레임)")
    print(f"수평 거리 min {r.min():.2f} / 중앙 {np.median(r):.2f} / max {r.max():.2f} m,  0.3 m 안 {int((r < 0.3).sum())}개 (로봇 몸체 반사 의심)")
    ang = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    hist, _ = np.histogram(ang, bins=36, range=(-180, 180))
    print(f"방위 10° 칸별 점 수 (−180→180): {hist.tolist()}")
    print(f"z 고유값 수 {len(np.unique(np.round(p[:, 2], 3)))} (2D 라이다면 1에 가깝다)")
frames = set(); [frames.update(sum(map(list, v), [])) for v in tf_pairs.values()]
for t, v in tf_pairs.items():
    print(f"TF {t}: {len(v)} 쌍" + (f", 예 {sorted(v)[:4]}" if v else ""))
print(f"라이다 frame '{m.header.frame_id}' 이 TF 에 있음: {m.header.frame_id in frames}")
print(f"odom (/World/ridgeback_franka/odom): {len(odom)}개" + (f", child_frame {odom[-1].child_frame_id}, frame {odom[-1].header.frame_id}" if odom else ""))

# 위에서 본 그림 (라이다 프레임, 1 px = 1 cm, 반경 6 m)
S = 1200; img = np.full((S, S, 3), 255, np.uint8); c = S // 2
for rr in range(1, 7):
    cv2.circle(img, (c, c), rr * 100, (220, 220, 220), 1); cv2.putText(img, f"{rr}m", (c + rr * 100 + 2, c - 2), 0, 0.4, (160, 160, 160), 1)
cv2.line(img, (c, c), (c + 60, c), (0, 0, 255), 2); cv2.putText(img, "+x", (c + 62, c + 5), 0, 0.5, (0, 0, 255), 1)
for (x, y, z) in p:
    u, v = int(c + x * 100), int(c - y * 100)
    if 0 <= u < S and 0 <= v < S:
        slice_ = abs(z) < 0.05           # 라이다 높이 ±5 cm = 2D 스캔맵에 쓰일 단면
        img[v, u] = (0, 0, 255) if slice_ else (90, 90, 90)
print(f"라이다 높이 ±5cm 단면 점 {int((np.abs(p[:, 2]) < 0.05).sum()) if len(p) else 0}개 (빨강)")
path = os.path.join(args.out, "lidar_topdown.png"); cv2.imwrite(path, img); print("그림", path)
rclpy.shutdown()
