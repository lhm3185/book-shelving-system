"""vision_manager(트리거 방식)에 검출 요청을 N 번 보내고, arm_base_link 로 나온 책 점을 트레이 칸 정답과 비교한다.

정답: shelving_manipulation/config/book_profiles.yaml 의 tray.slots (책 AABB 중심, arm_base_link).
비전 점은 박스 영역 깊이 중앙값으로 역투영한 점 = 책등 윗면 근처 → z 는 AABB 중심 + 책 폭/2 와 비교.
    python3 trigger_eval.py --n 10
"""
import argparse
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=10)
ap.add_argument("--period", type=float, default=1.5)
ap.add_argument("--profiles", default=os.path.expanduser(
    "~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/config/book_profiles.yaml"))
ap.add_argument("--slot-widths", default="",
                help="여러 종류를 놓았을 때 칸별 책 폭(깊이) m, 쉼표로 구분. "
                     "Isaac 실행기가 시작할 때 찍는 '깊이' 값을 그대로 넣으면 된다. 비우면 기본 프로파일 폭")
args = ap.parse_args()

cfg = yaml.safe_load(open(args.profiles))
truth = np.array([s["center"] for s in cfg["tray"]["slots"]], float)
width = float(cfg["profiles"]["default"]["width"])
widths = [float(v) for v in args.slot_widths.split(",") if v.strip()] or [width] * len(truth)
if len(widths) < len(truth):
    widths += [width] * (len(truth) - len(widths))
top = truth.copy()
for k in range(len(top)):
    top[k, 2] += widths[k] / 2

rclpy.init()
n = Node("trigger_eval")
pts = []
n.create_subscription(PointStamped, "/perception/books", lambda m: pts.append(m), 100)
pub = n.create_publisher(Bool, "/perception/detect_request", 10)
t_end = time.time() + 3
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.1)
for i in range(args.n):
    pub.publish(Bool(data=True))
    t_end = time.time() + args.period
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
print(f"요청 {args.n}회, 책 점 {len(pts)}개, frame {sorted(set(m.header.frame_id for m in pts))}")
if pts:
    a = np.array([[m.point.x, m.point.y, m.point.z] for m in pts])
    for k in range(len(top)):
        d = np.linalg.norm(a[:, :2] - top[k, :2], axis=1)
        near = a[d < 0.04]
        if len(near):
            med = np.median(near, 0); e = med - top[k]
            print(f"칸 {k}: 검출 {len(near):3d}회  중앙값 ({med[0]:+.4f}, {med[1]:+.4f}, {med[2]:+.4f})  "
                  f"오차 dx {e[0]*1000:+.0f} dy {e[1]*1000:+.0f} dz {e[2]*1000:+.0f} mm")
        else:
            print(f"칸 {k}: 검출 없음 (4cm 안)")
    far = a[np.min(np.linalg.norm(a[:, None, :2] - top[None, :, :2], axis=2), axis=1) >= 0.04]
    print(f"어느 칸과도 4cm 밖인 점 {len(far)}개" + (f", 예 {np.round(far[:3], 3).tolist()}" if len(far) else ""))
rclpy.shutdown()
