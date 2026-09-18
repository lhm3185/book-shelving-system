"""창 없이 손목 카메라 한 장을 받아 YOLO 로 검출하고, 박스 수·크기와 그림 파일을 남긴다.

트레이 책이 한 덩어리로 묶여 인식되는지(= 큰 박스 1개) 수치로 보려고 만들었다.
    python3 tray_detect_shot.py --out /tmp/b1_demo/mixed.jpg
"""
import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

ap = argparse.ArgumentParser()
ap.add_argument("--model", default=os.path.expanduser("~/ws_cobot_pjt/arm/models/book_best.pt"))
ap.add_argument("--conf", type=float, default=0.75)
ap.add_argument("--topic", default="/rgb")
ap.add_argument("--out", default="/tmp/b1_demo/tray_detect.jpg")
ap.add_argument("--seconds", type=float, default=20.0)
args = ap.parse_args()

import cv2  # noqa: E402
from ultralytics import YOLO  # noqa: E402

rclpy.init()
node = Node("tray_detect_shot")
frame = {}


def on_image(msg):
    if msg.encoding in ("rgb8", "bgr8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        frame["img"] = a[:, :, ::-1].copy() if msg.encoding == "rgb8" else a.copy()


node.create_subscription(Image, args.topic, on_image, 10)
t_end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
while "img" not in frame and node.get_clock().now().nanoseconds < t_end:
    rclpy.spin_once(node, timeout_sec=0.2)
rclpy.shutdown()
if "img" not in frame:
    raise SystemExit(f"{args.topic} 에서 영상이 오지 않는다")

img = frame["img"]
res = YOLO(args.model)(img, conf=args.conf, verbose=False)[0]
boxes = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else np.zeros((0, 4))
confs = res.boxes.conf.cpu().numpy() if res.boxes is not None else np.zeros(0)
h, w = img.shape[:2]
print(f"영상 {w}x{h}, 임계값 {args.conf}, 박스 {len(boxes)}개")
for i, (b, c) in enumerate(zip(boxes, confs)):
    bw, bh = b[2] - b[0], b[3] - b[1]
    print(f"  {i}: 신뢰도 {c:.2f}  폭 {bw:4.0f}px ({bw / w * 100:4.1f}%)  높이 {bh:4.0f}px  "
          f"중심 ({(b[0] + b[2]) / 2:.0f}, {(b[1] + b[3]) / 2:.0f})")
    cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
    cv2.putText(img, f"{c:.2f}", (int(b[0]), max(12, int(b[1]) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
os.makedirs(os.path.dirname(args.out), exist_ok=True)
cv2.imwrite(args.out, img)
print("저장", args.out)
