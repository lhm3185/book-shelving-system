#!/usr/bin/env python3
"""`sensor_msgs/Image` 토픽을 **그림 파일로** 떨군다 — `cv_bridge` 없이.

왜 `cv_bridge` 를 안 쓰나: GPU PC(OpenCV 5 + NumPy 2)에서 못 쓴다. OpenCV 5 가
`CV_CN_SHIFT` 를 3 → 5 로 바꿔서 `CV_8UC3` 상수가 달라졌고, `KeyError` 나 세그폴트가
난다 (2026-09-21 실측, `docs/doyoon-kim/manipulation/TRAPS_20260921.md` 2절).
`Image` 는 그냥 바이트 배열이므로 직접 풀면 된다.

    python3 topic_shot.py --topic /perception/debug_image --out /tmp/shots/dbg
    python3 topic_shot.py --topic /perception/debug_image --count 5 --every 1.0

`--count` 를 2 이상 주면 연속으로 여러 장을 받는다 — **한 장으로는 흔들리는지
모른다.** 검출이 판마다 튀는 것을 볼 때는 여러 장이 필요하다.
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_array(msg):
    """`Image` → HxWx3 BGR 배열. 흔한 인코딩만 다룬다."""
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1].copy() if enc == "rgb8" else a.copy()
    if enc in ("rgba8", "bgra8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4)[:, :, :3]
        return a[:, :, ::-1].copy() if enc == "rgba8" else a.copy()
    if enc == "mono8":
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
        return np.dstack([a] * 3)
    if enc in ("32FC1", "16UC1"):
        # 깊이 영상 — 보이게 정규화한다. **절대값을 보려면 이 그림을 쓰지 말 것**
        d = np.frombuffer(msg.data, np.float32 if enc == "32FC1" else np.uint16)
        d = d.reshape(msg.height, msg.width).astype(np.float32)
        ok = np.isfinite(d) & (d > 0)
        if not ok.any():
            return np.zeros((msg.height, msg.width, 3), np.uint8)
        lo, hi = float(d[ok].min()), float(d[ok].max())
        g = np.zeros_like(d)
        if hi > lo:
            g[ok] = (d[ok] - lo) / (hi - lo) * 255.0
        return np.dstack([g.astype(np.uint8)] * 3)
    raise RuntimeError(f"모르는 인코딩 {enc} — 이 도구에 추가할 것")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", default="/tmp/b1_shots/shot")
    ap.add_argument("--count", type=int, default=1, help="몇 장 받을 것인가")
    ap.add_argument("--every", type=float, default=0.5, help="장 사이 최소 간격(초)")
    ap.add_argument("--seconds", type=float, default=60.0, help="기다릴 시간")
    a = ap.parse_args()

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    import cv2

    rclpy.init()
    node = Node("topic_shot")
    got, last = [], [0.0]

    def on_image(msg):
        now = time.time()
        if len(got) < a.count and now - last[0] >= a.every:
            last[0] = now
            got.append(msg)

    node.create_subscription(Image, a.topic, on_image, 10)
    print(f"{a.topic} 를 {a.seconds:.0f}초 기다린다 ({a.count}장) …")
    start = time.time()
    while len(got) < a.count and time.time() - start < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not got:
        print(f"**{a.seconds:.0f}초 안에 한 장도 안 왔다.** "
              f"토픽 이름·도메인·발행자를 볼 것: ros2 topic info {a.topic}")
        return 2

    for i, msg in enumerate(got):
        path = f"{a.out}_{i:02d}.png" if a.count > 1 else f"{a.out}.png"
        img = to_array(msg)
        cv2.imwrite(path, img)
        print(f"  {path}  {msg.width}x{msg.height} {msg.encoding} "
              f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec // 10**6:03d} "
              f"frame={msg.header.frame_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
