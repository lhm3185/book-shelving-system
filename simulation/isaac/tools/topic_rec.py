#!/usr/bin/env python3
"""`sensor_msgs/Image` 토픽 여럿을 **받는 즉시** PNG 로 떨구고 받은 시각을 적는다.

    python3 topic_rec.py --out ~/b1_rec_0929/raw --seconds 1200 \
        --topic dbg_rgb=/perception/debug_image --topic depth=/perception/depth_debug_image

`cv_bridge` 는 쓰지 않는다 — 이 PC 의 OpenCV 5 + NumPy 2 에서 깨진다
(`topic_shot.py` 머리말 참고). `Image` 는 바이트 배열이라 직접 푼다.

깊이(32FC1/16UC1)는 **0 ~ 2 m 고정 범위**를 컬러맵으로 칠한다 — 판마다 대비가
달라지지 않게. 그림으로 절대값을 읽지 말 것.

프레임마다 `<이름>.csv` 에 `index,wall_time` 을 적는다. 뒤에 ffmpeg 로 실제 간격대로 잇는다.
"""
import argparse, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2

DEPTH_MAX_M = 2.0


def to_bgr(msg):
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
        if enc == "32FC1":
            d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).astype(np.float32)
        else:
            d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        ok = np.isfinite(d) & (d > 0)
        g = np.zeros(d.shape, np.uint8)
        g[ok] = np.clip(d[ok] / DEPTH_MAX_M, 0, 1)[...] * 255
        col = cv2.applyColorMap(g, cv2.COLORMAP_TURBO)
        col[~ok] = 0                      # 값 없는 화소는 검게
        return col
    raise RuntimeError(f"모르는 인코딩 {enc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", action="append", required=True, help="이름=/토픽")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=1200.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node("topic_rec")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    subs, state = [], {}
    for spec in a.topic:
        name, topic = spec.split("=", 1)
        d = os.path.join(os.path.expanduser(a.out), name)
        os.makedirs(d, exist_ok=True)
        st = {"n": 0, "dir": d, "csv": open(os.path.join(d, "index.csv"), "w", buffering=1)}
        st["csv"].write("index,wall_time\n")
        state[name] = st

        def make(nm):
            def cb(msg):
                s = state[nm]
                try:
                    img = to_bgr(msg)
                except Exception as e:      # 인코딩이 뜻밖이면 그 토픽만 조용히 거른다
                    if s["n"] == 0:
                        node.get_logger().warn(f"{nm}: {e}")
                    return
                p = os.path.join(s["dir"], f"f{s['n']:06d}.png")
                cv2.imwrite(p, img)
                s["csv"].write(f"{s['n']},{time.time():.3f}\n")
                s["n"] += 1
            return cb
        subs.append(node.create_subscription(Image, topic, make(name), qos))
        print(f"구독 {name} ← {topic}", flush=True)

    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    for nm, s in state.items():
        s["csv"].close()
        print(f"{nm}: {s['n']}장 → {s['dir']}", flush=True)
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
