"""가짜 RGB-D 카메라 발행기 — vision_manager 입력(/rgb, /depth, /camera_info)을 다른 PC 에서 흉내 낸다.

세 토픽을 **같은 header.stamp** 로 함께 보낸다 (vision_manager 는 세 토픽 시각이 맞아야 처리한다).

    python3 fake_camera_pub.py                          # Isaac 손목 카메라 캡처 이미지 (트레이 책 6권)
    python3 fake_camera_pub.py --image 파일.jpg          # 원하는 이미지
    python3 fake_camera_pub.py --pattern noise           # 임의 값 (무작위 픽셀) — 검출은 안 나오는 게 정상
    python3 fake_camera_pub.py --depth 0.5 --rate 5 --frame sim_camera

깊이는 전 화면 같은 값(m, 32FC1). 카메라 정보는 학습 데이터·Isaac 과 같은 화각 90.5°, 640×480.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

DEFAULT_IMAGE = os.path.expanduser("~/ws_cobot_pjt/arm/docs/img/wrist_home_rgb.jpg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--pattern", choices=["image", "noise", "gray"], default="image")
    ap.add_argument("--depth", type=float, default=0.5, help="전 화면 깊이 (m)")
    ap.add_argument("--rate", type=float, default=5.0, help="Hz")
    ap.add_argument("--frame", default="sim_camera", help="header.frame_id")
    ap.add_argument("--rgb-topic", default="/rgb")
    ap.add_argument("--depth-topic", default="/depth")
    ap.add_argument("--info-topic", default="/camera_info")
    args = ap.parse_args()

    W, H = 640, 480
    if args.pattern == "image":
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"이미지를 못 읽음: {args.image}")
        rgb = cv2.cvtColor(cv2.resize(bgr, (W, H)), cv2.COLOR_BGR2RGB)
    elif args.pattern == "noise":
        rgb = np.random.default_rng(0).integers(0, 256, (H, W, 3), dtype=np.uint8)
    else:
        rgb = np.full((H, W, 3), 128, np.uint8)
    depth = np.full((H, W), args.depth, np.float32)
    fx = (W / 2) / np.tan(np.radians(90.5) / 2)

    rclpy.init()
    node = Node("fake_camera_pub")
    pub_rgb = node.create_publisher(Image, args.rgb_topic, 10)
    pub_depth = node.create_publisher(Image, args.depth_topic, 10)
    pub_info = node.create_publisher(CameraInfo, args.info_topic, 10)

    info = CameraInfo()
    info.width, info.height = W, H
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fx, 0.0, W / 2, 0.0, fx, H / 2, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, W / 2, 0.0, 0.0, fx, H / 2, 0.0, 0.0, 0.0, 1.0, 0.0]

    img = Image(height=H, width=W, encoding="rgb8", is_bigendian=0, step=W * 3, data=rgb.tobytes())
    dimg = Image(height=H, width=W, encoding="32FC1", is_bigendian=0, step=W * 4, data=depth.tobytes())

    node.get_logger().info(
        f"발행 시작: {args.rgb_topic} {args.depth_topic} {args.info_topic} frame={args.frame} "
        f"pattern={args.pattern} depth={args.depth} m, {args.rate} Hz  (Ctrl+C 로 종료)")
    period = 1.0 / args.rate
    count = 0
    try:
        while rclpy.ok():
            stamp = node.get_clock().now().to_msg()
            for m in (img, dimg, info):
                m.header.stamp = stamp
                m.header.frame_id = args.frame
            pub_rgb.publish(img); pub_depth.publish(dimg); pub_info.publish(info)
            count += 1
            if count % int(max(1, args.rate * 5)) == 0:
                node.get_logger().info(f"{count}프레임 발행, 구독자 rgb={pub_rgb.get_subscription_count()}")
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
