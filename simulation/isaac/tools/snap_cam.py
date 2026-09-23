"""손목 카메라 /rgb 와 비전 debug 화면을 PNG 한 장씩 저장한다 (검출 요청 한 번 보냄)."""
import os, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from PIL import Image as PilImage

out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/b1_work/logs/snap")
os.makedirs(out, exist_ok=True)
rclpy.init(); n = Node("snap_cam"); got = {}
def save(msg, name):
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding.lower())
    if ch is None: return
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding.lower().startswith("bgr"): a = a[:, :, [2, 1, 0] + ([3] if ch == 4 else [])]
    PilImage.fromarray(a[:, :, :3]).save(os.path.join(out, name)); got[name] = True
n.create_subscription(Image, "/rgb", lambda m: save(m, "wrist_rgb.png"), qos_profile_sensor_data)
n.create_subscription(Image, "/perception/debug_image", lambda m: save(m, "vision_debug.png"), qos_profile_sensor_data)
pub = n.create_publisher(Bool, "/perception/detect_request", 10)
t0 = time.monotonic(); last = 0
while time.monotonic() - t0 < 20 and len(got) < 2:
    if time.monotonic() - last > 1.0: pub.publish(Bool(data=True)); last = time.monotonic()
    rclpy.spin_once(n, timeout_sec=0.1)
print("저장:", sorted(got), "→", out)
