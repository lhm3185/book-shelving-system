import sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
rclpy.init(); n = Node("grab"); b = CvBridge(); got = {}; frames = set()
n.create_subscription(Image, "/rgb", lambda m: got.setdefault("rgb", m), 5)
n.create_subscription(Image, "/depth", lambda m: got.setdefault("depth", m), 5)
n.create_subscription(CameraInfo, "/camera_info", lambda m: got.setdefault("info", m), 5)
def on_tf(m):
    for t in m.transforms: frames.add((t.header.frame_id, t.child_frame_id))
n.create_subscription(TFMessage, "/tf", on_tf, 50)
t0 = time.time()
while time.time() - t0 < 25 and not (len(got) == 3 and frames):
    rclpy.spin_once(n, timeout_sec=0.2)
for k, m in got.items():
    if k == "info": print("info", m.header.frame_id, m.width, m.height, "K", [round(v, 1) for v in m.k])
    else: print(k, m.header.frame_id, m.encoding, m.width, m.height, "stamp", m.header.stamp.sec, m.header.stamp.nanosec)
print("tf pairs", sorted(frames)[:40])
out = sys.argv[1]
if "rgb" in got: cv2.imwrite(out + "/wrist_rgb.jpg", cv2.cvtColor(b.imgmsg_to_cv2(got["rgb"], "rgb8"), cv2.COLOR_RGB2BGR))
if "depth" in got:
    d = b.imgmsg_to_cv2(got["depth"], "passthrough"); f = d[np.isfinite(d)]
    print("depth m min/med/max", round(float(f.min()), 3), round(float(np.median(f)), 3), round(float(f.max()), 3))
