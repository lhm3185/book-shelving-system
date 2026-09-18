"""vision_manager 가 카메라 광학 좌표로 낸 책 점을 TF 로 arm_base_link 로 바꿔 트레이 정답과 비교"""
import time, threading, rclpy, numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
rclpy.init(); n = Node("books_eval", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer(); TransformListener(buf, n, spin_thread=True)
raw = []
n.create_subscription(PointStamped, "/perception/books", lambda m: raw.append(m), 100)
ex = MultiThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
time.sleep(40)
print("points", len(raw), "frames", set(m.header.frame_id for m in raw))
out = []
for m in raw[-300:]:
    try:
        tf = buf.lookup_transform("arm_base_link", m.header.frame_id, rclpy.time.Time())   # 팔 정지 상태라 최신 TF 사용
        p = do_transform_point(m, tf).point; out.append((p.x, p.y, p.z, m.point.z))
    except Exception as e:
        print("tf fail", e); break
TRUTH_X = [-0.6623, -0.5873, -0.5123, -0.4373, -0.3623, -0.2873]
TOP_Z = 0.1119 + 0.1631 / 2          # 책등 윗면 (책 AABB 중심 + 폭/2)
if out:
    a = np.array(out); order = np.argsort(a[:, 0]); groups = []
    for v in a[order]:
        if groups and abs(v[0] - groups[-1][-1][0]) < 0.03: groups[-1].append(v)
        else: groups.append([v])
    for g in groups:
        g = np.array(g); med = np.median(g, 0); k = int(np.argmin([abs(med[0] - t) for t in TRUTH_X]))
        print("검출 x %.4f y %.4f z %.4f (카메라 깊이 %.3f) n %3d | 정답 칸 %d x %.4f y 0.0788 윗면 z %.4f | 오차 dx %+.4f dy %+.4f dz %+.4f"
              % (med[0], med[1], med[2], med[3], len(g), k, TRUTH_X[k], TOP_Z, med[0] - TRUTH_X[k], med[1] - 0.0788, med[2] - TOP_Z))
rclpy.shutdown()
