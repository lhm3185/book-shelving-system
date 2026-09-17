"""연동 시연 — 손목 카메라 화면에 YOLO 결과를 띄우고, 비전 노드 검출 요청·결과를 함께 보여준다.

    python3 detect_view.py                    창: d = 비전 노드에 검출 요청, s = 화면 저장, q = 종료
    python3 detect_view.py --model <best.pt>

- 창의 박스는 이 도구가 같은 모델로 직접 돌린 결과 (눈으로 보기용)
- 오른쪽 위 글자는 vision_manager 가 /perception/books 로 보낸 최근 책 좌표 (arm_base_link)
"""
import argparse
import os
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--model", default=os.path.expanduser("~/ws_cobot_pjt/arm/models/book_best.pt"))
ap.add_argument("--conf", type=float, default=0.75, help="vision_manager confidence_threshold 와 맞춘다")
ap.add_argument("--save-dir", default="/tmp/b1_demo")
args = ap.parse_args()
os.makedirs(args.save_dir, exist_ok=True)

rclpy.init()
node = Node("detect_view")
bridge = CvBridge()
model = YOLO(args.model)
state = {"img": None, "books": [], "last_req": 0.0}
node.create_subscription(Image, "/rgb", lambda m: state.__setitem__("img", m), 1)
node.create_subscription(PointStamped, "/perception/books",
                         lambda m: state["books"].append((time.time(), m.header.frame_id, m.point)), 50)
req = node.create_publisher(Bool, "/perception/detect_request", 10)
print("창 키: d = 검출 요청, s = 저장, q = 종료")
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.03)
    if state["img"] is None:
        continue
    if not state.get("first"):
        state["first"] = True
        print(f"첫 영상 수신 {state['img'].width}x{state['img'].height} frame {state['img'].header.frame_id}")
    frame = cv2.cvtColor(bridge.imgmsg_to_cv2(state["img"], "rgb8"), cv2.COLOR_RGB2BGR)
    r = model(frame, conf=args.conf, verbose=False)[0]
    view = r.plot()
    recent = [b for b in state["books"] if b[0] >= state["last_req"]] if state["last_req"] else []
    lines = [f"YOLO {len(r.boxes)} (conf>={args.conf})",
             f"vision_manager books since request: {len(recent)}"]
    lines += [f"  {f} x{p.x:+.3f} y{p.y:+.3f} z{p.z:+.3f}" for _, f, p in recent[-6:]]
    for i, t in enumerate(lines):
        cv2.putText(view, t, (8, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.imshow("wrist camera + YOLO", view)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k == ord("d"):
        state["last_req"] = time.time()
        req.publish(Bool(data=True))
        print("검출 요청 보냄")
    if k == ord("s"):
        p = os.path.join(args.save_dir, time.strftime("view_%H%M%S.jpg"))
        cv2.imwrite(p, view); print("저장", p)
cv2.destroyAllWindows()
if rclpy.ok():          # Ctrl+C 로 끝나면 rclpy 가 이미 종료했다
    rclpy.shutdown()
