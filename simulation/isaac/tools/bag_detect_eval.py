"""bag 을 재생하지 않고 직접 읽어 YOLO 검출을 평가한다 — **Isaac 없이** 비전 성능을 보기 위한 도구.

    python3 bag_detect_eval.py --bag tray_full --model book_tray_best.pt --conf 0.75
    python3 bag_detect_eval.py --bag tray_full --model old.pt --expect 6 --save-dir out

무엇을 보여주나
    프레임마다 상자 수, 상자 폭(화면 대비), 신뢰도를 세고
    "여러 권을 한 상자로 묶었는지"(= 폭이 화면의 40 % 이상인 상자) 비율을 낸다.
    --expect 를 주면 그 수와 맞은 프레임 비율(검출률)을 함께 낸다.

전제
    rosbag2_py 와 ultralytics 가 있으면 된다. ROS 노드를 띄우지 않으므로 재생·시간 동기화가 필요 없다.
"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True, help="bag 폴더 (metadata.yaml 이 있는 곳)")
ap.add_argument("--model", required=True)
ap.add_argument("--conf", type=float, default=0.75)
ap.add_argument("--topic", default="/rgb")
ap.add_argument("--expect", type=int, default=0, help="화면에 보여야 하는 책 수 (0 이면 검출률 계산 안 함)")
ap.add_argument("--stride", type=int, default=1, help="n 프레임마다 한 장만 본다")
ap.add_argument("--max-frames", type=int, default=0)
ap.add_argument("--save-dir", default="", help="상자를 그린 그림을 저장할 폴더")
ap.add_argument("--merge-ratio", type=float, default=0.4, help="이 비율보다 넓은 상자는 '묶임' 으로 센다")
args = ap.parse_args()

import cv2  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from ultralytics import YOLO  # noqa: E402

# zstd 로 압축해 녹화했으므로 압축을 아는 reader 를 쓴다 (일반 SequentialReader 는 열지 못한다)
try:
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
except Exception:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
model = YOLO(args.model)
if args.save_dir:
    os.makedirs(args.save_dir, exist_ok=True)

n_frames = n_merged = 0
counts, widths, confs, hits = [], [], [], 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != args.topic:
        continue
    n_frames += 1
    if (n_frames - 1) % args.stride:
        continue
    msg = deserialize_message(data, Image)
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    img = a[:, :, :3][:, :, ::-1].copy() if msg.encoding == "rgb8" else a[:, :, :3].copy()
    res = model(img, conf=args.conf, verbose=False)[0]
    boxes = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else np.zeros((0, 4))
    cf = res.boxes.conf.cpu().numpy() if res.boxes is not None else np.zeros(0)
    counts.append(len(boxes))
    merged = 0
    for b, c in zip(boxes, cf):
        w = (b[2] - b[0]) / msg.width
        widths.append(w)
        confs.append(float(c))
        if w >= args.merge_ratio:
            merged += 1
        if args.save_dir:
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
    n_merged += merged
    if args.expect and len(boxes) == args.expect and merged == 0:
        hits += 1
    if args.save_dir:
        cv2.imwrite(os.path.join(args.save_dir, f"{len(counts):05d}.jpg"), img)
    if args.max_frames and len(counts) >= args.max_frames:
        break

n = len(counts)
print(f"bag {args.bag}")
print(f"모델 {os.path.basename(args.model)}  임계값 {args.conf}  평가 프레임 {n} (전체 {n_frames})")
if n:
    print(f"프레임당 상자 수  평균 {np.mean(counts):.2f}  중앙값 {int(np.median(counts))}  "
          f"최소 {min(counts)}  최대 {max(counts)}")
    print(f"상자 없음 프레임  {sum(1 for c in counts if c == 0)} / {n}")
if widths:
    print(f"상자 폭(화면 대비)  평균 {np.mean(widths) * 100:.1f}%  최대 {max(widths) * 100:.1f}%")
    print(f"묶임 상자(폭 {args.merge_ratio * 100:.0f}% 이상)  {n_merged} 개")
if confs:
    print(f"신뢰도  평균 {np.mean(confs):.2f}  최소 {min(confs):.2f}  최대 {max(confs):.2f}")
if args.expect:
    print(f"정확히 {args.expect}권을 묶임 없이 잡은 프레임  {hits} / {n} = {hits / max(n, 1) * 100:.1f}%")
