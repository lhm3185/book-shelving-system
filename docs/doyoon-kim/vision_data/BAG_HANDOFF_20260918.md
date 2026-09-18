# Isaac 없이 비전 시험하기 — bag 과 부속 자료 (2026-09-18, 로봇팔 D 김도윤 → 비전 B 윤재민)

Isaac Sim 이나 GPU PC 없이, **이 폴더만으로** 손목 RealSense 화면을 그대로 재생해 인식률을 재고 코드를 고칠 수 있습니다.
같은 USB 의 `260918_train/` 은 **학습용**(데이터셋·모델), 이 폴더는 **시험·구현용**(녹화 데이터)입니다.

## 1. 폴더 구성

```
260918_bag_vision/
  README.md
  bags/
    tray_full_6books/    트레이에 책 6권, 로봇 정지. 120초, /rgb 830장          630 MB
    tray_during_place/   로봇이 책을 꽂는 동안 (책이 줄어드는 장면). 90초, 582장  477 MB
  model/book_tray_best.pt   새로 학습한 모델 (260918_train 과 같은 파일)
  tools/
    bag_detect_eval.py      bag 을 바로 읽어 검출 성능을 수치로 내는 도구 (노드 실행 없이)
    record_vision_bag.sh    bag 을 다시 뜰 때 쓰는 녹화 스크립트 (GPU PC 용)
  meta/
    camera_info.yaml             카메라 내부 파라미터 (fx 317.04, cx 320, 640×640, 가로 화각 90.5°)
    tf_frames.txt                bag 에 들어 있는 좌표 프레임 관계
    tray_slots_book_profiles.yaml 트레이 6칸의 정답 좌표(arm_base_link 기준) + 책 기본 치수
  samples/    프레임 예시 (새 모델 상자 그린 것, 기존 모델 비교용 1장)
  results/    두 모델의 측정 결과 텍스트
```

## 2. bag 에 들어 있는 토픽

| 토픽 | 형식 | 비고 |
| --- | --- | --- |
| `/rgb` | `sensor_msgs/Image` (rgb8, 640×640) | 손목 RealSense 컬러. 약 7 Hz |
| `/depth` | `sensor_msgs/Image` (32FC1, m) | 같은 시점의 깊이 |
| `/camera_info` | `sensor_msgs/CameraInfo` | frame_id `sim_camera` |
| `/tf`, `/tf_static` | | `panda_link0 → Camera_...Color`, `Camera_...Color → sim_camera`(항등), `panda_link0 → arm_base_link`(항등) 포함 |
| `/clock` | | 시뮬 시간. 재생할 때 `use_sim_time` 과 함께 |

**좌표 변환에 필요한 TF 가 bag 안에 모두 들어 있습니다.** 따로 static_transform_publisher 를 띄울 필요가 없습니다.

## 3. 쓰는 법 (셋 중 편한 것)

### (a) 노드 없이 수치만 — 가장 빠름

```bash
source /opt/ros/jazzy/setup.bash          # rosbag2_py 가 필요
python3 tools/bag_detect_eval.py --bag bags/tray_full_6books \
        --model model/book_tray_best.pt --conf 0.75 --expect 6 --stride 5
```

출력: 프레임당 상자 수, 상자 폭(화면 대비), 신뢰도, **"6권을 묶임 없이 잡은 프레임 비율"**.
`--save-dir out` 을 주면 상자를 그린 그림도 저장합니다. 기존 모델과 바로 비교해 보세요.

### (b) 실제 노드로 — vision_manager 를 그대로 검증

```bash
# 터미널 1 — 재생 (반복 재생: --loop)
source /opt/ros/jazzy/setup.bash
ros2 bag play bags/tray_full_6books --clock --loop

# 터미널 2 — 비전 노드 (시뮬 시간 사용)
ros2 run shelving_perception vision_manager --ros-args \
    --params-file <perception.yaml> -p model_path:=<이 폴더>/model/book_tray_best.pt \
    -p confidence_threshold:=0.75 -p use_sim_time:=true

# 터미널 3 — 검출 요청, 결과 보기
ros2 topic pub --once /perception/detect_request std_msgs/Bool "{data: true}"
ros2 topic echo /perception/books
```

### (c) 그림만 보고 싶을 때

```bash
ros2 bag play bags/tray_full_6books --clock --loop
ros2 run rqt_image_view rqt_image_view /rgb
```

## 4. 이 bag 으로 낸 기준 수치 (참고값)

`bags/tray_full_6books`, 화면에 책 6권이 계속 보이는 구간입니다.

| 모델 | 임계값 | 프레임당 상자 | 상자 폭(화면 대비) | 묶임 상자 | 6권 정확히 잡은 프레임 |
| --- | --- | --- | --- | --- | --- |
| 기존 `book_best.pt` | 0.75 | **0.00** | — | — | **0 %** |
| 기존 `book_best.pt` | 0.25 | 5.51 | 평균 **62.6 %** (최대 84.7 %) | 374개 | **0 %** |
| **새 `book_tray_best.pt`** | 0.75 | **6.00** (항상 6) | 평균 **7.4 %** | **0개** | **100 %** (166/166) |

"묶임 상자" = 폭이 화면의 40 % 이상인 상자. 기존 모델이 트레이 전체를 한 덩어리로 보던 문제를 수치로 나타낸 것입니다.

## 5. 정답값 (인식률 계산용)

- `meta/tray_slots_book_profiles.yaml` 의 `tray.slots` 가 **트레이 6칸의 책 AABB 중심 좌표**(`arm_base_link` 기준, m)입니다. 비전이 낸 점과 이 값을 비교하면 좌표 오차를 낼 수 있습니다.
- `tray_full_6books` 는 **6칸 모두 책이 있는 상태**로 고정입니다 (로봇 정지).
- `tray_during_place` 는 로봇이 책을 꺼내 가므로 중간에 **6권 → 5권**으로 줄어듭니다. 프레임별 정답 수가 필요하면 말씀해 주세요, 시점별로 뽑아 드리겠습니다.
- 책 6종의 치수와 종류는 `260918_train/README.md` 4절에 있습니다.

## 6. 알아 두실 점

- 전부 **시뮬 영상**입니다. 실물 카메라 성능은 따로 확인이 필요합니다.
- bag 은 zstd 로 압축돼 있습니다. `ros2 bag play` 는 그대로 읽고, 파이썬에서 직접 읽을 때는 `SequentialCompressionReader` 를 써야 합니다 (`tools/bag_detect_eval.py` 가 그렇게 돼 있습니다).
- 카메라 발행 주기를 5 Hz 로 낮춰 녹화했습니다 (파일 크기 때문). 실제 시연에서는 10 Hz 입니다.
- 다른 장면이 필요하면 (책 배치·각도·빈 트레이 등) 말씀해 주세요. `tools/record_vision_bag.sh` 로 GPU PC 에서 다시 뜹니다.
