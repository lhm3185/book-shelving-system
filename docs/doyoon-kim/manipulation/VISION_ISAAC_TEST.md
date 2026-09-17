# 손목 RealSense(Isaac) ↔ 비전 팀원 vision_manager 연동 시험

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17 18:10, D 로봇팔 |
| 대상 | `shelving_perception/vision_manager.py` @ 94b7967 (코드 수정 없이 파라미터만) |
| 결론 | **연결은 된다** (이미지·깊이·카메라정보·TF·sim time 이 비전 노드까지 들어가 책 좌표가 나온다). **책 인식 품질과 좌표 계산은 아직 쓸 수 없는 수준** — 3절 |

## 1. 실행 방법

```bash
# ① GPU PC: Isaac (레벨 v3 + 트레이 책 6권 + 손목 카메라 + ROS 발행). "준비 완료" 로그까지 약 40초
ROS_DOMAIN_ID=77 ~/arm/isaac/run_place_book_server.sh --camera          # 화면 보려면 --gui 추가

# ② 이 PC: 수신 확인 (안 보이면 4절)
export ROS_DOMAIN_ID=77 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
ros2 topic list --no-daemon        # /rgb /depth /camera_info /tf /clock 이 보여야 함

# ③ 이 PC: 정적 TF 2개 + vision_manager
ROS_DOMAIN_ID=77 ros2_ws/src/shelving_manipulation/isaac_sim/tools/run_vision_test.sh

# ④ (선택) 검출 좌표를 arm_base_link 로 바꿔 트레이 정답과 비교
python3 ros2_ws/src/shelving_manipulation/isaac_sim/tools/books_eval.py
```

- 팀 도메인(130 등)으로 할 때는 ①~④ 의 `ROS_DOMAIN_ID` 를 모두 같은 값으로
- 비전 노드는 **이 PC** 에서 돈다 (GPU PC 시스템 Python 에는 ultralytics 가 없다)
- 모델: `<비전 모델 best.pt> (MODEL_PATH 로 지정)` (GPU PC `runs/segment/runs/book/weights/best.pt` 복사본)

## 2. Isaac 쪽 구성 (`isaac_sim/isaac/ros_sensors.py`)

| 항목 | 값 | 근거 |
| --- | --- | --- |
| 카메라 위치 | `panda_hand` 에서 손 x 축 0.06 m, 광축 = 그리퍼 접근축 | **임시 가정** (레벨에 카메라 없음). 손가락이 화면 가운데를 덜 가리게 옆으로 뺐다 |
| 화각·해상도 | 가로 90.5°, 640×480, fx = fy = 317.2 | 학습 데이터와 같게 (camera_info 로 확인) |
| 토픽 | `/rgb`(rgb8) `/depth`(32FC1, m) `/camera_info` | vision_manager 기본값 그대로 |
| 프레임 | 이미지 frame_id `wrist_camera_optical_frame` | 광학 규약 |
| TF | Isaac: `panda_link0 → wrist_camera` / 이 PC 정적: `panda_link0 → arm_base_link`, `wrist_camera → wrist_camera_optical_frame`(항등) | 3-2 |
| 시간 | `/clock` (sim time), 노드는 `use_sim_time:=true` | TF 조회 시각 정합 |
| 촬영 자세 | 홈 = 트레이 위 30 cm 에서 내려다봄 | 트레이 관측 자세 |

손목 카메라 화면: `docs/img/wrist_home_rgb.jpg` (책 6권이 보인다. 가운데에 손가락과 그 그림자가 걸린다)

## 3. 결과

### 3-1. 연결 — 됨

| 확인 | 결과 |
| --- | --- |
| 이 PC 에서 수신 | `/rgb` `/depth` `/camera_info` 640×480, frame `wrist_camera_optical_frame`, TF 43 Hz, clock 100 Hz |
| 깊이 | 최소 0.042 / 중앙 0.593 / 최대 0.87 m (손가락 ~ 데크 바닥) |
| vision_manager | 모델 로드, 검출·역투영·발행까지 동작 (40초에 책 점 1,100~1,400개) |

### 3-2. 시험 중 잡은 문제 3건 (Isaac·TF 쪽 — 수정 완료)

| # | 증상 | 원인 | 조치 |
| --- | --- | --- | --- |
| 1 | 책 좌표가 카메라보다 **0.8 m 위** | Isaac `ROS2PublishTransformTree` 는 카메라 TF 를 **이미 광학 규약(+Z 앞)** 으로 낸다. 그 위에 X 180° 정적 TF 를 또 붙여 뒤집혔다 | 광학 프레임 정적 TF 를 **항등**으로. 광축(+z)이 트레이 쪽(아래)을 향하는 것을 TF 값으로 확인 |
| 2 | tf2 "tree contains a loop" | 로봇 루트를 TF 대상으로 넣으면 ridgeback_franka 가 `world→panda_link2→…→base_link→…→world` 로 **고리**가 된다 | 카메라만 `panda_link0` 기준으로 발행 |
| 3 | PC 간 토픽이 간헐적으로 안 보임 | 원인 미확정 (`PLACEBOOK_SIM_LINK.md` 5절). 이번에도 한 번 발생 후 GPU PC 에서 시스템 ros2 명령을 실행한 뒤 보이기 시작 | 재발 시 4절 |

### 3-3. 비전 코드 쪽 — **팀원에게 전달할 것** (리뷰 문서에서 예상한 항목이 실제로 재현됨)

**① `target_frame:=arm_base_link` 로는 좌표가 하나도 안 나온다**
- 로그: `Could not transform ... extrapolation into the past/future` 40초 동안 297회
- 원인: 이미지 콜백 안에서 `lookup_transform(timeout=0.2)` 로 기다리는데, TF 수신도 같은 스레드라 기다리는 동안 TF 버퍼가 갱신되지 않는다 (리뷰 4절 #5)
- 시험은 `target_frame:=wrist_camera_optical_frame` 으로 돌리고, 변환은 별도 노드(`books_eval.py`)에서 했다
- 제안: `TransformListener(buffer, node, spin_thread=True)` 또는 MultiThreadedExecutor

**② 좌표 정확도 — 깊이를 제대로 읽은 책만 맞다**

| 검출 | 카메라 깊이 | arm_base_link 오차 (정답: 트레이 칸 책등 윗면) |
| --- | --- | --- |
| 칸 5 | 0.400 m | dx −7 mm, dy +26 mm, **dz −0.9 mm** |
| 칸 4 | 0.507 m | dx −17 mm, dy +52 mm, dz −108 mm |
| 칸 2 부근 | 0.869 m | dy −675 mm, dz −469 mm (바닥 깊이) |

- 박스 **중심 한 픽셀**의 깊이를 쓴다. 중심이 책 사이 틈·칸막이·바닥에 떨어지면 틀린 좌표가 그대로 나간다 (리뷰 2-3절)
- 제안: seg 마스크 안 깊이 중앙값

**③ 인식 자체 — 트레이 위 책을 한 권씩 못 잡는다**

`docs/img/wrist_home_yolo.jpg` (같은 모델을 같은 프레임에 직접 돌린 결과):
- 검출 4개, 신뢰도 0.80 / 0.78 / 0.55 / 0.26
- **여러 권을 한 박스로** 묶는다 (0.80, 0.78 박스가 트레이 절반씩)
- **책이 없는 바닥 영역**을 책으로 검출 (0.55)
- 추정 원인: 학습 데이터는 책 한 권(또는 겹친 책 더미)을 여러 각도에서 찍은 것. **트레이 칸에 나란히 선 책을 위에서 본 장면**은 분포 밖이다. 손가락·그림자 가림도 있다

→ 웹 클로드 v7 회신대로 **1차는 트레이 칸 좌표로 동작하고 비전은 그림자 모드**가 맞다는 근거가 실측으로 생겼다.

## 4. 토픽이 안 보일 때

1. GPU PC 에서 `ros2 topic list --no-daemon` → Isaac 토픽이 보이는지 (Isaac 자체 문제인지 분리)
2. 이 PC 에서 `ros2 daemon stop` 후 `ros2 topic list --no-daemon`
3. 그래도 안 보이면 GPU PC 에서 시스템 ROS 명령 하나 실행 (`ros2 topic hz /clock`) 후 다시 2 — 오늘 두 번 모두 이 뒤에 보이기 시작했다 (인과 미확인)

## 5. 학습자료용 원자료

| 파일 | 내용 |
| --- | --- |
| `docs/img/wrist_home_rgb.jpg` | 손목 카메라 원본 (홈 자세, 트레이 책 6권) |
| `docs/img/wrist_home_yolo.jpg` | 같은 프레임 YOLO 결과 (박스·마스크, 빨간 점 = vision_manager 가 깊이를 읽는 중심 픽셀) |
| 3-2 표 | 좌표계 문제 3건 (증상 → 원인 → 조치) |
| 3-3 ② 표 | 깊이를 읽은 위치에 따른 오차 (0.9 mm ~ 675 mm) |
