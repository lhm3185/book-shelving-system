# shelving_perception 검토 — 로봇팔(D) 관점

| 항목 | 값 |
| --- | --- |
| 작성일 | 2026-09-17 |
| 검토자 | D 김도윤 (로봇팔) |
| 대상 | `ros2_ws/src/shelving_perception` @ `94b7967` (vision 브랜치 병합본) |
| 파일 | `vision_manager.py` 269줄, `book_detector.py` 50줄, `target_detector.py` 33줄, `config/camera.yaml`, `config/perception.yaml` |
| 결론 | **뼈대로서 방향은 맞다.** 다만 RealSense 가 **손목(Eye-in-Hand)** 에 달려 있어서 **로봇팔과 반드시 연동돼야 하는데, 그 연결 계약이 아직 어디에도 없다** (1절). 좌표계 규약 2건은 통합 날 좌표가 조용히 틀어지는 종류라 먼저 맞춰야 한다 (2절) |

---

## 0. 왜 로봇팔과 연동돼야 하나 — 확인 결과

| 근거 | 내용 |
| --- | --- |
| 하드웨어 확정 사항(프로젝트 브리프) | **손목 RealSense D435, Eye-in-Hand, 측면 오프셋 장착, 주행 중 촬영 금지** |
| 학습 데이터 | 화각 90.5° 는 **Isaac 손목 카메라** 실측값으로 맞춰 만들었다 |
| `target_detector.py` 주석 | "The robot is responsible for **moving the camera to each scan position**" — 카메라를 옮기는 주체가 로봇(=팔) |
| 팀 문서 `02_architecture_and_flow.md` 6절 | `camera_link` 가 **`base_link` 바로 아래**로 그려져 있음 → 몸체 고정 카메라 전제. **브리프와 다르다** |

**따라서 확실히 연동 대상이다.** 손목 카메라면 다음이 전부 로봇팔 쪽 일이 된다.

1. 카메라를 어디서 보게 할지 = **팔 관측 자세** (트레이 관측, 서가 관측)
2. `camera_link` 의 위치 = **팔 관절값에 따라 매 순간 바뀜** → `joint_states` 기반 TF 가 필요
3. 촬영 시점 = **팔이 멈춘 뒤** (주행 중·팔 이동 중 촬영 금지)

---

## 1. 연동 계약 공백 — 인터페이스 합의 필요 (4인 합의 사항)

### 1-1. 서가 관측 자세를 누가 명령하나

- FSM → `DetectTargetSlot` → perception 순서인데, **perception 은 팔을 움직일 수 없다**
- `PlaceBook` 은 "책 인식 → 파지 → 삽입" 이라 **서가를 먼저 보는 단계가 없다**
- 현재 흐름대로면 `DetectTargetSlot` 호출 시점의 팔 자세(트레이를 내려다보는 홈)로 서가를 찍게 된다 → **서가가 화면에 없다**

**제안(택1)**

| 안 | 내용 | 장단 |
| --- | --- | --- |
| A | FSM 이 `DetectTargetSlot` 전에 팔에 "서가 관측 자세" 요청 (새 action `MoveArmToNamedPose` 또는 `PlaceBook` 앞단 분리) | 흐름이 명시적. 인터페이스 1개 추가 |
| B | `DetectTargetSlot` goal 에 관측 자세 이름을 넣고, perception 이 팔에 요청 | perception → manipulation 의존 생김 (03 문서 10절 "interfaces 에만 의존" 위반은 아니나 호출 방향 증가) |

로봇팔 쪽 선호는 **A**. 관측 자세는 로봇팔이 충돌 검증 후 제공한다
(오늘 실제로 **팔을 높이 든 채 회전하면 서가 윗판에 부딪히는** 것을 확인했다 — 관측 자세는 임의로 정하면 안 된다).

### 1-2. 트레이 책 인식 결과를 로봇팔이 어떻게 받나

- `PlaceBook` 피드백 첫 단계가 `DETECTING_BOOK` → **로봇팔 서버가 책 인식 결과를 받아야 한다**
- 지금 `vision_manager` 는 `/perception/books` 에 `PointStamped` 를 **매 프레임 계속 발행**한다
  - 책 ID·자세(yaw)·크기·신뢰도·몇 번째 칸인지가 없음
  - 여러 권이면 **점이 권마다 따로** 나가서 한 프레임 묶음을 알 수 없음
  - 요청-응답이 아니라서 "팔이 멈춘 뒤 찍은 결과" 를 구분할 수 없음
- **제안**: 요청형(service 또는 action)으로 `DetectTrayBooks` — goal: tray_id, 결과: 책 배열(pose(AABB 중심), 크기, confidence, header.stamp)
  - 1차 시연은 공식 문서 전제("책은 트레이 고정 위치")대로 **로봇팔이 알려진 칸 좌표로 동작 가능**. 인식은 검증·보정 용도로 붙여도 된다

### 1-3. 촬영 시점

- 콜백이 들어오는 **모든 프레임**에 YOLO 를 돌리고 결과를 발행한다 → 팔이 움직이는 중에도 `TargetSlot` 이 나간다
- **제안**: action 요청을 받았을 때만 처리. 팔 쪽은 `RobotStatus(component=manipulation, state=IDLE)` 또는 관측 자세 도착 결과로 "정지" 를 보장

---

## 2. 좌표계 — 통합 날 조용히 틀어지는 부분 (우선 수정)

### 2-1. 광학 좌표계(optical frame) 규약 ★

`book_detector.py` / `_scan_position` 의 역투영 `x=(u-cx)z/fx, y=(v-cy)z/fy, z=depth` 는
**ROS 광학 좌표계(x 오른쪽, y 아래, z 앞)** 결과다. 그런데 이 점에 `rgb_msg.header.frame_id` 를 그대로 붙인다.

| 경우 | 결과 |
| --- | --- |
| frame_id 가 `camera_color_optical_frame` (광학) | 정상 |
| frame_id 가 `camera_link` (REP-103: x 앞, y 왼쪽, z 위) | **축이 뒤바뀐 좌표**가 나오고 에러 없이 발행됨 |
| Isaac 카메라 prim 경로를 그대로 TF 로 발행 | Isaac 5.1.0 카메라 prim 은 **USD 축(-Z 앞, +Y 위)** 이다 (`isaacsim.sensors.camera` `get_world_pose(camera_axes=...)` 에 world/ros/usd 3종 명시, 설치본 확인) → 광학 프레임과 180° 차이 |

추가로 Isaac 5.1.0 `ROS2CameraHelper` 의 `frameId` 기본값은 **`sim_camera`** (설치본 ogn 확인) → 그래프에서 따로 지정하지 않으면 TF 트리에 없는 이름이 붙는다.

**제안**: 광학 프레임 이름을 `camera.yaml` 에 고정하고, 메시지 frame_id 가 그것과 다르면 경고·폐기. 단위시험에 "카메라 정면 1m 점 → base 좌표" 한 건 추가.

### 2-2. 기준 프레임 이름

- `target_frame` 기본값 `base_link` ↔ 팀 계약 `TargetSlot` 은 **`arm_base_link` 권장** (03 문서 5절)
- 로봇팔 `docs/FRAMES_CONTRACT.md`: **"모든 물체 Pose 는 AABB 중심 기준, arm_base_link, REP-103, m"**
- `arm_base_link` ↔ Franka `panda_link0` 은 로봇팔이 static TF 로 제공 (`shelving_manipulation/launch/arm_frames.launch.py`, 아직 로컬)
- **제안**: 기본값 `arm_base_link`

### 2-3. "책 좌표" 가 가리키는 점

- 지금은 **박스 중심 픽셀의 깊이 한 점** = 책 **윗면(책등)** 위의 점
- 로봇팔 계약은 **AABB 중심**. 책 폭(세운 높이) 16.3cm 의 절반만큼 z 가 다르다
- 중심 한 픽셀 깊이는 칸막이·옆 책 경계에 걸리면 튄다 → **마스크 내부 깊이 중앙값** 권장 (seg 모델이라 마스크가 이미 있다. 지금은 `boxes` 만 사용)

### 2-4. sim time

- Isaac 이 sim time 스탬프를 찍는다 (`useSystemTime` 기본 False, 설치본 확인)
- 노드가 `use_sim_time:=true` 가 아니면 `lookup_transform(stamp)` 가 외삽 오류로 **매번 실패**할 수 있다 → launch 에 명시 필요

---

## 3. 빈 칸 판정 (`TargetDetector`) — 기능상 한계

| 현재 | 문제 |
| --- | --- |
| 화면 중앙 픽셀 한 점이 "책 중심에서 15cm 밖" 이면 빈 칸 | 한 점 판정이라 칸 폭·높이를 보지 않음. 책 사이 3.5cm 틈도 빈 칸이 될 수 있음 |
| `available_width/height` = 파라미터 고정 0.3 | **측정값이 아님**. 로봇팔은 이 값으로 끼울 수 있는지 판단한다 |
| `orientation` = 단위 쿼터니언 | 03 문서: "orientation = **책 삽입 방향**". 지금은 의미 없는 값 |
| `confidence` = 파라미터 1.0 | 03 문서 P2xx "낮은 신뢰도" 판정 불가 |
| 대상 책 크기 비교 없음 | `DetectTargetSlot` goal 의 책 폭·높이·두께·safety_margin 을 쓰지 않음 |
| 양면 뚫린 서가(`shelf_brown`) | 빈 칸 중앙 깊이가 **서가 뒤 벽/바닥**을 찍음 → 1차 전제("서가 뒷판 있음")와 확인 필요 |

로봇팔이 받아야 하는 최소 값 (현재 `spine_out` 동작 기준, 실측):

| 필드 | 로봇팔 사용처 | 비고 |
| --- | --- | --- |
| position | 칸 x 중심, 칸 바닥 z, 서가 앞면 y | **"빈 공간 중심" 이 바닥 기준인지 부피 중심인지 합의 필요** — 로봇팔은 바닥 z 와 앞면이 필요 |
| orientation | 서가 앞면 법선(삽입 방향) | yaw 만 의미 있음 |
| available_width | 책 두께 3.5cm + 손가락 여유 비교 | 인접 책 간격 최소 **8cm** 필요 (손가락 두께 2.64cm 포함, 오늘 4권 시험 통과 간격) |
| available_height | 세운 책 높이 23.7cm + 손목 여유 | 1번 칸 바닥~윗판 49cm (선반 z×1.4 후) |

---

## 4. 코드 품질·빌드

| # | 내용 | 영향 |
| --- | --- | --- |
| 1 | `ultralytics` 가 `package.xml` 에 없음 (pip 의존) | PC B 새 환경에서 import 실패. runbook 에 설치 절차 필요 |
| 2 | `model_path` 기본값이 GPU PC 절대경로 `/home/rokey/book_dataset/runs/segment/runs/book/weights/best.pt` | PC B 에 없음. 파라미터·YAML 로 빼기 |
| 3 | 파일명·실행명이 07 문서와 다름: `vision_manager` ↔ `perception_node`, `book_detector` ↔ `tray_book_detector`, `target_detector` ↔ `shelf_slot_detector`, `coordinate_transformer` 없음 | 07 문서 기준 리뷰·인수 시 혼선 |
| 4 | `target_topic` 기본값 `/m0609/empty_shelf_position` | **M0609 보류, ridgeback_franka 확정**. 토픽명에서 로봇 모델 제거 |
| 5 | TF 조회(`timeout 0.2s`)를 이미지 콜백 안에서 동기 실행 | 단일 스레드 executor 에서 콜백 정체 |
| 6 | 선반 모델(어제 인계한 `shelf_handoff`)은 아직 미사용 | 3절 빈 칸 판정에 쓸 수 있음 |
| 7 | 테스트는 lint 3종뿐, **로직 단위시험 없음**. flake8 기준 위반 다수 (docstring 따옴표, 공백, 파일 끝 개행) | 07 문서 완료 기준 "실패 입력 포함 시험" 미충족 |
| 8 | `DetectTargetSlot` action server·`RobotStatus` 발행·P2xx 코드 미구현 | 07 문서 담당 범위 |

좋은 점: 순수 로직(`BookDetector`, `TargetDetector`)을 ROS 노드와 분리해 둬서 **시뮬 없이 단위시험 가능한 구조**다. RGB/depth 크기·frame 불일치·유효하지 않은 깊이·intrinsics 검사가 들어가 있다. depth 인코딩(`16UC1`→0.001, Isaac `32FC1`→1.0) 처리도 맞다.

---

## 5. 로봇팔이 제공할 것 (연동 준비)

| 항목 | 상태 |
| --- | --- |
| `arm_base_link`/`tool0`/`gripper_tcp` ↔ Franka 프레임 static TF | 작성 완료(로컬, 커밋 전) |
| 트레이 관측 자세 = 현재 홈 자세 (트레이 위 30cm 에서 내려다봄) | Isaac 검증 완료 |
| 서가 관측 자세 | **미정** — 카메라 장착 위치 확정 후 충돌 검증해서 제공 |
| 팔 정지 보장 (`RobotStatus` IDLE) | PlaceBook 서버와 함께 구현 예정 |
| 손목 카메라 장착 pose (hand→camera) | **레벨 v3 에 카메라 prim 이 하나도 없다** (Isaac 5.1.0 에서 레벨을 열어 전수 조사, 로봇 prim 481개 로드 확인). ROS2 그래프(OmniGraph)도 없다. 누가 어디에 붙일지 정해야 한다 (07 문서: 카메라 prim 은 윤재민·이동준 공동 검토, 손목이면 김도윤 포함) |

---

## 6. 합의가 필요한 질문 (팀)

1. 카메라는 **손목(브리프)** 인가 **몸체 고정(02 문서 TF 트리)** 인가 — 문서 하나를 고쳐야 한다
   - 현재 레벨에는 카메라가 없어서 **비전 노드를 Isaac 과 연결해 볼 수 있는 상태가 아니다**. 장착 위치가 정해지면 로봇팔이 손목 자세·충돌을 함께 확인한다
2. 서가 관측 자세 명령 주체 — 1-1 안 A / B
3. 트레이 책 인식 결과 전달 방식 — 1-2 요청형 인터페이스 추가 여부, 1차는 고정 칸 좌표 허용 여부
4. `TargetSlot.position` 이 가리키는 점 (칸 바닥 앞 모서리 중심 / 빈 공간 부피 중심)
