# 연동 시험 — 카메라·라이다 탑재 매니퓰레이터 + 비전 + 로봇팔 (2026-09-17 저녁)

| 항목 | 값 |
| --- | --- |
| 목적 | AMR 담당(이동준)이 준 `franka_camera.usd` 의 **손목 RealSense** 로 비전(윤재민) 책 인식 → 로봇팔(김도윤) 파지·꽂기가 한 장면에서 함께 도는지 |
| 코드 | 로컬 통합 브랜치 `test/arm-vision-integration` = `origin/main` + `origin/vision`(230093d) + `feature/robot_control` (push 안 함) |
| 레벨 | GPU PC `~/Desktop/ing_library_env_v4.usd` = v3 에서 로봇만 `franka_camera.usd` 로 교체, 배치 좌표 v3 유지 (v3 원본 그대로) |
| 원본 수정 | `franka_camera.usd`, 비전 코드 **수정 없음**. 빠진 TF·clock 만 시험 실행 때 보탬 |
| 결과 요약 | **로봇팔 정상 (2/2, 1 mm 이내). 카메라·TF 연결 정상. 비전 인식은 사용 불가 수준** |

---

## 1. 실행 구성

```
GPU PC (Isaac, 도메인 77)
  ing_library_env_v4.usd ─ franka_camera.usd 그래프 : /rgb /depth /camera_info (640×640, frame sim_camera)
                                                     /point_cloud, /World/ridgeback_franka/{tf,odom}
  place_book_server.py --camera-prim …/Camera_OmniVision_OV9782_Color
                         → 추가: /tf (panda_link0 → Camera_OmniVision_OV9782_Color), /clock
이 PC
  static TF  panda_link0 → arm_base_link (항등)
  static TF  Camera_OmniVision_OV9782_Color → sim_camera (항등 — Isaac 카메라 TF 는 이미 광학 규약)
  vision_manager (origin/vision, perception.yaml 그대로, model_path 만 이 PC 경로)
  manipulation_node (executor:=sim)
  tools/trigger_eval.py (검출 요청 10회 → 트레이 칸 정답과 비교)
```

## 2. 결과

### 2-1. 로봇팔 — 카메라 탑재 로봇에서도 그대로 동작

| 작업 | 트레이 칸 → 목표 | 결과 | 꽂힌 책 AABB 중심 | 오차 | 각속도 최대 |
| --- | --- | --- | --- | --- | --- |
| v4job1 | 0 → (−0.3497, 0.5495, 0.3399) | 성공 | (−0.3504, 0.5492, 0.3405) | 1.0 mm | 23% |
| v4job2 | 1 → (−0.4297, 0.5495, 0.3399) | 성공 | (−0.4298, 0.5493, 0.3406) | 0.7 mm | 23% |

- 손목 RealSense 가 파지·삽입 경로에서 충돌하지 않음 (5개 판정 모두 통과)
- 작업당 실제 경과 35~42 s (카메라 렌더 포함. 렌더 없을 때 6~8 s)

### 2-2. 카메라·TF — 연결됨

| 확인 | 값 |
| --- | --- |
| 이미지 | `/rgb` rgb8, `/depth` 32FC1, 640×640, frame `sim_camera` |
| 내부 파라미터 | fx = fy = 317.0, cx = cy = 320 → 가로 화각 **90.5°** (학습 데이터와 같음) |
| 홈 자세 화면 | 트레이 책 6권이 위에서 보임 (손가락 일부 가림) |
| 깊이 | 0.022 ~ 0.853 m, 중앙 0.447 |
| arm_base_link 변환 | 비전 노드가 `arm_base_link` 좌표를 냄 (x·y 오차 수 cm → 축 규약 맞음) |

### 2-3. 비전 인식 — 아직 쓸 수 없음

**책 6권이 트레이에 있을 때** (요청 10회, 책 점 29개)

| 칸 | 검출 | 오차 dx / dy / dz (mm, dz 는 책등 윗면 기준) |
| --- | --- | --- |
| 0, 1, 2 | 없음 | — |
| 3 | 1회 | +37 / −8 / −90 |
| 4 | 8회 | −26 / −7 / −86 |
| 5 | 4회 | +32 / −10 / −106 |
| 칸과 무관한 점 | **17개** | 예 (−1.031, 0.189, −0.275) — 바닥·데크 |

**로봇팔이 2권을 옮긴 뒤** (칸 0·1 비어 있음, 4권 남음, 요청 10회, 책 점 20개)

| 칸 | 검출 |
| --- | --- |
| 0~5 전부 | **없음** |
| 칸과 무관한 점 | **20개 전부** |

같은 프레임에 모델을 직접 돌린 결과 (`results/20260917_v4_integration/wrist_after_2_yolo.jpg`):
검출 6개, 신뢰도 0.83~0.29. **트레이 전체·빈 데크·바닥을 한 박스로** 잡고 책 한 권 단위 박스는 없다.

- dz −86~−106 mm: 박스 안 깊이 중앙값(9/17 오후 비전 수정)을 쓰면서 점이 책등 윗면이 아니라 **책 중심 높이 근처**로 내려옴 (책 폭 163 mm 의 절반 ≈ 82 mm). 좌표 계약(AABB 중심)과는 오히려 가까움
- 오후 시험(손목 가정 카메라)과 같은 결론: **학습 데이터에 "트레이 칸에 나란히 선 책을 위에서 본 장면"이 없다**

## 3. 담당자별 전달 사항

### 이동준 (franka_camera.usd)

| # | 내용 | 영향 |
| --- | --- | --- |
| 1 | 로봇 TF·odom 이 **`/World/ridgeback_franka/tf`, `/World/ridgeback_franka/odom`** 으로 나간다 (TF 노드에 nodeNamespace `/World/ridgeback_franka`) | 표준 `/tf` 를 보는 Nav2·비전·로봇팔 노드가 로봇 TF 를 못 받는다 |
| 2 | 카메라 이미지 frame `sim_camera` 가 **TF 트리에 없다** | 카메라 → 로봇 좌표 변환 불가. 시험에서는 `panda_link0 → 카메라 prim` TF 와 `카메라 prim → sim_camera` 정적 TF 를 보태서 해결 |
| 3 | `/clock` 발행 없음 | `use_sim_time:=true` 노드의 시각이 멈춘다 (비전 perception.yaml 은 use_sim_time true) |
| 4 | 해상도 640×640 | 가로 화각은 90.5° 로 학습과 같다. 세로가 더 넓어 학습(640×480)과 화면 비율이 다르다 — 윤재민 확인 |
| 5 | `Pattern '…/rsd455/RSD455' did not match any rigid bodies` 오류 1회 | 동작 영향 없음 (시작 시 1회) |
| 참고 | 로봇 TF 를 로봇 루트로 발행하면 고리가 생기는 문제를 로봇팔 쪽에서 겪었다 (`COORDINATE_CONTRACT.md` 3절). parentPrim 을 base_link 로 준 현재 설정에서 고리 여부는 네임스페이스 때문에 이번에 확인 못 함 |

### 윤재민 (vision_manager, origin/vision 230093d)

| # | 내용 | 근거 |
| --- | --- | --- |
| 1 | **인식 품질**: 트레이 위 책을 권 단위로 못 잡음, 바닥·데크 오검출 | 2-3 표, `wrist_after_2_yolo.jpg`. 학습 데이터에 트레이 장면 추가 필요 |
| 2 | TF 조회 대기 문제 여전함 (`extrapolation into the future` 경고) | `results/20260917_v4_integration/vision.log` — 트리거 방식이라 오후보다 줄었지만 남음 |
| 3 | 깊이 중앙값 수정은 효과 있음 | z 가 표면이 아니라 책 중심 높이 근처로 옴 |
| 4 | `sim_camera` frame 확인 로직 정상 동작 | 이미지 frame 과 파라미터 일치 |

## 4. 결론

- **로봇팔 쪽은 연동 준비됨**: 카메라 탑재 로봇으로 교체해도 동작·정확도 변화 없음
- **카메라 ↔ 로봇 좌표 연결은 가능**하나 로봇 USD 쪽 TF 네임스페이스·카메라 frame·clock 보완 필요 (이동준)
- **비전 결과를 파지 목표로 쓰는 단계는 아직 아님**: 1차는 트레이 칸 좌표 + 비전 그림자 모드 유지 (웹 클로드 v7 회신 A2 와 같은 판단)

## 5. 재현

```bash
# GPU PC
ROS_DOMAIN_ID=77 ~/arm/isaac/run_place_book_server.sh --usd ~/Desktop/ing_library_env_v4.usd \
  --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color
# 이 PC (통합 브랜치 빌드 후)
ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color --child-frame-id sim_camera --ros-args -p use_sim_time:=true &
ros2 run shelving_perception vision_manager --ros-args --params-file src/shelving_perception/config/perception.yaml -p model_path:=<best.pt> &
ros2 run shelving_manipulation manipulation_node --ros-args --params-file src/shelving_manipulation/config/manipulation.yaml -p executor:=sim &
python3 src/shelving_manipulation/isaac_sim/tools/trigger_eval.py --n 10
```
