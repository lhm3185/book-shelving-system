# 연동 시연 실행 안내 — 카메라 + AMR 에셋 + 로봇팔 + 책 인식

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17, D 김도윤 (헤드리스로 전체 흐름 사전 확인) |
| 보여줄 것 | ① 카메라·라이다 탑재 로봇(레벨 v5)에서 로봇팔이 트레이 책을 서가에 꽂음 ② 손목 RealSense 영상으로 책 인식, 로봇팔 기준 좌표 수신 |
| 코드 | 이 PC 로컬 통합 브랜치 `test/arm-vision-integration` (main + vision + feature/robot_control), 빌드 완료 |
| 아직 없는 것 | AMR 주행 코드 (이동준 구현 예정). 로봇 에셋의 라이다·odom 토픽 발행까지만 보인다 |

## 0. 공통

- 두 PC 모두 **같은 `ROS_DOMAIN_ID`** (팀 기본 130). 아래는 `130` 기준
- 이 PC 터미널은 **4개** 필요 (① PC 노드 ② 카메라 창 ③ 책 꽂기 ④ 확인용)

## 0-1. 시작 전 워밍업·확인 게이트 (PC 간 간헐 불통 대비, 웹 클로드 v9 회신 A4)

원인은 미확정이지만, 두 번 모두 **GPU PC 에서 시스템 ros2 명령을 실행한 뒤** 토픽이 보였다. 그래서 순서로 고정한다.

```
1. GPU PC:  source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=130; ros2 topic list     ← 워밍업
2. GPU PC:  Isaac 실행 (아래 1절), "준비 완료" 확인
3. 이 PC:   ros2 topic list --no-daemon  →  /rgb /clock /tf /point_cloud /manipulation/sim/state 확인   ← 게이트
4. 3에서 안 보이면 GPU PC 에서 ros2 topic hz /clock 한 번 실행 후 3 다시. 그래도 안 되면 1부터
5. 통과하면 2절부터 시연
6. 2절 실행 뒤: ros2 param get /vision_manager confidence_threshold  →  0.75 가 아니면 멈춘다   ← 임계값 확인
```

6번: 비전 임계값은 코드 기본값과 `perception.yaml` 두 곳에 있다. 지금은 둘 다 0.75 지만 다시 어긋나면 리허설에서 잡는다 (단일 출처화는 2차 정리 항목).

**시연 중에 발견하지 말고 시작 전에 발견한다.** 시연 30분 전 전체 리허설 1회.

## 1. GPU PC — Isaac (레벨 v5 + 로봇팔 실행기)

GPU PC 터미널에서:

```bash
ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --amr-test-overrides
```

- `--amr-test-overrides`: AMR 에셋의 라이다 한 바퀴 발행(fullScan)·TF 고리·네임스페이스를 **실행에서만** 보완 (`LIDAR_TEST_20260917.md`). 라이다를 안 볼 때는 빼도 된다
- 터미널에 시스템 ROS 가 source 돼 있어도 된다 (실행 스크립트가 걸러냄 — 9/17 저녁 "켜지자마자 꺼짐" 원인)

- GPU PC 모니터에 Isaac 창이 뜬다 (약 40~60초). 로봇이 팔을 접었다가 트레이 위 홈 자세로 간다
- 터미널에 **`준비 완료 … 명령 대기`** 가 나오면 다음 단계
- 창 없이: `ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --headless`
- **다시 시작할 때**(책을 다 꽂았거나 취소한 뒤): Ctrl+C 후 같은 명령. 트레이 책 6권이 새로 놓인다

## 2. 이 PC — 비전 + 로봇팔 노드 (터미널 ①)

```bash
cd ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools
ROS_DOMAIN_ID=130 ./run_demo_pc.sh
```

`실행됨 … 로그: /tmp/b1_demo` 가 나오면 된다. Ctrl+C 한 번에 모두 종료.

연결 확인 (터미널 ④):

```bash
export ROS_DOMAIN_ID=130; source /opt/ros/jazzy/setup.bash
ros2 topic list --no-daemon      # /rgb /depth /camera_info /tf /clock /point_cloud 가 보여야 함
ros2 action list                 # /place_book
ros2 topic hz /rgb               # 약 60 Hz
```

**토픽이 안 보이면** (PC 간 간헐 불통, 원인 미확정): GPU PC 에서 `source /opt/ros/jazzy/setup.bash; ros2 topic hz /clock` 을 한 번 실행한 뒤 이 PC 에서 다시 `ros2 topic list --no-daemon`. 오늘 두 번 모두 이 뒤에 보였다.

## 3. 이 PC — 카메라 화면 + 책 인식 (터미널 ②)

```bash
export ROS_DOMAIN_ID=130
source /opt/ros/jazzy/setup.bash; source ~/ws_cobot_pjt/book-shelving-system/ros2_ws/install/setup.bash
python3 ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools/detect_view.py
```

- 창에 **손목 RealSense 영상 + YOLO 박스**가 실시간으로 나온다 (이 도구가 같은 모델로 직접 돌린 결과)
- 창을 클릭한 뒤 **`d`** → 비전 노드(vision_manager)에 검출 요청 → 왼쪽 위에 **로봇팔 기준(arm_base_link) 책 좌표**가 뜬다
- `s` 화면 저장(`/tmp/b1_demo`), `q` 종료

정답과 수치로 비교 (터미널 ④):

```bash
source ~/ws_cobot_pjt/book-shelving-system/ros2_ws/install/setup.bash
python3 ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools/trigger_eval.py --n 10
```

## 4. 이 PC — 로봇팔이 책 꽂기 (터미널 ③)

```bash
cd ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools
ROS_DOMAIN_ID=130 ./place_books.sh 1      # 1권. 최대 4권 (서가 1번 칸 4곳)
```

- 단계가 차례로 출력된다: PLANNING_GRASP → APPROACHING_BOOK → GRASPING → MOVING_TO_PRE_INSERT → INSERTING → RELEASING → INSERTING(책등 밀기) → RETREATING → VERIFYING → `success: true`
- 1권에 실제 약 35~45 초 (카메라 렌더 포함)
- 꽂은 뒤 창에서 `d` 를 다시 누르면 줄어든 트레이를 다시 인식한다

## 4-1. 이 PC — 라이다 스캔맵 (터미널 ④, 선택)

```bash
cd ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools
ROS_DOMAIN_ID=130 python3 lidar_check.py --seconds 10     # 주기·TF·위에서 본 그림 (/tmp/b1_demo/lidar_topdown.png)
ROS_DOMAIN_ID=130 ./lidar_scanmap_test.sh 30               # 30초 뒤 /tmp/b1_demo/scanmap/map.pgm 저장
```

## 5. 미리 알고 볼 것 (사전 확인 결과)

| 항목 | 기대 결과 |
| --- | --- |
| 로봇팔 | 성공, 꽂힌 위치 오차 1 mm 안팎, 관절 튐 0 |
| 카메라 | 640×640, 가로 화각 90.5°, 트레이 책 6권이 보임 (손가락 일부 가림) |
| 책 좌표 | arm_base_link 로 나온다 (좌표계 연결 확인) |
| **책 인식 품질** | **낮다.** 트레이 전체·바닥을 한 박스로 잡거나 오검출. 6권 중 0~3권 근처만 나옴. 학습 데이터에 트레이를 위에서 본 장면이 없어서 — 비전 담당 개선 항목 (`INTEGRATION_TEST_20260917.md` 3절) |
| AMR | 이동 없음 (주행 코드 없음). 보완 옵션을 켜면 `/point_cloud` 한 바퀴 13.9 Hz, `/odom`, `/tf` 정상, 정지 위치 기준 스캔맵 생성 |

## 5-1. 센서 부하 절감 (기본 동작)

Isaac 실행기는 **파지 확인(들어 올린 책 상승 확인) 직후 카메라·라이다 발행을 끄고, 작업이 끝나면 다시 켠다.** 대기 중 카메라는 10 Hz.
끄고 싶지 않으면 `--sensor-policy always`, 카메라 주기는 `--camera-hz N`.

같은 조건(레벨 v5, 라이다 보완 옵션, 헤드리스, 책 1권) 측정:

| 방식 | 1권 **벽시계** (명령→결과) | 렌더 스텝 / 시뮬 스텝 | /rgb 파지 뒤 | /point_cloud 파지 뒤 |
| --- | --- | --- | --- | --- |
| 항상 켬, 카메라 60 Hz | 23.0 s | 1305 / 1305 | 약 57 Hz | 약 9.5 Hz |
| **파지 뒤 끔, 카메라 10 Hz** | **11.5 s** | **286 / 1305** | **0** | **0** |

- **시뮬 스텝은 같다(1,305 = 21.75 s).** 줄어든 것은 계산 부하와 실제 경과 시간이지 로봇 사이클 타임이 아니다 (`BASELINE.md` 0절)
- 대기 중에는 그대로 나온다: /rgb 약 14 Hz(시뮬 10 Hz, 시뮬이 실시간보다 빠름), /point_cloud 약 14 Hz
- 파지·운반·삽입 판정(M405~M409)은 Isaac 내부 물리 상태로 하므로 센서를 꺼도 영향 없음
- 카메라 발행 주기는 **재생 전에 USD 로** 넣어야 반영된다 (실행 중 OmniGraph 로 바꾸면 무시 — 실측)

## 6. 정리

- GPU PC: Isaac 터미널 Ctrl+C
- 이 PC: 터미널 ① Ctrl+C, 창은 `q`
