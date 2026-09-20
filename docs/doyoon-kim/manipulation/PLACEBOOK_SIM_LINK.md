# PlaceBook ↔ Isaac Sim 연결 (방식 '나')

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17, D 로봇팔 |
| 상태 | **ROS 경유 실제 동작 확인** — 이 PC 노드 → GPU PC Isaac, 4곳 연속 4/4 + 취소 1회 |

## 1. 구조

```
FSM ──PlaceBook action (/place_book)──▶ manipulation_node (시스템 Python 3.12, 이 PC 또는 PC B)
                                           │  /manipulation/sim/command  std_msgs/String (JSON)
                                           ▼
                                        place_book_server.py (Isaac 내장 Python 3.11, GPU PC)
                                           │  /manipulation/sim/state    std_msgs/String (JSON, 10Hz)
                                           ▲
```

- 명령 규약·단계 이름·오류 코드: 팀 저장소 `shelving_manipulation/book_placer.py` **한 파일을 양쪽이 import**
  (GPU PC 에는 `sync_to_gpu.sh` 가 `~/shelving_manipulation_py/` 로 보낸다)
- 궤적은 Isaac 안에서 사전 계획(5mm·2°)·관절 보간 실행. 매 스텝 관절 목표를 ROS 로 보내지 않는다 (웹 클로드 v6 회신)
- 장면·계획·실행 조립: `isaac_sim/isaac/book_scene.py` (multi_book.py 에서 검증된 코드를 옮김)
- 커스텀 인터페이스를 Isaac(3.11)에서 빌드하지 않으려고 Isaac 쪽은 **표준 메시지만** 쓴다

## 2. 실행

```bash
# GPU PC — Isaac 작업 실행기 (시작 홈 이동 후 "준비 완료" 로그)
ROS_DOMAIN_ID=130 ~/arm/isaac/run_place_book_server.sh          # --gui 로 화면 보기

# 노드 PC
source /opt/ros/jazzy/setup.bash && source ~/ws_cobot_pjt/book-shelving-system/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=130 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file src/shelving_manipulation/config/manipulation.yaml -p executor:=sim

# 목표 (1차 고정 칸 4곳 중 하나)
ros2 action send_goal --feedback /place_book shelving_interfaces/action/PlaceBook \
  "{job_id: j1, book_id: b1, target_slot: {header: {frame_id: arm_base_link}, \
    pose: {position: {x: -0.3497, y: 0.5495, z: 0.3399}, orientation: {z: 0.7071068, w: 0.7071068}}, confidence: 1.0}}"
```

Isaac 없이 시험: `-p executor:=mock` (노드 안) 또는 `ros2 run shelving_manipulation sim_mock` (토픽 경유).

## 3. 검증 결과 (2026-09-17, 시험 도메인 77)

| 작업 | 트레이 칸 → 목표 (arm_base_link) | 결과 | 꽂힌 책 AABB 중심 | 오차 | 각속도 최대 |
| --- | --- | --- | --- | --- | --- |
| isaac1 | 0 → (-0.3497, 0.5495, 0.3399) | 성공 | (-0.3502, 0.5493, 0.3405) | 0.8 mm | 23% |
| isaac2 | 1 → (-0.5097, …) | 성공 | (-0.5101, 0.5493, 0.3407) | 0.9 mm | 23% |
| isaac3 | 2 → (-0.4297, …) | 성공 | (-0.4304, 0.5492, 0.3406) | 1.0 mm | 23% |
| isaac4 | 3 → (-0.2697, …) | 성공 | (-0.2706, 0.5490, 0.3405) | 1.2 mm | 23% |
| isaac_cancel | 4 → 3초 뒤 취소 | **M412**, 액션 CANCELED, failed_phase MOVING_TO_PRE_INSERT | 책 쥔 채 정지 | — | — |

피드백 단계: DETECTING_BOOK → PLANNING_GRASP → APPROACHING_BOOK → GRASPING → MOVING_TO_PRE_INSERT →
INSERTING → RELEASING → INSERTING(책등 밀기) → RETREATING → VERIFYING.

노드 단위·통합·lint 시험: `colcon test --packages-select shelving_manipulation` 46개 통과 (실패 입력 포함).

## 4. 1차 한계

| 한계 | 설명 |
| --- | --- |
| 취소 후 복구 없음 | 책을 쥔 채 그 자리 정지(HOLD_GRIP). 다음 목표 전에 Isaac 재시작 필요 |
| `insertion_speed` 미반영 | 노드가 계산해 보내지만 Isaac 은 검증된 밀기 속도(관절 0.12 rad/s) 고정 |
| 삽입 방향 | yaw +90°(arm_base_link +Y)만 지원. 다른 방향은 노드가 M410 |
| 트레이 칸 기억 | 노드 메모리. 노드를 재시작하면 칸 0 부터 다시 센다 |
| TF 미발행 | Isaac 이 TF 를 발행하지 않는다. 좌표는 arm_base_link = panda_link0 전제 |

## 5. 트러블슈팅 기록

### Isaac 내장 rclpy 가 다른 PC 와 통신하지 않음 (간헐, 원인 미확정)

| 시각 | 관찰 |
| --- | --- |
| 16:04~16:12 | Isaac 안 rclpy 로드 정상, **같은 GPU PC 의 시스템 ros2 와는 양방향 통신**. 이 PC 와는 5회 연속 실패 (whitelist 유무 무관) |
| 같은 시각 | GPU PC **시스템 ROS** talker ↔ 이 PC 는 정상 → 네트워크·방화벽 문제 아님 |
| 16:39 | Isaac 프로세스 소켓 확인: 10.10.0.2 / 127.0.0.1 / 239.255.0.1 에 도메인 77 포트(26650~) 정상 바인딩, 내장 libfastrtps 2.14 로드 |
| 16:40 이후 | **같은 설정으로 4회 연속 성공** (시작 6초 만에 수신). 이후 실제 작업 5건도 문제 없음 |

- 실패 구간에만 있던 것: 직전에 이 PC 에서 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로 노드·데몬을 띄웠다가 정리한 직후였다. 인과는 확인 못 함
- 재발 시 순서: ① 양쪽 `ros2 daemon stop` ② GPU PC 시스템 ros2 로 같은 토픽을 보이는지(네트워크 분리) ③ Isaac 프로세스 소켓(`ss -uanp`) ④ 도메인 번호를 바꿔 재시도
- 노드는 실행기가 한 번도 응답하지 않으면 **M411 "작업 실행기 미연결"** 로 끝난다 (조용히 멈추지 않음)

### Isaac 에서 rclpy 를 쓰는 조건 (설치본 확인)

- `enable_extension("isaacsim.ros2.bridge")` 뒤 `import rclpy` → `exts/isaacsim.ros2.bridge/jazzy/rclpy` 가 로드됨
- 환경변수: `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `LD_LIBRARY_PATH` 에 `exts/isaacsim.ros2.bridge/jazzy/lib` (→ `run_place_book_server.sh`)

### 시험 중 발견한 노드 버그

- rclpy 는 **같은 호출 위치에서 로그 심각도를 바꾸면 예외**를 던진다 (`log = info if ok else warning; log(...)`).
  단위시험은 통과했고 CLI 로 성공 → 실패 순서로 보낼 때 드러났다. 결과가 빈 값(error_code 0)으로 나갔다.
  호출 위치를 분리하고 "성공 뒤 실패" 통합시험을 추가했다 (수정 전 실패 확인)
- mock 실패 주입이 폴링 간격보다 짧은 동작을 건너뛰었다 → 인덱스 기준으로 수정
