# FSM ↔ PlaceBook 연결 세션 — 한 장 안내 (이현민 전달용)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17, D 김도윤 |
| 목적 | 9/18 세션에서 **환경 맞추기 없이 바로 붙이기**. 호출 조건·사이클 시간은 붙여서 확인한다 |
| 로봇팔 코드 | `feature/robot_control` (main 미반영). 차이 보기: https://github.com/lhm3185/book-shelving-system/compare/main...feature/robot_control |

## 1. 어디서 무엇을 띄우나

| PC | 실행 | 명령 |
| --- | --- | --- |
| GPU PC | Isaac 로봇팔 실행기 (레벨 v5) | `ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh` |
| 김도윤 PC | `manipulation_node` (action 서버 `/place_book`) + 정적 TF + 비전 | `ROS_DOMAIN_ID=130 ./run_demo_pc.sh` (`shelving_manipulation/isaac_sim/tools/`) |
| 이현민 PC | FSM (action 클라이언트) | FSM 쪽 실행 명령 |

세 PC 모두 `ROS_DOMAIN_ID=130`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## 2. 순서 (워밍업 게이트 포함)

```
1. GPU PC:   source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=130; ros2 topic list   ← 워밍업
2. GPU PC:   Isaac 실행, 터미널에 "준비 완료 … 명령 대기" 확인 (40~60초)
3. 김도윤 PC: run_demo_pc.sh
4. 이현민 PC: 아래 3절 확인 명령 → 통과해야 FSM 실행
5. 안 보이면 GPU PC 에서 ros2 topic hz /clock 한 번 → 4 다시
```

## 3. 보이는지 확인 (이현민 PC)

```bash
ros2 topic list --no-daemon | grep -E "/clock|/manipulation/sim/state"
ros2 action list                                   # /place_book
ros2 action info /place_book                       # Action servers: 1
```

## 4. `/place_book` goal 예시 (1차 서가 1번 칸 첫 자리)

```bash
ros2 action send_goal --feedback /place_book shelving_interfaces/action/PlaceBook \
  "{job_id: fsm_test_1, book_id: book_0, target_slot: {header: {frame_id: arm_base_link}, pose: {position: {x: -0.3497, y: 0.5495, z: 0.3399}, orientation: {z: 0.7071068, w: 0.7071068}}, confidence: 1.0}}"
```

| 필드 | 1차 규칙 | 어기면 |
| --- | --- | --- |
| `job_id`, `book_id` | 비어 있으면 안 됨 | M410 |
| `target_slot.header.frame_id` | **`arm_base_link`** | M410 |
| `target_slot.pose.position` | **꽂힌 뒤 책의 AABB 중심** (m). 1차 칸 x = −0.3497 / −0.4297 / −0.5097 / −0.2697, y 0.5495, z 0.3399 | 범위 밖 M410 |
| `target_slot.pose.orientation` | **yaw +90°** (z 0.7071068, w 0.7071068). **단위 쿼터니언(w=1)은 yaw 0° 라 거절** | M410 |
| `book_width/height/thickness` | 0 이면 `book_profiles.yaml` 기본값. 일부만 0 이면 거절 | M410 |
| `confidence`, `header.stamp` | 1차는 검사 안 함 (`min_confidence 0`, `max_target_age_s 0`) | — |

**사전 확인 (2026-09-17 21:48, Isaac 없이 로봇팔 검증 코드로 확인)** — `origin/feature/system-fsm` `96c4c95` 는 비전 결과 `target_slot` 을 goal 에 그대로 넣는다. `mock_perception_server.py` 값으로는 거절된다.

| goal 값 | 로봇팔 결과 |
| --- | --- |
| mock 그대로: 위치 (0.55, 0, 0.80), 쿼터니언 w=1 | **M410** 삽입 방향 yaw 0° (1차 90°±6°) |
| 방향만 yaw 90° | **M410** 위치가 범위 밖 (x −0.56~−0.22, y 0.45~0.65, z 0.25~0.45) |
| 방향 + 위 표의 1차 칸 위치 | **통과** |

책 치수(0.18/0.24/0.035), 칸 폭·높이(0.08/0.30), 신뢰도 0.95, `arm_base_link` 는 통과. **mock 의 칸 위치·방향 두 가지만 1차 값으로 맞추면 된다.**

- 트레이에서 집을 책은 **로봇팔이 정한다** (트레이 칸 고정 좌표). FSM 은 서가 칸만 준다
- 작업 중 두 번째 goal 은 거절 (M411)
- 피드백 `phase` 순서: PLANNING_GRASP → APPROACHING_BOOK → GRASPING → MOVING_TO_PRE_INSERT → INSERTING → RELEASING → INSERTING(책등 밀기, 다시 보고) → RETREATING → VERIFYING. progress 는 줄지 않는다

## 5. 결과와 실패 코드

성공: `success: true`, `placement_verified: true`, `error_code: 0`.
**사이클 타임 기준: 1권 21.75 s (시뮬 시간, 홈→홈).** 벽시계는 PC 부하에 따라 다르다.

| 코드 | 이름 | 뜻 | FSM 이 할 일 |
| --- | --- | --- | --- |
| **410** | TARGET_INVALID | goal 이 규칙 위반 (4절 표). **로봇은 안 움직임** | goal 고치기 — 재시도 무의미 |
| **411** | NOT_READY | Isaac 실행기 미연결 / 작업 중 / 트레이 책 없음 | 2절 순서 다시, 연결 확인 후 재시도 |
| 404 | MOTION_TIMEOUT | 실행 중 시뮬 응답 끊김 (10 s) | 사람 확인 |
| 405 | GRASP_FAILED | 들어 올렸는데 책이 안 따라옴 → 홈 복귀 | 재시도 가능 |
| 406 | BOOK_DROPPED | 운반 중 떨어뜨림 | 사람 확인 |
| 407 | INSERT_BLOCKED | 끼우기 깊이 부족 → 뒤로 빠짐 | 재시도 가능 |
| 409 | PLACEMENT_NOT_VERIFIED | 꽂은 뒤 자세 조건 불만족 | 재시도 가능 |
| 412 | CANCELLED | FSM 이 취소 | 책 보유 여부 확인 후 재개 |

전체 표: `shelving_manipulation/shelving_manipulation/book_placer.py` `ERRORS`. 좌표 규약: `COORDINATE_CONTRACT.md`.

## 6. 세션에서 확인할 것

- [ ] FSM 이 보낸 goal 로 1권 성공 (M410 없이)
- [ ] FSM 이 결과(성공·코드)를 받아 다음 상태로 넘어감
- [ ] 연속 2권 (두 번째 goal 은 첫 결과 뒤에)
- [ ] 취소 → M412 를 FSM 이 처리
- [ ] Isaac 을 끈 상태에서 goal → M411 을 FSM 이 처리
- [ ] 사이클 시간 기록 (시뮬 21.75 s 기준과 비교)

---

## 7. `book_id` 가 실제로 어떻게 쓰이는가 (2026-09-19 실측으로 확인)

시험 중 **트레이에 없는 `book_9` 를 요청했는데 성공**해서 확인한 결과, **설계대로**였습니다. 다만 FSM 쪽에서 알고 계셔야 합니다.

```
1차 규칙: book_id 에 지정된 트레이 칸이 있으면 그 칸,
          없으면 "아직 꺼내지 않은 첫 칸"
```

| 요청한 `book_id` | 실제 동작 | 결과 |
| --- | --- | --- |
| `book_profiles.yaml` 의 `tray.assignments` 에 있는 이름 | **그 칸**에서 꺼낸다 | 의도대로 |
| 매핑에 **없는** 이름 (예: `book_9`) | **아직 안 꺼낸 첫 칸**에서 꺼낸다 | 성공하지만 **다른 책일 수 있다** |
| 매핑에 있지만 이미 꺼낸 칸 | 거절 | **411** `트레이 칸 N 은 이미 꺼냄` |
| 트레이가 비었을 때 | 거절 | **411** `트레이에 남은 책이 없음` |

**즉 `book_id` 는 매핑이 있을 때만 구속력이 있습니다.**

| 어느 쪽이 필요하신가요 | 하실 일 |
| --- | --- |
| "이 책을 꼭 그 칸에서" | `book_profiles.yaml` 의 `tray.assignments` 에 이름→칸 매핑을 넣습니다 |
| "순서대로 아무 책이나" | 지금 그대로 두시면 됩니다 (1차 시연은 이쪽으로 충분) |

**빈 트레이 요청이 411 로 정확히 거절되는 것**은 확인했습니다. 4권 시연 뒤 5번째 요청이 오면 조용히 성공하지 않고 411 을 돌려드립니다.
