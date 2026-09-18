# 로봇팔(D 김도윤) 1차 개발 계획 — 공식 상세 문서 분석

| 항목 | 값 |
| --- | --- |
| 근거 | `B-1project_detail.pdf` (2026-09-16 배포) |
| 내 담당 | **CP4 · 책 인식·파지 및 서가 삽입** |
| 1차 마감 | **9/19 단위개발 완료 → 9/19 16~18시 통합 → 9/20 14시 기능 동결 → 9/21 10시 시연** |
| 1차 완료 조건 | **고정 책 Pick & Insert** |

---

## 1. 내 과제가 정확히 무엇인가

정상 플로우에서 내가 책임지는 구간은 **CP4** 하나다.

```
[CP3 끝] 슬롯·삽입 Pose 확정
   ↓
트레이에서 대상 책 선택 → 책 위치·자세 확인 → 파지 자세·삽입 경로 생성
   → 책 파지 → 사전 삽입 위치 이동 → 지정 슬롯에 저속 삽입
   → 그리퍼 해제·로봇팔 후퇴 → 배치 완료 확인
   ↓
[FSM] 슬롯 점유 상태·작업 결과 갱신
```

**CP4 범위**: 목표 슬롯 확정 시점부터 배치 확인까지.
**1차 성공 기준**: 3회 연속 삽입, 낙하·비정상 충돌 0회, 삽입 최종 위치 오차 30mm 이하.

### 1차에서 내가 하지 않는 것

문서가 1차 전제로 못박은 것들이다. **여기에 시간을 쓰면 안 된다.**

- 책 자세 추정 — 트레이 **고정 위치**, **고정 파지 자세**를 쓴다
- 가변 그리퍼 폭 — **고정값**. 책 규격별 프로파일은 2차(9/22~23)
- 실패·재시도 분기 — 2차. 1차는 **정상 플로우만**
- 겹친 책 분리 — 1차는 **책 한 권**

> 우리가 만든 YOLO 겹침 인식·6D 자세 추정은 **2차 "가변 책 파지"** 에서 쓰인다.
> 1차 시연에는 들어가지 않는다.

---

## 2. 초기 전제와 달라진 것 (반드시 확인할 것)

| 항목 | 우리가 알던 것 | 공식 문서 |
| --- | --- | --- |
| Critical Point 번호 | CP1이 파지 | **CP4가 파지·삽입** (CP1은 트레이 인수) |
| 트레이 | 로봇팔이 책을 트레이에 적재 | **AMR이 책 담긴 트레이를 통째로 인수.** 로봇팔은 꺼내 꽂기만 |
| 삽입 방식 | 빈 슬롯에 drop-in 권고 | **저속 직선 삽입 + 삽입 깊이** |
| 실행 PC | GPU PC A에서 제어 | **PC B(로봇 온보드)** — 비전·Nav2와 같은 PC |
| 그리퍼 | Surface Gripper(흡착) 검토 | **"그리퍼 폭"** 전제 → 평행 조로 읽힌다 |

### 매니퓰레이터가 ridgeback_franka 로 변경됐다 (2026-09-16 저녁)

**M0609 은 보류다.** Clearpath Ridgeback(전방향 베이스) + Franka Panda(7축) 조합으로 바뀌었다.
Isaac Sim 5.1.0 설치본에서 직접 확인한 내용이다.

| 항목 | 값 |
| --- | --- |
| Franka USD | `<assets_root>/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` |
| 그리퍼 | `ParallelGripper`, dof `panda_finger_joint1`·`panda_finger_joint2` |
| 그리퍼 위치값 | open `[0.05, 0.05]`, closed `[0.0, 0.0]`, `action_deltas` `[0.05, 0.05]` |
| End effector prim | `panda_rightfinger` |
| IK EE 프레임 | `right_gripper` |
| **RMPflow 설정** | `motion_policy_configs/franka/rmpflow/` **제공됨** |
| 기성 컨트롤러 | `PickPlaceController` (10단계 `events_dt`) |
| 예제 | `pick_place` · `stacking` · `follow_target_with_ik` · `follow_target_with_rmpflow` · `franka_gripper` · `multiple_tasks` |
| **Ridgeback** | 양쪽 PC·번들 **어디에도 없음.** URDF 임포트 필요 |

**이 변경으로 두 가지가 정리된다.**

1. **그리퍼 논쟁 종료.** Franka Hand 는 평행 조이고 폭으로 제어한다.
   공식 문서의 "그리퍼 폭·파지 깊이" 표현과 정확히 맞는다. 흡착 검토는 폐기한다.
2. **장애물 회피가 기본 제공된다.** M0609 을 유지할 때의 최대 약점(RMPflow·cuMotion 설정 부재)이
   사라졌다. 다만 **삽입은 직선이어야 하므로** 구간을 나눈다.

```
자유공간 접근 (트레이 위 → 슬롯 앞)   →  RMPflow    (장애물 회피)
슬롯 삽입·후퇴                        →  IK + 선형 보간 (직선 보장)
```

RMPflow 는 부드럽지만 직선을 보장하지 않는다. 슬롯에 밀어 넣는 구간에서 곡선으로 들어가면
옆 책이나 슬롯 벽을 긁는다. **경로 종류를 프리미티브 단위로 분리하는 이유가 여기 있다.**

### 확인이 필요한 것

- **로컬 에셋이 없다.** `~/isaacsim/assets` 가 없어 `get_assets_root_path()` 가 원격을 가리킬 수 있다.
  오프라인이면 `franka.usd` 로드가 실패한다. **GPU PC 를 켜면 가장 먼저 확인할 것**
- Ridgeback 베이스 URDF 를 누가 임포트하는지 (AMR 담당 영역이지만 팔의 `base_link` 가 그 위에 있다)
- 팔 원점이 움직이는 베이스 위에 있으므로 **`book_pose` · `slot_pose` 의 frame 합의가 더 중요해졌다**

### (참고) 이전 그리퍼 논의 — 이제 무효

### 그리퍼가 가장 큰 쟁점이다

문서는 1차 전제에 **"파지 자세·그리퍼 폭·삽입 깊이는 고정값"**, 2차에 **"책 규격별 그리퍼 폭·파지 깊이"**
라고 쓴다. 흡착에는 "폭"이라는 개념이 없다. **평행 조 전제로 작성된 문서다.**

그리고 이번 삽입 방식이라면 **평행 조가 맞다.** 책을 세워 슬롯에 밀어 넣으려면 책등이나 표지면을
잡고 밀어야 하는데, 흡착은 미는 방향으로 힘을 받으면 전단력에 바로 떨어진다.
드롭인이 아니라 삽입으로 바뀐 순간 흡착의 근거가 사라졌다.

**→ 평행 조로 확정하는 것을 제안한다.** 다만 이건 나 혼자 정할 수 없고,
Isaac Sim USD에서 그리퍼를 누가 붙이는지와도 엮인다(내가 Joint·Gripper·책 물리 특성 검수 담당).

---

## 3. FSM(이현민)과 합의해야 할 인터페이스 — 9/17 오전까지

9/19 16~18시에 "책 Pose–로봇팔 파지·삽입" 통합이 잡혀 있다.
**그날 처음 인터페이스를 맞추면 늦는다.** 아래를 제안안으로 들고 간다.

### 제안: Action 2개

```
PickBook.action                      InsertBook.action
---                                  ---
# Goal                               # Goal
string  job_id                       string  job_id
geometry_msgs/PoseStamped book_pose  geometry_msgs/PoseStamped slot_pose
float32 grasp_width                  float32 insert_depth
float32 approach_distance            float32 approach_distance
---                                  ---
# Result                             # Result
bool    success                      bool    success
string  error_code                   string  error_code
geometry_msgs/PoseStamped actual     float32 final_error_m
---                                  ---
# Feedback                           # Feedback
string  phase                        string  phase
float32 progress                     float32 progress
```

- `phase` 는 프리미티브 이름을 그대로 돌려준다 (`approach` / `grasp` / `lift` / `retreat` …).
  FSM이 어디서 멈췄는지 바로 안다
- `error_code` 는 2차 오류코드 규약에 맞춘다: **`GRASP_FAILED`, `INSERT_FAILED`**
  (문서 8절에 이미 정의된 이름이다. 새로 만들지 않는다)
- 1차에서는 `grasp_width` · `insert_depth` 를 FSM이 **보내지 않아도** 되게 한다.
  0이면 `arm_config.yaml` 의 고정값을 쓴다. 2차에 값이 채워져 들어온다

### 확인해야 할 질문 4개

1. **메시지 패키지 소유자** — 1차 "ROS2 인터페이스"가 이현민 담당이다. 내가 `.action` 초안을 주고
   그쪽 패키지에 넣는 것이 맞나?
2. **frame_id** — `book_pose` · `slot_pose` 를 어느 좌표계로 받나.
   `base_link`(로봇팔 기준)로 받으면 내가 TF를 안 봐도 된다. 윤재민이 발행하는 Pose의 frame 과 맞춰야 한다
3. **TCP 오프셋을 누가 더하나** — 보내는 Pose가 책 중심인지 그리퍼가 가야 할 목표인지.
   **책 중심으로 받고 오프셋은 내가 처리**하는 쪽을 제안한다
4. **삽입 성공 판정** — `final_error_m` 을 내가 재서 보고할지, FSM이 비전으로 확인할지

---

## 4. 파일 구성 (내 소유)

```
arm_primitives.py    동작 단위 정의 + 논블로킹 실행기   ← 핵심
arm_config.yaml      자세·속도·폭·깊이·제한시간         ← 튜닝은 전부 여기
arm_mock.py          시뮬 없이 도는 가짜 백엔드
arm_action_server.py ROS 2 Action 래퍼                 ← FSM 접점
tests/test_arm.py    동선 로직 단위 테스트 (GPU 불필요)
```

**다른 사람 파일을 고치지 않는다.** 이게 프리미티브 구조를 택한 이유다.

### 프리미티브 목록 (1차)

| 이름 | 하는 일 | 주요 인자 |
| --- | --- | --- |
| `home()` | 초기 자세 | — |
| `stow()` | 주행 안전 자세 (Nav2 목표 전 필수) | — |
| `move_joint(q)` | 관절 보간 이동 | 목표 관절, 속도 |
| `move_linear(pose)` | 직선 이동 (삽입·후퇴용) | 목표 Pose, 속도 |
| `gripper(width)` | 그리퍼 폭 지정 | 폭(m) |
| `approach(pose, d)` | 목표 위쪽/앞쪽 d 만큼 떨어진 곳으로 | 거리 |
| `grasp(pose)` | approach → 접근 → 닫기 → 들어올리기 | |
| `insert(pose, depth)` | 사전 삽입 → 저속 직선 삽입 → 열기 → 후퇴 | 깊이, 저속 |

모두 `enqueue` 후 매 스텝 `update()` 가 `RUNNING/SUCCEEDED/FAILED` 를 돌려준다.
**블로킹 함수를 만들지 않는다** — Isaac Sim 스탠드얼론은 단일 루프라 시뮬이 멈춘다.

---

## 5. 9/17(목) 할 일 — 순서대로

문서상 9/17 내 항목은 **"Pre-grasp·Grasp·Lift Pose"** 다. 실제로는 이렇게 쪼갠다.

| 시간 | 할 일 | 완료 판정 |
| --- | --- | --- |
| 오전 | `.action` 초안 들고 **이현민과 인터페이스 합의** | frame_id·TCP 오프셋 주체 확정 |
| 오전 | `arm_primitives.py` + `arm_config.yaml` + `arm_mock.py` 골격 | `pytest` 로 grasp→insert 시퀀스가 mock 위에서 통과 |
| 오후 | `arm_action_server.py` — **mock 백엔드로 먼저 동작** | 이현민이 Action을 호출해 SUCCEEDED 를 받는다 |
| 18시 이후 | GPU PC에서 M0609 USD·Articulation·IK 연결 | 실제로 한 관절이라도 움직인다 |

**Action 서버를 mock 위에서 먼저 띄우는 것이 핵심이다.** 그러면 이현민은 내 IK가 완성되기 전에
FSM 연결을 끝낼 수 있고, 9/19 통합에서 붙이는 일만 남는다. 문서도 같은 방식을 지시하고 있다
("기능 개발자는 USD 완성을 기다리지 않고 Primitive 환경과 Mock Topic으로 단위 테스트한다").

### 확인이 필요한 사전 조건

- M0609 URDF/USD 를 누가 Isaac Sim Stage 에 넣나 — USD 담당 2명이 "AMR+로봇팔" 제작
- Lula IK 설정 파일(Doosan 용은 NVIDIA 미제공) — 수업 자료에 있는지 확인
- `/World/Robot` Prim 경로 규약 (문서 7절), 단위 미터, Up Axis Z

---

## 6. 2차(9/22~23) 미리보기 — 지금 준비할 것은 없다

"가변 책 파지·재시도", 완료 조건 "2개 규격 파지·1회 재시도".
우리가 만든 자세 추정기(중심 5.2mm, 법선 1.2°)와 YOLO 모델이 여기서 쓰인다.
**1차가 끝나기 전에는 손대지 않는다.**
