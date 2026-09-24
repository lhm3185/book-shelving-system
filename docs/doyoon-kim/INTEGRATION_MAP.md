# 로봇팔 통합 지도 — `shelving_manipulation` 이 주고받는 것 전부

김도윤(D) · 2026-09-24 · **통합 담당자(A)가 `main` 에 합칠 때 보는 문서**

여기 적힌 것은 전부 코드에서 확인한 것이다. 기억으로 쓰지 않았다.
확인 위치를 줄 번호로 같이 적는다.

---

## 1. 한 눈에

```
                  ┌─────────────────────────────┐
  FSM ──PlaceBook─▶                             │
       (액션)     │      manipulation_node       │──JSON 명령──▶ Isaac 작업 실행기
  FSM ─DetectTargetSlot─▶   executor: sim|mock   │◀──JSON 상태──  (manipulation_executor)
       (액션)     │                             │
                  └──┬────────────┬─────────────┘
                     │            │
         /robot/status│            │/perception/detect_request
         (RobotStatus)│            │/perception/slot_scan_active
                     ▼            ▼
                   FSM          비전
                                   │
                     /perception/books ─────┐
            /perception/empty_shelf_position┘
                                            ▼
                                     manipulation_node
```

`executor: mock` 으로 두면 **Isaac 없이** FSM 연동을 시험할 수 있다. 노드 안에서
흉내 내고, 실패 코드를 파라미터로 주입할 수 있다(`mock_fail_at`, `mock_fail_code`).

---

## 2. 우리가 제공하는 것 (서버)

| 종류 | 이름 | 타입 | 조건 |
|---|---|---|---|
| 액션 서버 | `/place_book` | `PlaceBook` | 항상 |
| 액션 서버 | `/detect_target_slot` | `DetectTargetSlot` | `enable_perception_bridge: true` 일 때만 |
| 퍼블리시 | `/robot/status` | `RobotStatus` | 2 Hz |
| 퍼블리시 | `/perception/detect_request` | `Bool` | 책 좌표가 필요할 때 |
| 퍼블리시 | `/perception/slot_scan_active` | `Bool` | 빈칸 스캔 중에만 true |

`manipulation_node.py:128~130, 157~172`

### 2.1 `/perception/slot_scan_active` 를 반드시 알아야 한다

**비전은 이 신호가 true 인 동안에만 빈칸을 검출한다.** 그리고 이 신호는
`DetectTargetSlot` 액션 안에서만 켜진다. 즉 **액션을 거치지 않으면 빈칸이 안 온다.**

`scan_place_from_vision.py` 같은 옛 도구가 빈칸을 못 받는 이유가 이것이다 —
그 경로는 게이트가 생긴 뒤로 **죽은 경로**다.

---

## 3. 우리가 필요로 하는 것 (클라이언트·구독)

| 종류 | 이름 | 타입 | 누가 낸다 | 없으면 |
|---|---|---|---|---|
| 구독 | `/perception/books` | `PointStamped` | 비전 | 책 좌표를 못 받아 파지 불가 |
| 구독 | `/perception/empty_shelf_position` | `PointStamped` | 비전 | 빈칸 후보가 안 쌓인다 |
| 퍼블리시 | `/manipulation/sim/command` | `String`(JSON) | 우리 → 시뮬 | — |
| 구독 | `/manipulation/sim/state` | `String`(JSON) | 시뮬 → 우리 | 하트비트 끊김 → M404 |

`manipulation_node.py:138~141, 150~153`

**`/perception/books` 는 "보이는 윗면 중심"이다.** 책 폭의 절반을 빼서 AABB 중심으로
바꾸는 일은 **치수를 아는 쪽(우리)** 이 한다. 비전이 바꿔서 주지 않는다.

---

## 4. 좌표계 — 틀린 사고는 전부 여기서 났다

```
arm_base_link    액션이 주고받는 기준. TargetSlot.pose · GraspObservation 전부 이것
Camera_..._Color → sim_camera   정적 TF 로 이어 준다 (run_vision.sh 가 띄운다)
panda_link0      → arm_base_link 정적 TF
```

**차체가 작업 중에 움직인다.** 스캔 전 중심 맞추기로 266 mm, 꽂기 전 맞춤으로
273 mm. 그래서 `arm_base_link` 기준 좌표는 **잰 시점에 묶여 있다.**

> `TargetSlot` 에 `base_pose` 를 실어 달라는 요청을 비전팀에 보낼 준비가 돼 있다
> (`docs/doyoon-kim/proposals/interfaces/`). 지금은 **호출 순서**로 막고 있는데,
> 그건 사람이 기억해야 하는 규칙이라 언젠가 깨진다. 실제로 한 번 깼다.

### 4.1 호출 순서 — 어기면 조용히 틀린다

```
반드시    빈칸 검출(DetectTargetSlot) → 책 검출 → PlaceBook
틀림      책 검출 → 빈칸 검출 → PlaceBook      ← 사이에 차체가 266 mm 간다
```

틀린 쪽은 **오류가 안 난다.** 좌표는 멀쩡해 보이고 나이 검사(`max_target_age_s`)도
통과한다. 틀어진 것은 시간이 아니라 기준이기 때문이다.

---

## 5. 실행

```bash
# 실물/시뮬 공통
ros2 run shelving_manipulation manipulation_node \
  --ros-args --params-file ros2_ws/src/shelving_manipulation/config/manipulation.yaml

# Isaac 없이 FSM 연동만
ros2 run shelving_manipulation manipulation_node \
  --ros-args -p executor:=mock -p mock_fail_at:=wedge -p mock_fail_code:=407
```

`setup.py` 진입점: `manipulation_node`, `sim_mock`.

### 5.1 `manipulation.yaml` 에서 **반드시 켜져 있어야 하는 것**

```yaml
enable_perception_bridge: true   # 없으면 /detect_target_slot 자체가 안 뜬다
align_base_before_work: true     # 작업 전 차체 정렬. 시뮬 rotate_base 가 있어야 한다
executor: sim                    # Isaac 과 붙일 때
slot_x_snap_to_gap: true         # ← 2026-09-24 추가. 없으면 검증된 결과가 안 나온다
slot_y_from_shelf_front: true    # ←
scan_command: scan_shelf         # ← 코드 기본 scan_sweep 은 아래 판에서 404 로 죽는다
```

**아래 셋은 9/24 까지 저장소에 없었다.** 밤새 잰 결과가 켜진 상태의 값인데, 켜는
일이 실행 셸의 환경변수(`MAN_EXTRA`)에만 있었다. 지금은 yaml 에 있고, **이유는 그
파일의 주석에 같이 적혀 있다** — 값만 옮기면 다음 사람이 또 뺀다.

**같은 사고가 세 번 났다.** 그래서 개별로 막지 않고 목록을 시험에 고정했다
(`test/test_manipulation_yaml.py`). 한 줄이라도 빠지거나 값이 바뀌면 시험이 죽고,
바꿔야 할 때는 시험도 같이 고치게 되니 "왜 바꿨나" 가 커밋에 남는다.

---

## 6. 실패 코드 — FSM 이 구분해야 하는 것

| 코드 | 뜻 | FSM 이 할 일 |
|---|---|---|
| 404 | 시뮬 상태 끊김 / 시간 초과 | 재시도. 실행기 생존 확인 |
| 405 / 407 | 파지·삽입 단계 실패 | 다른 칸 또는 사람 호출 |
| 409 | 끝까지 갔는데 **배치가 확인 안 됨** | 그 칸을 점유로 기록하면 안 된다 |
| 410 | **계획을 거부했다** (여유 부족, 빈칸 못 찾음, 맞춤 거절) | 메시지를 읽고 판단. 재시도해도 같다 |
| 411 | 이미 작업 중 / 실행기가 한 번도 응답 안 함 | 순서 문제. 앞 작업 종료 확인 |
| 412 | 취소됨 | 정상 경로 |

**410 은 "못 한다" 가 아니라 "안 한다" 이다.** 문턱을 낮추면 되는 게 아니라, 그
자리가 위험하다는 뜻이다. 메시지에 사유가 들어 있다(예: 가장 가까운 실측 빈칸이
멀다 → 어느 칸을 겨냥한 명령인지 알 수 없다).

---

## 7. 우리가 안 건드리는 것

`07_file_ownership.md` 를 따른다.

```
shelving_interfaces    건드리지 않는다. 메시지 정의를 먼저 바꾸면 비전 빌드가 깨진다
shelving_perception    건드리지 않는다. 관측 보고만 문서로 보낸다
shelving_navigation    건드리지 않는다
shelving_manipulation  우리 것
레벨 USD               **코드로 고치지 않는다.** 콜리전 교정은 런타임에서 한다
main 브랜치            통합 담당자 것. 우리는 작업 브랜치에만 올린다
```

---

## 8. 합칠 때 걸릴 것

| 항목 | 상태 |
|---|---|
| `manipulation_node.py` flake8 25건 | `origin/vision` 에서 물려받았다. **`colcon test` 가 막힌다** |
| `scan_place_from_vision.py` | 게이트 이후 **죽은 경로**. 지우거나 문서에 명시해야 한다 |
| `scan_sweep` 아래 판 404 | 우회 중. 위 판만 쓴다 |
| 트레이 칸 x·y | −10 / −94 mm 차이가 보이지만 **안 고쳤다** — 지그 실측이 먼저다 |

flake8 25건은 우리가 만든 것이 아니라서 손대지 않았다. **합치기 전에 누가 고칠지
정해야 한다** — 우리가 고쳐도 되지만, 그러면 비전팀 쪽 변경과 부딪칠 수 있다.

---

## 9. 더 볼 것

```
docs/doyoon-kim/VERIFICATION_AND_LIMITS.md      검증 결과·재현법·한계
docs/doyoon-kim/proposals/interfaces/README.md  비전팀에 보낼 요청·측정 보고
docs/doyoon-kim/proposals/level_collision.md    레벨 콜리전 교정 (메시별)
ros2_ws/src/shelving_manipulation/config/       파라미터. **주석이 근거다**
```
