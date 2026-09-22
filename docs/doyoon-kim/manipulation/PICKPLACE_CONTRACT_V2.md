# 계약 1장 — 비전 2단계 관측 픽앤플레이스 (초안 v2, 4인 합의 대상)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-21, D 로봇팔 (김도윤) |
| 대상 | 비전 2인 · TM(FSM) · AMR |
| 로봇 | **Ridgeback-Franka**, 제자리 (주행은 다음 단계) |
| 상태 | **초안.** 합의되면 `shelving_interfaces` 를 **한 번에** 바꾼다 |
| 바탕 | `COORDINATE_CONTRACT.md`(9/17 4인 합의) 를 그대로 잇는다 |

> **정정 이력 (중요)** — 이 문서의 첫 판은 "**TM 이 비전을 부르고** 로봇팔은 관측 자세로
> 가기만 한다"(`LookAt` 액션)로 썼다. **틀렸다.** 팀 구조는 **로봇팔이 비전의
> 클라이언트**다. 이 판은 그에 맞춰 다시 썼다. 첫 판에서 제안한 `LookAt` 액션은
> **없애지 않고 용도를 바꿔 남긴다** (§7 — 시험·디버그용).

---

## 0. 왜 이걸 먼저 쓰나

비전 2명이 **지금** 빈 공간·파지점 인식을 만들고 있고 TM 도 재구축 중이다.
합의가 늦을수록 **세 사람이 다시 짠다.**

그리고 오늘 배운 것이 하나 있다 — **"손끝"의 뜻이 조용히 "플랜지"로 바뀌어도
단위 시험은 전부 통과한다.** 이름이 같고 뜻이 다르면 통합해야 드러난다.

---

## 1. 시스템 구조 (팀 설명 그대로)

```
AMR ──"책장 앞에 도착했다"──▶ TM(FSM)
TM  ──"반납해라"────────────▶ 로봇팔
                              로봇팔 ──"빈 공간 찾아라"──▶ 비전
                              로봇팔 ◀──빈 공간 좌표────── 비전
                              로봇팔 ──"집을 책 찾아라"──▶ 비전
                              로봇팔 ◀──책 좌표────────── 비전
                              로봇팔 : 집어서 꽂는다
로봇팔 ──결과──▶ TM
```

| 누가 | 책임 |
| --- | --- |
| **TM(FSM)** | 언제 시작할지. **로봇팔에게 한 번만** 시킨다 |
| **로봇팔** | 관측 자세 이동 · **비전 호출** · 파지 · 삽입 · **계약 위반 거절** |
| **비전** | 본 것을 **잰 그대로** 낸다. 못 재는 것은 추정하지 않는다 |
| **AMR** | 도착 통보 (이번 단계에서는 사람이 대신해도 된다) |

**비전은 TM 과 직접 통신하지 않는다.** 그래서 이 문서의 계약은 대부분
**로봇팔 ↔ 비전** 사이의 것이다.

---

## 2. 로봇팔 → 비전 : 두 번의 요청

### 2-1. 빈 공간 (`DetectTargetSlot`) — **이미 있다**

```
string job_id
string book_id
string shelf_id
float32 book_width
float32 book_height
float32 book_thickness
float32 safety_margin
uint32  max_recaptures
---
bool success
shelving_interfaces/TargetSlot target_slot     # pose = 칸의 AABB 중심 (좌표 계약 2번)
uint32 candidate_count
int32  error_code
string message
```

로봇팔이 **책 치수를 실어 보낸다** — 비전이 "이 책이 들어갈 칸"을 고를 수 있게.

### 2-2. 파지점 (`DetectGraspPoint`) — **새로 필요**

```
# shelving_interfaces/action/DetectGraspPoint.action
string job_id
string book_id                          # 어떤 책을 찾을지 (빈 문자열이면 "아무거나 집을 만한 것")
float32 book_thickness                  # 아는 값이 있으면 (0 이면 모름)
float32 book_width
builtin_interfaces/Time not_before      # **이 시각 이후의 영상만 쓸 것** (§3)
uint32  max_recaptures
---
bool success
shelving_interfaces/GraspObservation grasp
uint32 candidate_count
int32  error_code
string message
---
string  phase
uint32  candidate_count
float32 best_confidence
```

`DetectTargetSlot` 과 **같은 모양**으로 맞췄다 — 비전 쪽 코드 구조를 재사용할 수 있게.

---

## 3. 타이밍 — `not_before` 로 푼다

로봇팔이 **관측 자세로 가서 멈춘 뒤** 비전을 부른다. 흔들리는 중에 찍으면 깊이가 튄다.

**로봇팔이 자기가 언제 멈췄는지 아는 구조**이므로, 첫 판의 `settled_stamp`(결과로 돌려주는 값)
대신 **요청에 `not_before` 를 실어 보낸다.** 더 단순하고 책임이 분명하다.

| 로봇팔이 보장하는 것 (요청을 보내기 전) | 값 |
| --- | --- |
| 관절 오차 | `max|q − q_view| < 0.01 rad` |
| 정지 | `max|q̇| < 0.01 rad/s` 가 **0.3 s 이상 유지** |
| 센서 | 게이트가 열리고 그 뒤로 **렌더된 프레임 2장 이상** |

`not_before` = 위가 **처음 만족된 시뮬 시각**.
**비전은 `header.stamp ≥ not_before` 인 영상만 쓴다.**

> `DetectTargetSlot` 에도 같은 필드가 필요하다 (지금은 없다). 인터페이스를 한 번에
> 바꿀 때 같이 넣자.

---

## 4. `GraspObservation` — 무엇을 보낼 것인가

### 원칙: 계약에 싣는 양은 **보내는 쪽이 직접 잴 수 있는 것**. 유도는 아는 쪽이.

| 양 | 누가 직접 아나 |
| --- | --- |
| 보이는 **윗면의 중심·방향·두께·폭** | **비전** (깊이로 직접 잰다) |
| 책 **높이** (가려진 쪽) | **TM / 로봇팔** — `PlaceBook.book_height` 에 이미 있다 |
| **AABB 중심** | 위 둘을 합쳐야 나온다 → **유도값** |

비전에게 AABB 중심을 요구하면 **비전이 모르는 책 높이를 추정**하게 만드는 것이다.
틀려도 비전은 알 수가 없다.

```
# shelving_interfaces/msg/GraspObservation.msg
std_msgs/Header header          # frame_id = arm_base_link, stamp = **관측한 영상의 시각**
geometry_msgs/Point top_center  # 보이는 윗면의 중심. **AABB 중심이 아니다**
float32 spine_yaw               # 책등이 향하는 방향 (arm_base_link z 축 기준, rad)
float32 thickness               # 윗면에서 잰 두께 (m)
float32 width                   # 윗면에서 잰 폭 (m)
float32 confidence
```

### 이름을 나누는 이유

9/17 사고들(8 cm, 0.67 m)의 원인은 "윗면 한 점"이라는 **선택**이 아니라
**같은 필드 이름에 서로 다른 점을 넣은 것**이다.

| 용도 | 양 | 필드 이름 |
| --- | --- | --- |
| **꽂을 곳** (`DetectTargetSlot`) | 칸의 **AABB 중심** — 좌표 계약 2번 그대로 | `TargetSlot.pose` |
| **집을 것** (`DetectGraspPoint`) | 보이는 **윗면 중심** — 이름으로 구분되는 예외 | `GraspObservation.top_center` |

**`pose` 라는 이름은 좌표 계약 2번 전용으로 남긴다. 윗면 중심은 절대 `pose` 라고
부르지 않는다.** 이름이 다르면 섞일 수가 없다.

---

## 5. 로봇팔이 하는 검사 — 틀린 점을 **경계에서** 잡는다

비전이 준 값은 **쓰기 전에 전부 검사**한다. 통과 못 하면 작업을 거절한다.

| # | 검사 | 걸리면 | 막는 사고 |
| --- | --- | --- | --- |
| 1 | `frame_id == arm_base_link` | **410** | `base_link`·`sim_camera`·`world` 혼동 |
| 2 | `top_center.z − 트레이 바닥 ≈ 책 폭(W)` ±1 cm | **410** "관측 높이가 책 규격과 다르다" | **다른 점을 보냈을 때** (8 cm 계열) |
| 3 | `top_center` 가 트레이 범위 안 | **410** | 좌표 계산 오류 (0.67 m 계열) |
| 4 | `thickness ≈ book_thickness` ±5 mm | **410** | 다른 책을 봤을 때 |
| 5 | `slot.available_width ≥ thickness + 2×여유` | **410** "칸이 좁다" | 들어가지 않는 칸 |
| 6 | 관측 나이: 칸 **90 s**, 파지점 **30 s** | **411** (다시 보면 되는 실패) | 오래된 좌표 |

**2번이 핵심이다.** 트레이에 세운 책은 윗면이 바닥에서 **책 폭(W)만큼** 위에 있어야 한다.
비전이 윗면 대신 책등 중앙이나 AABB 중심을 보냈으면 **여기서 바로 수 cm 차이가 난다.**
계약 위반을 **이름뿐 아니라 물리량으로도** 잡는 것이다.

---

## 6. 로봇팔 내부 — 블로킹하면 안 된다 (구현 제약)

Isaac 스탠드얼론은 **매 스텝 `world.step()` 을 도는 단일 루프**다.
비전 응답을 기다리는 동안 **멈추면 시뮬 전체가 선다** (팀 합의 2026-09-16, `arm_primitives.py` 머리말).

그래서 비전 호출은 **프리미티브(상태 기계)로 만든다** — 기존 `MoveJoint`/`JointPath` 와 같은 방식:

```
AskVision(view="shelf")   상태: MOVING → SETTLING → REQUESTED → WAITING → DONE/FAILED
   · MOVING     관측 자세로 이동
   · SETTLING   §3 의 세 조건을 만족할 때까지
   · REQUESTED  not_before 를 실어 액션 goal 전송 (논블로킹)
   · WAITING    매 스텝 결과를 확인. 제한 시간 초과 → 404
   · DONE       §5 검사 → 통과하면 결과를 job 에 저장, 실패면 410/411
```

**제한 시간은 스텝 수로 센다** (벽시계로 재면 렌더가 느린 PC 에서 멀쩡한 동작이 실패한다).

### 작업 전체 순서 (로봇팔 내부)

```
1. AskVision(shelf)   → TargetSlot      (빈 칸)
2. AskVision(tray)    → GraspObservation (파지점)
3. plan_job(파지점, 빈칸)                 ← 여기서 계획이 실패하면 401/402 로 거절
4. approach → down → grip → lift → carry → insert → release → home
5. (선택) AskVision(shelf) 로 **놓은 뒤 확인** — 이때는 책이 광축을 안 가린다
```

> **①과 ④ 사이 시차**: 빈 칸을 먼저 보고 20~30초 뒤에 꽂는다.
> **삽입 직전 재관측은 넣지 않는다** — 그때는 팔이 책을 들고 있어 손목 카메라
> 광축 바로 앞을 책이 가린다. **볼 수 없는 것을 본 척하지 않는다.**
> 대신 나이 상한(§5-6)과 **놓은 뒤 확인**(5번)으로 대신한다.

---

## 7. `LookAt` 은 남긴다 — 용도만 바꿔서

첫 판에서 TM 용으로 제안했던 `LookAt` 액션은 **이 구조에서는 정상 흐름에 쓰이지 않는다.**
다만 **시험·디버그용으로 남기면 값어치가 있다**:

- 비전 팀이 **로봇팔 동작 없이** "책장을 보는 화면"만 띄우고 인식을 시험할 수 있다
- 관측 자세를 실측하거나 조정할 때 한 줄로 부를 수 있다

정상 흐름에서는 로봇팔이 내부적으로 같은 자세를 쓴다. **자세 값은 한 곳**
(`arm.yaml` 의 `poses.view_shelf` / `poses.view_tray`)에만 둔다.

---

## 8. TM(FSM) → 로봇팔

지금 `PlaceBook` 을 그대로 쓸 수 있다. 다만 **의미가 바뀐다**:

| 필드 | 새 구조에서 |
| --- | --- |
| `target_slot` | **비워도 된다.** 로봇팔이 비전에게 직접 받는다 |
| `book_id` | 어떤 책을 집을지 (비전에 그대로 전달) |
| `book_width/height/thickness` | **TM 이 알려준다** — 비전이 못 재는 높이를 여기서 받는다 |
| `grasp` / `has_grasp` | 첫 판에서 추가한 필드. **TM 이 채우지 않는다.** 로봇팔이 비전에게 받는다 |

> `grasp`/`has_grasp` 를 **지울지 남길지 정해야 한다.**
> 남기면 "비전을 못 쓸 때 TM 이 직접 좌표를 주는" 우회로가 된다 (가짜 비전·수동 시험용).
> 제 생각은 **남기는 쪽**이다 — 비전이 늦어도 흐름을 돌려볼 수 있다.

---

## 9. 알려진 함정 (재발 방지 목록)

| 함정 | 언제 있었나 | 이번에 |
| --- | --- | --- |
| 비전이 `base_link` 로 낸다 | 9/20 `origin/vision` 기본값 | Franka 에서 `base_link` 는 **Ridgeback 몸체**다. `arm_base_link`(=`panda_link0`)로 낼 것 |
| 카메라 토픽 이름 충돌 | 팔·AMR 둘 다 `/rgb` | 구독 토픽을 계약에 적는다. 필요하면 `--camera-ns /arm` |
| 흔들리는 중 촬영 → 깊이 튐 | — | `not_before` (§3) |
| 요청 시각 ≠ 영상 시각 | 비전 멈춤 (TF 버퍼 밖 14.7 s) | 관측 결과에 **영상 stamp** 를 싣고 받는 쪽이 나이를 본다 |
| 광학 프레임 180° 이중 적용 | 9/17, **0.8 m** | 정적 TF 는 **항등** 2개만 (`VISION_HANDOFF.md` §2) |
| 깊이 단위 | — | **m**, `32FC1` (mm 아님) |
| 시뮬이 통째로 멈춤 | — | 비전 호출은 **논블로킹 프리미티브**로 (§6) |

---

## 10. 확인 부탁드릴 것

| 대상 | 물어볼 것 |
| --- | --- |
| **비전 2인** | ① `top_center`·`thickness`·`width`·`spine_yaw` 를 **실제로 잴 수 있나?** 못 재는 항목은 빼는 게 낫다 ② 인터페이스를 **액션**으로 여는 것이 맞나 (토픽 구독 방식이면 구조가 달라진다) ③ `DetectGraspPoint` 라는 이름으로 괜찮은가 |
| **이현민(TM)** | ① `PlaceBook` 하나로 시작 신호를 주는 것이 맞나 ② `target_slot` 을 비우는 것이 맞나 ③ 책 치수는 TM 이 주는 것이 맞나 |
| **전원** | **인터페이스 병합 시각** — 바뀌면 4명 전원이 다시 빌드한다 |

---

## 11. 우리(로봇팔)가 먼저 할 일

| 순서 | 할 일 | 왜 |
| --- | --- | --- |
| 1 | **M406 H1 판별·수정** | 비전과 무관하고 **유일하게 알려진 실패 원인**이다. 비전 좌표가 아무리 좋아도 운반 중 3 cm 밀리면 결과는 2/4 에 머문다 |
| 2 | 관측 자세 2개 실측 (`view_shelf` · `view_tray`) | 계약이 정해진 뒤에 재야 다시 안 잰다 |
| 3 | **`AskVision` 프리미티브** (논블로킹 상태 기계) | 비전이 준비되기 전에 뼈대를 만들 수 있다 |
| 4 | **가짜 비전 노드** — 진짜 인터페이스로 고정 좌표를 내는 목 노드 | 비전을 기다리지 않고 흐름 전체를 돌려볼 수 있다 |

> 가짜 비전 노드에는 **일부러 틀린 값**을 섞는다 — `frame_id: base_link` 한 번,
> 책 높이와 안 맞는 `top_center` 한 번. **410 으로 거절되는지**가 곧 §5 검사의 음성 대조다.
