# 계약 1장 — 비전 2단계 관측 픽앤플레이스 (초안, 4인 합의 대상)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-21, D 로봇팔 (김도윤) |
| 대상 | 비전 2인 · FSM · AMR |
| 로봇 | **Ridgeback-Franka**, 제자리 (주행은 다음 단계) |
| 상태 | **초안.** 합의되면 `shelving_interfaces` 를 **한 번에** 바꾼다 |
| 바탕 | `COORDINATE_CONTRACT.md`(9/17 4인 합의) 를 그대로 잇는다 |

---

## 0. 왜 지금 이걸 쓰나

비전 2명이 **지금** 빈 공간·파지점 인식을 만들고 있고 FSM 도 재구축 중이다.
합의가 늦을수록 **세 사람이 다시 짠다.**

그리고 오늘 배운 것이 하나 있다 — **"손끝"의 뜻이 조용히 "플랜지"로 바뀌어도
단위 시험은 전부 통과한다.** 이름이 같고 뜻이 다르면 통합해야 드러난다.
이 문서는 그걸 미리 막으려는 것이다.

---

## 1. 흐름과 책임

```
① FSM → 비전 :  DetectTargetSlot        빈 칸을 찾아라
     (그 전에)  FSM → 로봇팔 : LookAt{view:"shelf"}   책장을 봐라
② FSM → 로봇팔:  LookAt{view:"tray"}     트레이를 봐라
   FSM → 비전 :  집을 책을 찾아라 → GraspObservation
③④ FSM → 로봇팔: PlaceBook(target_slot=①, grasp=②)   집어서 꽂아라
```

| 누가 | 무엇을 책임지나 |
| --- | --- |
| **FSM** | 순서 조율. **무엇을** 볼지/할지 말한다 |
| **로봇팔** | **어떻게** 할지. 관측 자세 값·경로·파지·삽입. 계약 위반을 **거절**한다 |
| **비전** | 본 것을 **잰 그대로** 낸다. 못 재는 것은 추정하지 않는다 |

> **FSM 과 비전은 `/manipulation/sim/command` 에 직접 쓰지 않는다.**
> 그 토픽은 `manipulation_node` 아래의 **내부 통로**다. 거기에 직접 쓰면
> 위층의 검사(§4)를 건너뛴다. 모든 것은 액션을 거친다.

---

## 2. 새 인터페이스 ① — `LookAt`

**FSM 은 "무엇을 볼지"만 말하고, 자세 값은 로봇팔 설정에 둔다.**
FSM 이 관절각이나 손 자세를 들고 다니기 시작하면 좌표 계약이 하나 더 생긴다.

```
# shelving_interfaces/action/LookAt.action
string job_id
string view                            # "shelf" | "tray" — 이름만
---
bool   success
int32  error_code                      # 0 성공 / 410 모르는 view / 411 작업 중 / 404 도달·정지 실패
builtin_interfaces/Time settled_stamp  # **이 시각 이후의 영상만 유효**
geometry_msgs/PoseStamped camera_pose  # settled 시점 카메라 자세 (frame_id = arm_base_link)
string message
---
string  phase                          # MOVING | SETTLING | READY
float32 joint_error
```

### `settled_stamp` 가 핵심이다

"도달했다"만 알리면 비전은 **언제부터 찍을지를 타이밍 운에 맡긴다.**
흔들리는 중에 찍으면 깊이가 튄다.

**성공 조건 (셋 다 만족해야 한다)**

| 조건 | 값 |
| --- | --- |
| 관절 오차 | `max|q − q_view| < 0.01 rad` |
| 정지 | `max|q̇| < 0.01 rad/s` 가 **0.3 s 이상 유지** |
| 센서 | 게이트가 열리고 그 뒤로 **렌더된 프레임 2장 이상** |

`settled_stamp` = 위가 **처음 만족된 시뮬 시각**.

**비전은 `header.stamp ≥ settled_stamp` 인 영상만 쓴다.**

---

## 3. 새 인터페이스 ② — `GraspObservation`

### 원칙: 계약에 싣는 양은 **보내는 쪽이 직접 잴 수 있는 것**. 유도는 아는 쪽이.

| 양 | 누가 직접 아나 |
| --- | --- |
| 보이는 **윗면의 중심·방향·두께·폭** | **비전** (깊이로 직접 잰다) |
| 책 **높이** (가려진 쪽) | **FSM / 로봇팔** — `PlaceBook.book_height` 에 이미 있다 |
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
| **집을 것** (새로) | 보이는 **윗면 중심** — 이름으로 구분되는 예외 | `GraspObservation.top_center` |

**`pose` 라는 이름은 좌표 계약 2번(AABB 중심) 전용으로 남긴다.
윗면 중심은 절대 `pose` 라고 부르지 않는다.** 이름이 다르면 섞일 수가 없다.

> 참고로 위에서 집을 때 로봇팔이 실제로 쓰는 값도 윗면이다 —
> 지금 코드는 `AABB 중심 → 윗면 − 3.5 cm` 로 **거꾸로 유도**하고 있다.

---

## 4. `PlaceBook` 추가 필드

```
# 기존
string job_id
string book_id
shelving_interfaces/TargetSlot target_slot
float32 book_width
float32 book_height
float32 book_thickness
float32 insertion_speed
# 추가
shelving_interfaces/GraspObservation grasp
bool has_grasp                 # ROS 2 메시지에는 optional 이 없다
```

**`has_grasp=false` 면 지금처럼 설정 파일의 트레이 칸을 쓴다.**
비전이 늦어도 흐름이 멈추지 않는다.

### 받는 쪽 검사 — 틀린 점을 **경계에서** 잡는다

| # | 검사 | 걸리면 | 막는 사고 |
| --- | --- | --- | --- |
| 1 | `frame_id == arm_base_link` | **410** | `base_link`·`sim_camera`·`world` 혼동 |
| 2 | `top_center.z − 트레이 바닥 ≈ 책 폭(W)` ±1 cm | **410** "관측 높이가 책 규격과 다르다" | **다른 점을 보냈을 때** (8 cm 계열) |
| 3 | `top_center` 가 트레이 범위 안 | **410** | 좌표 계산 오류 (0.67 m 계열) |
| 4 | `thickness ≈ book_thickness` ±5 mm | **410** | 다른 책을 봤을 때 |
| 5 | `slot.available_width ≥ thickness + 2×여유` | **410** "칸이 좁다" | 들어가지 않는 칸 |

**2번이 핵심이다.** 트레이에 세운 책은 윗면이 바닥에서 **책 폭(W)만큼** 위에 있어야 한다.
비전이 윗면 대신 책등 중앙이나 AABB 중심을 보냈으면 **여기서 바로 수 cm 차이가 난다.**
계약 위반을 **이름뿐 아니라 물리량으로도** 잡는 것이다.

---

## 5. 시차 — 관측을 얼마나 믿고 들고 있나

①(빈 칸)과 ④(삽입) 사이에 **20~30초**가 흐른다.

**④ 직전 재관측은 넣지 않는다.** 그때는 팔이 책을 들고 있어 손목 카메라 광축 바로 앞을
책이 가린다 — **볼 수 없는 것을 본 척하지 않는다.** 대신 셋을 한다.

| | 무엇 | 방법 |
| --- | --- | --- |
| **1. 나이 상한** | 오래된 관측 거절 | 칸 **90 s**, 파지점 **30 s**. 넘으면 **411** (다시 보면 되는 실패) |
| **2. 가정 명시** | "작업 중 서가는 바뀌지 않는다" | 이 문서에 적는 것이 곧 가정이다 |
| **3. 사후 검증** | 놓은 뒤 확인 | 손이 빈 뒤 `LookAt(shelf)` 로 **한 번 더 본다** — 이때는 가릴 게 없다 |

> **"미리 확인할 수 없으면, 나중에 확인한다."** 4/4 를 2/4 로 정정하며 얻은 원칙이다.

---

## 6. 로봇팔 쪽 전제 (명시)

| 전제 | 내용 |
| --- | --- |
| `PlaceBook` 시작 자세 | 계획이 **홈에서 시작**한다고 가정한다. 시작 시 팔이 홈에서 **0.05 rad 넘게** 떨어져 있으면 **411 로 거절**한다 |
| 센서 게이트 | `LookAt` 이 **열고**, 다음 움직임 명령이 **닫는다** |
| 관측 자세 값 | `arm.yaml` 의 `poses.view_shelf` / `poses.view_tray` (실측 예정) |
| 거절 코드 | 기존 M401~M412 체계를 그대로 쓴다 |

> 새 흐름의 순서가 ① 책장 → ② 트레이(=홈) → ③④ 라서 ② 뒤에 팔이 홈에 있다.
> **우연히 맞는 것**이라 위처럼 명시한다.

---

## 7. 알려진 함정 (재발 방지 목록)

| 함정 | 언제 있었나 | 이번에 |
| --- | --- | --- |
| 비전이 `base_link` 로 낸다 | 9/20 `origin/vision` 기본값 | Franka 에서 `base_link` 는 **Ridgeback 몸체**다. `arm_base_link`(=`panda_link0`)로 낼 것 |
| 카메라 토픽 이름 충돌 | 팔·AMR 둘 다 `/rgb` | 구독 토픽을 계약에 적는다. 필요하면 `--camera-ns /arm` |
| 흔들리는 중 촬영 → 깊이 튐 | — | `settled_stamp` (§2) |
| 요청 시각 ≠ 영상 시각 | 비전 멈춤 사건 (TF 버퍼 밖 14.7 s) | 모든 관측에 **영상 stamp** 를 싣고 받는 쪽이 나이를 본다 (§5) |
| 광학 프레임 180° 이중 적용 | 9/17, **0.8 m** | 정적 TF 는 **항등** 2개만 (`VISION_HANDOFF.md` §2) |
| 깊이 단위 | — | **m**, `32FC1` (mm 아님) |

---

## 8. 합의가 필요한 것 / 바뀌는 범위

### 인터페이스 변경은 **한 번에** 묶는다

`shelving_interfaces` 가 바뀌면 **4명 전원이 다시 빌드**한다.
`LookAt` 추가 + `PlaceBook` 필드 추가 + `GraspObservation` 추가를 **한 번의 변경**으로
묶고, 변경 시각을 공지한다. 두 번 나눠 바꾸면 그 사이 누군가는 옛 인터페이스로 돌린다.

### 확인 부탁드릴 것

| 대상 | 물어볼 것 |
| --- | --- |
| **비전 2인** | `top_center`·`thickness`·`width`·`spine_yaw` 를 **실제로 잴 수 있나?** 못 재는 항목이 있으면 그것부터 빼자 — 못 재는 것을 계약에 넣으면 추정값이 섞여 더 위험하다 |
| **FSM** | `LookAt` 을 FSM 이 조율하는 구조가 맞나. `settled_stamp` 를 비전에 넘겨줄 수 있나 |
| **전원** | 인터페이스 변경 시각 |

---

## 9. 우리(로봇팔)가 먼저 할 일

| 순서 | 할 일 | 왜 |
| --- | --- | --- |
| 1 | **M406 H1 판별·수정** | 비전과 무관하고 **유일하게 알려진 실패 원인**이다. 비전 좌표가 아무리 좋아도 운반 중 3 cm 밀리면 결과는 2/4 에 머문다 |
| 2 | 관측 자세 2개 실측 (`view_shelf` · `view_tray`) | 계약이 정해진 뒤에 재야 다시 안 잰다 |
| 3 | **가짜 비전 노드** — 진짜 인터페이스로 고정 좌표를 내는 목 노드 | FSM 이 비전을 기다리지 않고 통합할 수 있다 |

> 가짜 비전 노드에는 **일부러 틀린 값**을 섞는다 — `frame_id: base_link` 한 번,
> 책 높이와 안 맞는 `top_center` 한 번. **410 으로 거절되는지**가 곧 §4 검사의 음성 대조다.
