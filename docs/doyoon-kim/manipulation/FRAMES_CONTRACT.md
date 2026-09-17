# 로봇팔 좌표 계약 (비전·FSM·AMR 공통)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17, D 로봇팔 |
| 근거 | 웹 클로드 v6 회신 0절 — 원점 사고가 두 번(92mm, 37cm) |

## 1. 반드시 지킬 것 — 두 줄

```
모든 물체 Pose 는 AABB(바운딩박스) 중심 기준.  에셋 원점 기준 아님.
좌표계는 arm_base_link, REP-103(+X 앞, +Y 왼쪽, +Z 위), 단위 m.
```

**비전 쪽도 이미 같은 규약이다.** `estimate_pose.py` 가 라벨의 `bbox_local` 중심으로 보정해 비교한다
(gpu_v1 회신 1절). 라벨 `translation`/`T_cam_obj` 는 **원점 기준**이므로 파지점으로 그대로 쓰면 안 된다.

## 2. 팀 프레임 ↔ Franka 프레임

코드 안 이름은 Franka 그대로 쓰고, 팀 이름은 **static TF 로만** 연결한다
(`docs/doyoon-kim/proposals/arm_frames.launch.py` (launch 는 A 관리 — 반영 요청)).

| 팀 계약 | Franka | 변환 (Isaac Sim 5.1.0 실측) |
| --- | --- | --- |
| `arm_base_link` | `panda_link0` | 동일 |
| `tool0` | `panda_hand` | 동일 (플랜지, `panda_link7` 에서 z +0.107) |
| `gripper_tcp` | `right_gripper` (Lula) | `panda_hand` 에서 **z +0.10, z축 180°** |

참고: `base_link` → `panda_link0` = (0.30, 0, 0.28), 회전 없음.

## 3. 아직 연결되지 않은 것

- **Isaac 레벨에 ROS2 그래프가 없다** (v3 기준 0개). Isaac 이 `base_link → … → panda_hand` TF 를 발행해야
  위 매핑이 실제 트리에 붙는다. → 레벨/Isaac 담당 작업
- 로봇 배치를 레벨에서 +Y 15cm 옮겼다 → **CP2 도킹 목표 좌표에 반영 필요** (AMR 담당)

## 4. 주행 안전 자세 (stow) — 검증 완료

| 항목 | 값 |
| --- | --- |
| 관절값 | `[0.0, -1.75, 0.0, -2.8, 0.0, 1.1, 0.79]` |
| 팔 xy | AMR 베이스 영역 안 |
| 팔 최고 높이 | 0.985 m (바닥 기준) |
| 홈↔stow 전환 | 관절 각속도 최대 한계의 37% |

관절 속도 한계(URDF 실측): joint1~4 **2.175 rad/s**, joint5~7 **2.61 rad/s**. 합격 기준은 그 80%.
