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

## 2. 팀 프레임 ↔ 로봇 프레임 — **로봇마다 다르다**

> **2026-09-20 갱신.** 아래 표가 Franka 만 담고 있어서 실제 사고가 났다.
> 비전 쪽이 `arm_base_link` 를 "프랑카 전용 링크" 로 이해하고 `base_link` 로 바꿨고,
> 그 사이 우리 static TF 도 `panda_link0` 으로 박혀 있어 M0609 에서 **별칭이 트리에 안 붙었다.**
> 계약 이름은 로봇과 무관하다는 것을 표로 못 보여준 것이 원인이다.

**계약 이름(`arm_base_link`)은 절대 바뀌지 않는다.** 붙는 대상만 로봇마다 다르다.

| 팀 계약 | Franka (~2026-09-18) | **M0609 결합체 (현재)** |
| --- | --- | --- |
| `arm_base_link` | `panda_link0` | **`base_link`** ← prim `m0609/base_link` 의 **이름** |
| 엔드이펙터 프레임 | `panda_hand` (`tool0`) | `link_6` |
| 손가락 | Franka Hand | `onrobot_rg2ft/{left,right}_inner_finger` |
| 팔 베이스 월드 z | 0.28 | **0.655** (AMR 데크 위) |

Isaac 은 팔 베이스 **prim 의 이름**을 TF frame 으로 발행한다. 그래서 로봇을 바꾸면
frame 이름이 같이 바뀐다. 우리는 그 위에 static TF 로 `arm_base_link` 별칭을 건다:

```
<팔 베이스 frame> → arm_base_link      # 부모는 robot_profiles 에서 읽는다 (박아두지 말 것)
```

`simulation/isaac/tools/run_demo_pc.sh` 가 자동으로 처리하고, 실행 시 화면에 찍는다:

```
팔 기준 프레임: base_link → arm_base_link (ARM_ROBOT=m0609)
```

회귀 테스트가 있다 — `simulation/isaac/tests/test_frame_contract.py`.
부모 프레임을 다시 박아 두면 테스트가 실패한다.

### ⚠️ `base_link` 를 직접 쓰지 말 것

M0609 결합체에서 `base_link` 는 **팔 베이스**지만, AMR(Nova Carter) 쪽도 같은 이름을
낼 수 있다. 그러면 tf2 가 팔 베이스와 AMR 몸체를 **같은 frame 으로 본다** — 0.655 m
어긋난 좌표가 조용히 통과한다. 반드시 `arm_base_link` 를 쓴다.
(이름 충돌 여부는 2026-09-20 기준 미확인. Isaac 을 켜고 `ros2 run tf2_tools view_frames` 로 확인할 것)

참고(Franka 시절): `base_link` → `panda_link0` = (0.30, 0, 0.28), 회전 없음.

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
