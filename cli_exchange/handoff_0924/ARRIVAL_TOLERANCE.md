# AMR 도착 정밀도 허용치 — 실제 실행 측정 (2026-09-24)

## 왜 실제 실행으로 쟀나
스윕 도구로는 재현이 안 된다. `move_rig` 는 `arm_base_world` y 가 실제 파이프라인과
20.3 mm 다르고(스윕 −3.0481 vs 실제 −3.068381), `--no-move` 는 로봇이 키오스크에 선 채
yaw +90° 라 13 점 전부 401 로 죽는다. 그래서 `night/run_vision.sh` 로 비전→파지→삽입→복귀
전 구간을 돌리며 `SIM_PICK_Y` 만 흔들었다.

기준점(데모 조합)
```
SIM_PICK_Y=-3.0695  SIM_GOAL_X=-0.3382  SIM_RELEASE_OPEN_FIRST=1
SIM_CARRY_MODE=swing  SIM_RETURN_MODE=swing
SIM_HOME_Q=-2.3634,-0.7300,-0.6933,-2.1884,-0.4396,1.5845,-0.5540
```
`GOAL_Y` 는 짝 규칙 `GOAL_Y = 0.5495 + (-3.019 - PICK_Y)` 로 코드가 알아서 따라온다
(기준점에서 0.600).

합격 기준: `code==0 && verified && min_margin >= 0.15 && 겹침 0`.

## 결과

| 편차 | SIM_PICK_Y | 합격 | min_margin (rad) | 겹침 | 비고 |
|---|---|---|---|---|---|
| −60 mm | -3.1295 | ❌ | — | — | `410 TARGET_INVALID` @PLANNING_GRASP. 목표 y 0.660 > 검증 상한 0.65 |
| −45 mm | -3.1145 | ✅ | 0.214 | 0 | |
| −30 mm | -3.0995 | ✅ | 0.234 | 0 | |
| **0** | **-3.0695** | ✅ ×5 | 0.246~0.279 | 0 | 데모 조합 (N2b,N2d,D3,D4,D5b) |
| +15 mm | -3.0545 | ✅ | 0.224 | 0 | |
| +30 mm | -3.0395 | ✅ | 0.205 | 0 | |
| +45 mm | -3.0245 | ✅ | 0.187 | 0 | |
| +52 mm | -3.0175 | ✅ | 0.182 | 0 | |
| +60 mm | -3.0095 | ❌ | 0.171 | **14.0 mm** | `409` — `no_jam` 하나만 실패. 나머지(upright/depth/spine/x/floor) 전부 통과 |

## 답
**동료가 물은 ±30 mm 구간 안에서는 min_margin 이 0.15 밑으로 내려가지 않는다.**
실측 안전 구간은 **−45 mm ~ +52 mm**. 그러니 AMR 에 요구할 도착 정밀도는
**y 방향 ±30 mm** 면 충분하고, 양쪽으로 15~22 mm 의 예비가 남는다.

다만 **한계의 성격이 min_margin 이 아니다** — 이게 이번 측정의 진짜 소득이다.

- **− 쪽**: 관절이 아니라 **작업영역 봉투**. 목표 y 가 검증 범위 `[0.45, 0.65]` 상한을 넘으면
  계획 단계에서 `410` 으로 거절한다. 기준점 y=0.600 이므로 산술적 상한은 정확히 **−50 mm**.
  로봇이 무리하게 시도하지 않고 멈춘다 — 안전한 실패다.
- **+ 쪽**: 관절 여유가 아니라 **선반 칸 안에서의 x 정렬**. +60 mm 에서 min_margin 은 0.171 로
  아직 0.15 위인데 옆 책과 14 mm 겹쳤다. 즉 min_margin 을 아무리 지켜도 겹침이 먼저 깨진다.

**따라서 min_margin 0.15 는 단독 합격 기준이 될 수 없다. 겹침 검사가 실질적 구속이다.**
(min_margin 추세: +쪽으로 약 −0.0014 rad/mm, −쪽으로 약 −0.0007 rad/mm. 양쪽 다 줄지만
어느 쪽도 봉투·겹침보다 먼저 0.15 에 닿지 않는다.)

## 측정 중 걸린 것
- `A_m30` (+30 mm 1차) 는 `manipulation_node` 의 action feedback publisher 가 파지 39 초 뒤
  `context is invalid` 로 죽어 무효였다. 재측정(`B_p30`)에서 정상 통과. 운동학 문제가 아니다.
- `B_p30` 1차 실행은 Isaac Sim 이 `app ready` 직후 죽어 시뮬 준비 단계에서 중단됐다
  (연속 실행 부팅 크래시, 기존 4 회 관측과 동일 징후). 20 초 간격을 두면 재현되지 않는다.
  → 야간 배치에서는 실행 사이에 **20 초 이상** 띄울 것.

## 재현
```
bash night/run_vision.sh <ID> "SIM_PICK_Y=<값> SIM_GOAL_X=-0.3382 SIM_RELEASE_OPEN_FIRST=1 \
  SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing SIM_TRACE_CARRY=1 \
  SIM_HOME_Q=-2.3634,-0.7300,-0.6933,-2.1884,-0.4396,1.5845,-0.5540"
python3 night/summarize_runs.py <ID>
```
(`SIM_USD=~/levels/Final_Level_Library/Final_Level_Library.usdc`)
