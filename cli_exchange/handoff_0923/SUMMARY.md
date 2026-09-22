> ⚠️ **작업 브랜치를 지우지 말 것.** 오늘 밤(2026-09-22~23) 작업 전부가 이 PC 의 로컬 브랜치 `night/0922` 에 있다.
> 이 브랜치는 원격 `temp/DELETE-AFTER-night-transfer-0922` 에서 땄다. 원격 temp 를 지워도 로컬 night/0922 는 남는다. 원격 백업은 `origin/night/0922` (23:4x 부터 push 허용, 블록마다 push).

## ▶▶▶ 아침에 쓸 명령 — 최선 (v11~v15 연속 5/5 성공, 매번 Isaac 재시작, 관절 여유 합격)
```bash
cd ~/b1_arm && git branch --show-current   # night/0922
bash night/run_vision.sh demo_best "SIM_PICK_Y=-3.049 SIM_GOAL_Y=0.5795 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"
```
- v11~v15: success=True code=0 verified=True · checks 5개 true · 책 서가 y −2.531 · 트레이 제자리 · 403 없음
- 삽입 구간 관절 최소여유 0.159~0.160 rad (관절 1) — v09/v10 조합(관절 6 여유 ≈0)보다 안정적. 스윙 Δφ −43.0°
- 성공기준 1~9 충족 (5 는 v14 측면 영상 night/handoff/v14_side_grid.jpg, 6 은 한 각도만). 손이 트레이 위에서 솟고 reorient 중 책이 45° 기우는 모양은 남아 있다(v09·v14 영상) — 고치지 않음.
- 상세: night/handoff/NIGHTLY_20260923.md · 결정할 것: DECISIONS.md · 시행착오: trial_and_error_0922.md
- 로봇을 3 cm 뒤로 빼고(PICK_Y) 삽입 지령을 같은 양 늘린(GOAL_Y) 것 = 월드 삽입 위치는 v09 와 같다.

- ⚠️ `night/` 는 .gitignore 라 run_vision.sh·run_rec.sh 등 실행 스크립트는 **이 PC 에만** 있다. 다른 PC 에서는 코드(origin/night/0922)는 받을 수 있지만 실행은 night/BASELINE_0922.md 6단계를 직접 치거나 night/ 폴더를 따로 복사해야 한다.

## 아침에 demo_best 가 실패하면 — 이 4줄부터 (로그: night/runs/<ID>/)
1. Isaac 이 하나만 떠 있나 — `ps -eo pid,args | grep -c "[r]un_simulation"` → 1 이어야 한다 (2 이상이면 /clock·TF 충돌. 남은 것을 PID 로 끄고 다시)
2. 스위치가 먹었나 — sim.log 의 `스위치:` 줄에 `그리퍼90도=True … 운반=swing 복귀=swing`
3. 트레이가 실렸나 — sim.log `[이송] 트레이 **실측**` 오차 < 1 cm
4. 비전이 좌표를 냈나 — cyc.log 에 `비전 좌표(윗면 중심)` 줄. 5판 모두 ≈ (−0.3622, +0.005~0.007, +0.195) 팔기준
- 대비책(한 줄): `bash night/run_vision.sh demo_v07 "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"` (4판 중 3판 성공, 관절6 여유 ≈0)
- 교육장 GPU PC 에서는 `night/` 대신 `cli_exchange/scripts/` 경로 (310db26).

**재현성 범위 (정직하게):** 5/5 동안 비전 입력은 y 1.1 mm·z 0.1 mm 만 흔들렸다. 관절 여유 0.159~0.160 이 일정했던 것이 '입력 흔들림에 둔감해서'인지 '입력이 거의 같아서'인지는 이 5판으로 가를 수 없다. 비전이 1 cm 흔들리는 경우(v07 vs v08 수준)는 이 조합으로 시험하지 않았다.

## ▶ (대비책) 이전 스윙 조합 (00:2x 기준, 스윙 동선 — v07_swing 에서 code=0 성공)
```bash
cd ~/b1_arm && git branch --show-current   # night/0922 여야 한다
bash night/run_vision.sh demo_morning "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"
```
- v07 결과: success=True code=0 verified=True, checks 5개 true, fallen_books 없음, 403 스파이크 0, 트레이 밀림 5 mm 이하(운반 전후 같은 자리).
- 반복 횟수·녹화 검증 결과는 아래 표/LEDGER 에서 갱신된다 (1회 성공은 아직 성공이 아니다 — SUCCESS_CRITERIA 9).
- 이전 조합(v06, 스윙 없이): `bash night/run_vision.sh demo_morning "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1"` — 삽입은 되나 트레이 17 cm 밀림 + 403.
> **검증 범위: 책 1권(cover58_01)·트레이 칸 0 만.** 다른 칸·책은 파지 자세와 스윙 각(Δφ)이 달라 관절 1·6 여유가 다르다 — '5권 다 되나' 는 아직 모른다.


