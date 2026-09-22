> ⚠️ **작업 브랜치를 지우지 말 것.** 오늘 밤(2026-09-22~23) 작업 전부가 이 PC 의 로컬 브랜치 `night/0922` 에 있다.
> 이 브랜치는 원격 `temp/DELETE-AFTER-night-transfer-0922` 에서 땄다. 원격 temp 를 지워도 로컬 night/0922 는 남는다. 원격 백업은 `origin/night/0922` (23:4x 부터 push 허용, 블록마다 push).

## ▶▶▶ 아침에 쓸 명령 — 최선 (v11·v12 연속 2회 성공, 관절 여유 합격)
```bash
cd ~/b1_arm && git branch --show-current   # night/0922
bash night/run_vision.sh demo_best "SIM_PICK_Y=-3.049 SIM_GOAL_Y=0.5795 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"
```
- v11·v12: success=True code=0 verified=True · checks 5개 true · 책 서가 y −2.531 · 트레이 제자리 · 403 없음
- 삽입 구간 관절 최소여유 0.160~0.221 rad (관절 1) — v09/v10 조합(관절 6 여유 ≈0)보다 안정적. 스윙 Δφ −43.0°
- 측면 녹화(성공기준 5)·5회 반복은 진행 중. 손이 트레이 위에서 솟는 모양은 남아 있다(v09 영상).
- 로봇을 3 cm 뒤로 빼고(PICK_Y) 삽입 지령을 같은 양 늘린(GOAL_Y) 것 = 월드 삽입 위치는 v09 와 같다.

- ⚠️ `night/` 는 .gitignore 라 run_vision.sh·run_rec.sh 등 실행 스크립트는 **이 PC 에만** 있다. 다른 PC 에서는 코드(origin/night/0922)는 받을 수 있지만 실행은 night/BASELINE_0922.md 6단계를 직접 치거나 night/ 폴더를 따로 복사해야 한다.

## ▶ (대비책) 이전 스윙 조합 (00:2x 기준, 스윙 동선 — v07_swing 에서 code=0 성공)
```bash
cd ~/b1_arm && git branch --show-current   # night/0922 여야 한다
bash night/run_vision.sh demo_morning "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"
```
- v07 결과: success=True code=0 verified=True, checks 5개 true, fallen_books 없음, 403 스파이크 0, 트레이 밀림 5 mm 이하(운반 전후 같은 자리).
- 반복 횟수·녹화 검증 결과는 아래 표/LEDGER 에서 갱신된다 (1회 성공은 아직 성공이 아니다 — SUCCESS_CRITERIA 9).
- 이전 조합(v06, 스윙 없이): `bash night/run_vision.sh demo_morning "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1"` — 삽입은 되나 트레이 17 cm 밀림 + 403.
> **검증 범위: 책 1권(cover58_01)·트레이 칸 0 만.** 다른 칸·책은 파지 자세와 스윙 각(Δφ)이 달라 관절 1·6 여유가 다르다 — '5권 다 되나' 는 아직 모른다.


