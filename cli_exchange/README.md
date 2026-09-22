# night/0922 반출본 (2026-09-23 00시)
night/ 는 .gitignore 라 원격에 안 올라가서 아침 시연용 실행 스크립트·문서를 여기에 복사했다. 원본은 이 PC(BryanKUBT) ~/b1_arm/night/.
스크립트는 `~/b1_arm`·`night/runs/`·`night/cleanup_demo.sh` 경로를 가정한다 — 다른 PC 에서는 `mkdir -p night && cp cli_exchange/scripts/*.sh night/` 후 쓰거나 경로를 고쳐 쓸 것 (지금 리팩터링 안 함).

최선 조합(v11~v15 연속 5/5 성공):
```bash
bash night/run_vision.sh demo_best "SIM_PICK_Y=-3.049 SIM_GOAL_Y=0.5795 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing"
```
