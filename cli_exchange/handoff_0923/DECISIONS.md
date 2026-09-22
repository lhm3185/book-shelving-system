# 사람이 결정할 것 (2026-09-23 아침)

## 제출 항목
1. **아침 시연 조합을 demo_best 로 할지** — v11~v15 5/5. 대비책은 v07 조합(PICK_Y −3.019, GOAL_Y 기본; 관절6 여유 ≈0 이라 v08 처럼 401 이 날 수 있음).
2. **시연 PC** — 교육장 GPU PC 에서 돌릴 거면 cli_exchange/scripts 의 경로(~/b1_arm, ~/Desktop/ing_library_env_v5.usd, DISPLAY=:1)를 그 PC 에 맞출 것. 한 번은 미리 돌려 볼 것 (이 PC 외에서는 검증 안 됨).
3. **reorient(d) 손목 특이점** — 지금은 직교 실패 시 관절공간으로 대체(책 45° 기울고 손 솟음). 고칠지: (a) d 의 목표 자세 순서를 바꾸기(손목 먼저) (b) d 를 j4~j7 만 도는 관절 순수 회전으로 나누기 (c) 그대로 둠. 시연 품질만 문제면 (c).
4. **검증 범위 1권·1칸** — 5권 연속을 제출에 넣을지. 넣으려면 full_cycle → pick_from_vision 인자 전달(book_index) 부터 고쳐야 다른 책 시험 가능.
5. **기본값으로 올릴지** — 오늘 스위치(swing·RELEASE_OPEN_FIRST·PICK_Y/GOAL_Y)는 전부 기본 꺼짐. 기본으로 올리면 F-1 회귀(스위치 끈 판)가 기준선이 아니게 된다.
6. **night/0922 정리** — 이관용 에셋 240MB·.claude/ 설정이 커밋돼 있다. 코드 변경(book_scene.py·manipulation_executor.py·pick_from_vision.py)만 골라 feature 브랜치로 옮길 것.

## 도구
7. JUDGE width 단위(반폭 ×2 vs 두께) 수정 — 판정식이 바뀌므로 사람 확인 뒤.
8. SIM_SPEED_SCALE 이 관절공간 구간 속도에 안 먹는 것 — 스윙에선 403 이 안 나 급하지 않음.

## AMR (블록 A) — 손대지 못함. 전부 [~].
