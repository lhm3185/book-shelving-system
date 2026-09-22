# 야간 TODO — 위에서부터. [ ] 미완 / [x] 완료 / [~] 포기(LEDGER 에 사유)
# 세부 규칙·결정 규칙·통과 기준은 NIGHTLY_PLAN.md 의 같은 번호 블록에 있다.

## 블록 0 — 준비와 기준선 (~23:15)
- [x] 0-0 **즉시 `git push origin night/0922`** — 아침 명령(SUMMARY 맨 위)이 c6066b8 코드(RELEASE_OPEN_FIRST·SIM_GOAL_*)에 기댄다. 원격은 아직 1e48669. 커밋할 때마다 바로 push. 확인: `git ls-remote origin refs/heads/night/0922`
- [x] 0-0b **실행 스크립트 반출 (지금, 10분)** — 도윤님은 교육장 GPU PC 에서 시연하는데 night/ 는 .gitignore 라 아침 명령(run_vision.sh)이 이 PC 밖으로 못 나간다. `mkdir -p cli_exchange/scripts && cp night/run_vision.sh night/run_rec.sh night/cleanup_demo.sh cli_exchange/scripts/ && cp night/BASELINE_0922.md night/handoff/SUMMARY.md night/SUCCESS_CRITERIA.md cli_exchange/` + cli_exchange/README.md 한 줄('night/ 경로를 가정하는 곳은 다른 PC 에서 고쳐 쓸 것 — 지금 리팩터링 안 함') → git add cli_exchange && 커밋 → git push origin night/0922. **그다음 남은 시간 배분(랩탑 세션 권고):** ① 5회째 판(성공기준 9) ② **F-2 아침 보고서·F-3 시행착오 문서에 나머지 시간 전부** — 블록 1·2·3·4·5·S·A 의 남은 항목은 시작하지 말고 [~] '시간 — 인계 우선' 으로. F-3 필수: 기각된 가설(얕게·8cm 외삽·GOAL_Y 표·JUDGE 폭 단위·'손가락이 안 잡음'·이송 차이·놓는 순서·ROT90 원인설), 남은 흠 1순위 = f140 reorient 중 책 45° 기울어짐 + 손 솟음(d reorient 관절공간 대체 구간, 고치지 말 것). 시간 남으면 0-1i(단계 step 로그)만. **[완료 00:22] 310db26 push.**
- [x] 0-1 키트 동작 확인(night/hook.log 에 차단 기록이 쌓이는지) · 태그 nightly-base-0922 · 녹화 도구 점검(정면+측면 시뮬 카메라, 10초 시험 영상)
- [x] 0-1b **기준선 재현 (최우선, 0-2 보다 먼저)** — `night/BASELINE_0922.md` 의 6단계(비전 포함, --start-home 기본 move, full_cycle.py --speed 0.6)로 1회. 기대 403 @ RETREATING. 안 나오면 (a) sim.log 의 `스위치:` 줄 (b) approach 목표 팔 기준 vs 저녁 (-0.3633,+0.0088,+0.195) (c) 베이스 실측 위치·yaw 를 LEDGER 에. drive_job.py 로 대신하지 말 것. v29.md 는 원격에 올라옴(BASELINE 맨 아래)
- [x] 0-1d **책 낙하 원인 판별 (최우선 — 403 보다 먼저)** — v01_base 로그 근거: `[JUDGE] width=13.6mm (book 35.2±3 → ng)` = 손가락이 책을 안 잡았다. 키네마틱파지가 가려서 rise 16.7cm 는 ok. `[배치] 책 충돌을 다시 켰다` 0.44 s 뒤 `[놓음]` 에서 책 중심 z 0.075(이미 바닥) → 충돌 재활성·부착 해제 순간 낙하. 또 carry_rotate 중(403추적 관절6 100% 구간) 트레이 중심이 (2.482,-3.328)→(2.509,-3.495) 17 cm 밀림. 판별(해석표: night/ROT90_DISCRIMINATION.md): 같은 기준선 조합에서 **SIM_GRIP_ROT90=0** 한 판 → JUDGE width 가 책 두께 ±3mm 로 ok 인지, 놓은 뒤 책 위치·checks. ok 면 GRIP_ROT90 이 물림축을 틀리게 돌린 것(삽입 자세 HORIZ 와 90° 어긋남 의심) — 파지·삽입 물림축을 일치시키는 수정은 스위치 뒤로. **성공은 checks 5개 true + fallen_books 비었을 때만** (SUCCESS_CRITERIA).
- [x] 0-1e **삽입 목표 교정 (최우선)** — v05 에서 [놓기직전] 책 y −2.836 = 서가 앞면 −2.575 보다 26 cm 앞(허공에서 놓아 낙하). 원인(랩탑 세션 확인): 저녁에 파지 자리를 SIM_PICK_Y −3.019 → −3.324 로 30.5 cm 물렸는데 삽입 목표는 팔기준 --goal-y 0.5495(pick_from_vision 기본값, 옛 자리에서 잡힌 값)로 고정 → 월드 y −3.324+0.5495 = −2.7745, 앞면에서 20 cm 모자람. 한 번에 하나씩: **(a) 먼저** SIM_PICK_Y=-3.019 로만 바꿔 기준선 1판(옛 검증 조합 복원, −3.019+0.5495=−2.4695 = 서가 안). (b) 그다음 SIM_PICK_Y=-3.324 + goal-y ≈0.83(유도값, 서가 중심 y≈−2.495) — full_cycle.py 가 pick_from_vision 에 인자를 안 넘기므로 goal-y 를 환경변수(예 SIM_GOAL_Y, 기본값=0.5495 불변)로 넘기는 스위치 필요. 각 판에서: 회전 뒤 panda_link0 월드 좌표 실측(가정 = 루트+0.30 → (2.835,−3.324)), [놓기직전] 책 y vs 서가 앞면, plan_job floor_z·grip_z vs 실제 선반판 z. checks 5개 + fallen_books 로 판정, 성공이면 SUCCESS_CRITERIA 대로 녹화 검증.
- [x] 0-1f **관절6 여유 확보 → 연속 2회 성공 → 아침 명령 갱신 → 녹화 1판 → (시간 남으면) 5회 반복** — v08 이 v07 조합으로 401(IK 가 관절6>3.0 해 50번+ 버림 → 다른 가지 → 역스윙 q1 +3.232 한계 밖). ① v07 조합 + SIM_GOAL_Y 를 0.54 → 0.53 → 0.52 로 한 판씩, 판마다 표: goal_y | 경로검사 wedge 관절6 한계여유 | checks 5개 | [놓음] y. **합격선 = 관절6 한계여유 ≥ 0.10 rad 이면서 checks 5개 true** (한 판 통과는 합격 아님 — 여유 −0.000 도 한 판은 통과했다, 비전이 판마다 ~1 cm 흔들림). ② 합격값으로 **연속 2회 성공**이면 night/handoff/SUMMARY.md 맨 위 명령을 그 조합으로 갈아 끼우고 녹화 1판(정면+측면). ③ 역스윙 버그(별도 스위치, 기본 불변): 역 b 가 `q_b[0] -= dphi` 상대값이라 중간에 j1 이 바뀌면 틀린다 → **홈 관절값 q_home[0] 절대값으로** 가게. ① 이 먹어도 넣어 둘 것. ④ 씨앗·j1 제한은 마지막 수단, 쓸 때는 '앞 자세와 가장 가까운 해' 기준(통과할 때까지 바꾸기 금지). 커밋할 때마다 즉시 git push origin night/0922. **[갱신 00:1x] goal-y 얕게는 기각(j6 커짐). 뒤로 빼기: GOAL_Y = 0.5495 + (−3.019 − PICK_Y) 로 월드 목표 유지. 3 cm(부호 확인) → 8 cm(−3.099/0.6295) → 30.5 cm(−3.324/0.8545) 양끝 먼저. 04:00 까지 여유 ≥0.10 못 넘으면 v09/v10 조합으로 0-1h 측면 녹화·SUMMARY 먼저.** **[갱신] v11(PICK_Y −3.049/GOAL_Y 0.5795) 합격 — 이 조합으로 반복. 판마다 `관절1 최소여유`(wedge~retreat 경로검사)와 스윙 Δφ 를 한 줄에 기록 → 여유 분포로 판단(제약이 6→1 로 옮겨갔고 관절1 은 v08 에서 터진 관절, 스윙 Δφ 가 곧 j1 소비). 여유 되면 5회 중 1~2판은 다른 책 — 비전 경로는 비전이 책을 고르므로 book_index 가 안 먹는다(drive_job 경로 전용) → 다른 책을 고르게 하는 방법이 없으면 '막힘' 으로 적고 넘어간다.** **[완료] v11~v15 demo_best 5/5, 관절1 여유 0.159~0.160.**
- [x] 0-1h **측면 녹화 (성공기준 5: 책이 칸에 꽂히는 것)** — 합격 조합이 연속 2회 나온 뒤. A 서가와 나란한 측면(권장): `--record-eye 0.60 -2.90 0.95 --record-look 2.60 -2.60 0.75`, 가능하면 B 남쪽 정면 동시: `--record-eye 2.70 -5.00 1.10 --record-look 2.60 -2.75 0.80`. 그다음 SUMMARY.md 갱신. **[완료] v14 측면 녹화(격자 night/handoff/v14_side_grid.jpg) + v15 뒤쪽.**
- [ ] 0-1i (나중, 스위치 뒤·기본 꺼짐) 단계 전환 때 `[단계] <phase> 시작 step=N` 한 줄 — 프레임 = step / record_every 로 '어느 구간이 솟는가' 를 추측 없이 확정. 지금은 기록만, 고치지 말 것.
- [ ] 0-1g RELEASE_OPEN_FIRST=0 한 판 (기준선에서 이것만 끔) — 삽입 성공에 필요 없으면 빼서 기본 동작 변경을 하나 줄인다
- [x] 0-1c 성공 판정 규칙 숙지 — 삽입이 됐다고 판단하려면 `night/SUCCESS_CRITERIA.md` 1~9 전부 + 성공 판은 같은 조건으로 재실행해 정면·측면 녹화. 판정 결과는 LEDGER `판정:` 줄로. 이 항목은 읽고 바로 [x]
- [x] 0-2 지금 코드·기본 스위치로 전체 한 바퀴 1회 + 영상("before" 영상). 결과·오류코드·단계별 시각 기록

## 블록 1 — 406 판정식 (~00:00)
- [~] 1-1 책 중심 = 자세 ⊗ c_loc(형상 중심) + 손 기준 회전각 로그 추가. 옛 판정식과 나란히 기록(판정은 안 바꿈) **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] 1-2 SIM_DRIFT_METRIC=center 스위치로 판정 전환 + 1회 실행해 두 값 비교표 **[~] 시간 — 인계 우선(0-0b 지시).**

## 블록 2 — 운반 동선 (~02:45) ← 가장 중요
- [x] 2-1 진단만: carry_rotate 양 끝 관절각·관절별 Δ, 관절공간 경로의 FK 손끝 z 최대·위치, 직교로 풀 때 튐 지점(t, 관절, 직전 값의 한계 거리) **[완료] SIM_TRACE_CARRY 로 v02 에서 측정: carry_rotate(관절) z최대 +0.157, Δq j1 −1.443·j5 −1.512 rad.**
- [x] 2-2 계획 단계 경로 검사(SIM_PATH_AUDIT=1): 모든 구간 FK 표본으로 z 상한·트레이/서가 여유·한계 여유·최대 걸음 → 위반이면 401 + 위치 포함 메시지 **[완료] book_scene.py _audit(SIM_TRACE_CARRY/SIM_PATH_AUDIT) — 판마다 [경로검사] 줄(z·한계여유·최대걸음).**
- [x] 2-3 스윙 동선(SIM_CARRY_MODE=swing): lift → j1 만 도는 스윙 → 직교 reach → 짧은 reorient → pre_ins. 영상 1회 **[완료] v07 swing 성공(0-1f 로 흡수).**
- [x] 2-4 결정 규칙대로 채택/보류 판정 + 반납(return) 구간도 역스윙으로 같은 처리 **[완료] 채택 — 운반 swing + 반납 거울(SIM_SWING_MIRROR). d reorient 는 직교 실패→관절공간 대체(흠).**

## 블록 3 — 403 (~03:45)
- [~] 3-1 SIM_SPEED_SCALE 이 관절공간 구간에 실제로 먹는지 확인(계획 속도 최대값 1.0 vs 0.5 비교) + RETREATING 구간 r 시험 **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] 3-2 계획 단계 재시간 배분(SIM_RETIME=1): 속도 한계 70% 로 경유점 간격 재배분 → 계획 속도 표 + 1회 실행 **[~] 시간 — 인계 우선(0-0b 지시).**

## 블록 4 — 파지 깊이 (~04:30)
- [~] 4-1 손 바닥면 오프셋(TCP→손가락 뿌리) 실측 + 책별 TIP_DOWN 자동 계산(SIM_TIP_AUTO=1) + down 구간 트레이 변위·실제 TCP 최저 z 기록 **[~] 시간 — 인계 우선(0-0b 지시).**

## 블록 5 — 삽입 정직성 (~05:00)
- [~] 5-1 잡은 책 콜리전 on/off 를 단계별로 로그. 삽입(wedge~push) 중 켜져 있는지, 꽂힌 뒤 이웃 책·선반판과 겹침 여부 **[~] 시간 — 인계 우선(0-0b 지시).**

## 블록 6 — 반복 (~06:15)
- [x] 6-1 채택한 스위치 전부 켜고 전체 한 바퀴 5회(매번 Isaac 재시작) + 매회 영상 → n/5 표 **[완료] v11~v15 5/5 매번 Isaac 재시작(녹화 v14 측면·v15 뒤쪽). 책 1권·칸 0 만.**

## 블록 A — AMR 자율주행 결합 (블록 6 뒤. 06:30 이 되면 어디서든 끊고 마무리로)
- [~] A0 temp/DELETE-AFTER-night-transfer-0922 브랜치 · cli_exchange/ + amr_integration/ 폴더 · 887efe7 의 AMR/FSM 패키지 원본 가져오기 · 의존성 확인 · colcon build **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A1 amr_integration/ARCHITECTURE.md (노드·토픽·액션·프레임·FSM 흐름 + 어긋남 10개 확인/기각) + waypoints_v5.yaml 초안 **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A2 bringup_check.sh: 도메인 130 · ROS2Context 도메인 설정 · /clock 발행자 1개 · 라이다→scan · TF 한 트리 · cmd_vel 구독자 **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A3 Nav2 기동(SIM_NAV_MODE=nav2, SIM_TRAY_CARRY=0) · RViz 창 캡처 · 지도↔월드 정렬 측정 (필요하면 v5 지도 새로) **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A4 /navigate_to_target 단일 목표 home→shelf_01→home 3회 · 도착 후 plan_job 드라이런 · (필요 시) 두 단계 접근 **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A5 FSM 전 과정 (task_manager + navigation_node + Nav2 + 우리 /place_book + 비전) · 처음 실패 상태까지 기록 **[~] 시간 — 인계 우선(0-0b 지시).**
- [~] A6 amr_integration/README.md · STATUS.md · patches/README.md **[~] 시간 — 인계 우선(0-0b 지시).**

## 마무리 (블록 6 뒤, 블록 A 중이면 06:30 에 끊고)
- [x] F-1 모든 새 스위치 끈 전체 한 바퀴 1회 (회귀 — 0-2 와 같은 결과여야 함) **[완료] 409(v02 406) — 계획 동일, 물리 확률 차. 보고서 맨 위.**
- [ ] F-2 NIGHTLY_20260923.md 아침 보고서 + night/handoff/SUMMARY.md
- [ ] F-3 시행착오 문서: 오늘 밤 항목을 우선순위 순서(블록 2 → 3 → 1 → 4 → 5 → 6 → A)로, 한 항목 한 페이지 (night/handoff/trial_and_error_0922.md, 형식은 NIGHTLY_PLAN 마무리 절)
- [ ] F-4 night/handoff/ 정리(영상·사진·media_index.md·LEDGER 사본) → 로컬 커밋 → `git push origin night/0922` (거부되면 생략, NIGHTLY_PLAN 맨 위) → touch night/DONE **마지막에 깨끗한 상태로:** cli_exchange/SUMMARY.md 를 night/handoff/SUMMARY.md 최신본으로 다시 복사·커밋·push → Isaac·ROS 노드 전부 PID 로 내리기(bash night/cleanup_demo.sh, 잔여 0 확인) → touch night/DONE (run_night 가 ACTIVE 를 지우고 끝난다). 아침 첫 명령이 깨끗한 기계에서 시작해야 한다.
- [ ] F-5 (선택, F-2·F-3 끝나고 시간 남으면) 민감도: demo_best 에서 SIM_GOAL_Y 0.5745·0.5845·0.5695·0.5895 (±5·±10 mm) 4판 — 판마다 관절1·6 최소여유만 기록, ≥0.10 유지 경계를 문서에. 안 해도 '5판 입력 흔들림 1.1 mm, 1 cm 급 미시험' 기록으로 충분.
- [ ] F-6 (선택, 문서 뒤) 비결정성 출처 확인: 판마다 refresh_base() 직후 팔 베이스 월드 좌표와 계획 씨앗 관절값을 한 줄 기록 — 입력이 같아도 이것이 판마다 다른지.
