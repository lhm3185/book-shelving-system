# 야간 TODO — 위에서부터. [ ] 미완 / [x] 완료 / [~] 포기(LEDGER 에 사유)
# 세부 규칙·결정 규칙·통과 기준은 NIGHTLY_PLAN.md 의 같은 번호 블록에 있다.

## 블록 0 — 준비와 기준선 (~23:15)
- [ ] 0-1 키트 동작 확인(night/hook.log 에 차단 기록이 쌓이는지) · 태그 nightly-base-0922 · 녹화 도구 점검(정면+측면 시뮬 카메라, 10초 시험 영상)
- [ ] 0-2 지금 코드·기본 스위치로 전체 한 바퀴 1회 + 영상("before" 영상). 결과·오류코드·단계별 시각 기록

## 블록 1 — 406 판정식 (~00:00)
- [ ] 1-1 책 중심 = 자세 ⊗ c_loc(형상 중심) + 손 기준 회전각 로그 추가. 옛 판정식과 나란히 기록(판정은 안 바꿈)
- [ ] 1-2 SIM_DRIFT_METRIC=center 스위치로 판정 전환 + 1회 실행해 두 값 비교표

## 블록 2 — 운반 동선 (~02:45) ← 가장 중요
- [ ] 2-1 진단만: carry_rotate 양 끝 관절각·관절별 Δ, 관절공간 경로의 FK 손끝 z 최대·위치, 직교로 풀 때 튐 지점(t, 관절, 직전 값의 한계 거리)
- [ ] 2-2 계획 단계 경로 검사(SIM_PATH_AUDIT=1): 모든 구간 FK 표본으로 z 상한·트레이/서가 여유·한계 여유·최대 걸음 → 위반이면 401 + 위치 포함 메시지
- [ ] 2-3 스윙 동선(SIM_CARRY_MODE=swing): lift → j1 만 도는 스윙 → 직교 reach → 짧은 reorient → pre_ins. 영상 1회
- [ ] 2-4 결정 규칙대로 채택/보류 판정 + 반납(return) 구간도 역스윙으로 같은 처리

## 블록 3 — 403 (~03:45)
- [ ] 3-1 SIM_SPEED_SCALE 이 관절공간 구간에 실제로 먹는지 확인(계획 속도 최대값 1.0 vs 0.5 비교) + RETREATING 구간 r 시험
- [ ] 3-2 계획 단계 재시간 배분(SIM_RETIME=1): 속도 한계 70% 로 경유점 간격 재배분 → 계획 속도 표 + 1회 실행

## 블록 4 — 파지 깊이 (~04:30)
- [ ] 4-1 손 바닥면 오프셋(TCP→손가락 뿌리) 실측 + 책별 TIP_DOWN 자동 계산(SIM_TIP_AUTO=1) + down 구간 트레이 변위·실제 TCP 최저 z 기록

## 블록 5 — 삽입 정직성 (~05:00)
- [ ] 5-1 잡은 책 콜리전 on/off 를 단계별로 로그. 삽입(wedge~push) 중 켜져 있는지, 꽂힌 뒤 이웃 책·선반판과 겹침 여부

## 블록 6 — 반복 (~06:15)
- [ ] 6-1 채택한 스위치 전부 켜고 전체 한 바퀴 5회(매번 Isaac 재시작) + 매회 영상 → n/5 표

## 블록 A — AMR 자율주행 결합 (블록 6 뒤. 06:30 이 되면 어디서든 끊고 마무리로)
- [ ] A0 temp/DELETE-AFTER-night-transfer-0922 브랜치 · cli_exchange/ + amr_integration/ 폴더 · 887efe7 의 AMR/FSM 패키지 원본 가져오기 · 의존성 확인 · colcon build
- [ ] A1 amr_integration/ARCHITECTURE.md (노드·토픽·액션·프레임·FSM 흐름 + 어긋남 10개 확인/기각) + waypoints_v5.yaml 초안
- [ ] A2 bringup_check.sh: 도메인 130 · ROS2Context 도메인 설정 · /clock 발행자 1개 · 라이다→scan · TF 한 트리 · cmd_vel 구독자
- [ ] A3 Nav2 기동(SIM_NAV_MODE=nav2, SIM_TRAY_CARRY=0) · RViz 창 캡처 · 지도↔월드 정렬 측정 (필요하면 v5 지도 새로)
- [ ] A4 /navigate_to_target 단일 목표 home→shelf_01→home 3회 · 도착 후 plan_job 드라이런 · (필요 시) 두 단계 접근
- [ ] A5 FSM 전 과정 (task_manager + navigation_node + Nav2 + 우리 /place_book + 비전) · 처음 실패 상태까지 기록
- [ ] A6 amr_integration/README.md · STATUS.md · patches/README.md

## 마무리 (블록 6 뒤, 블록 A 중이면 06:30 에 끊고)
- [ ] F-1 모든 새 스위치 끈 전체 한 바퀴 1회 (회귀 — 0-2 와 같은 결과여야 함)
- [ ] F-2 NIGHTLY_20260923.md 아침 보고서 + night/handoff/SUMMARY.md
- [ ] F-3 시행착오 문서: 오늘 밤 항목을 우선순위 순서(블록 2 → 3 → 1 → 4 → 5 → 6 → A)로, 한 항목 한 페이지 (night/handoff/trial_and_error_0922.md, 형식은 NIGHTLY_PLAN 마무리 절)
- [ ] F-4 night/handoff/ 정리(영상·사진·media_index.md·LEDGER 사본) → 회귀 통과 시 temp/DELETE-AFTER-night-transfer-0922 push → touch night/DONE
