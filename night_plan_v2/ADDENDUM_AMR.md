# 추가 지시 — AMR 자율주행 결합 + 작업 브랜치 변경 (2026-09-22 23:30)

CLI 에 그대로 붙여 넣는다. 야간 루프가 **이미 돌고 있어도** 아래 1~3 만 하면 이어서 반영된다.

## 무엇이 바뀌나 (요약)

1. **작업 브랜치**: 오늘 밤 모든 작업은 `temp/DELETE-AFTER-night-transfer-0922` 에서 한다 (feature/robot_control 이 아니다). 이 임시 브랜치에는 블록마다 push 해도 된다
2. **폴더 두 개**:
   - `cli_exchange/` — CLI 끼리 주고받는 자료 (기존 `night_transfer/`·`night_plan/`·`night_kit_night/` 도 이 아래로)
   - `amr_integration/` — AMR 결합 코드·설정·검사·패치·문서
3. **블록 A 추가**: AMR/FSM 담당자가 21:58 에 커밋한 Nav2(`origin/feature/1st_combine_integration_test` @ `887efe7`)를 가져와 FSM 흐름에 우리 로봇팔+비전을 결합한다. 좌표를 받으면 코스트맵 기반 Nav2 주행, RViz 로 확인
4. **순서는 그대로**: 제출 항목(블록 0~6: 트레이 이송·파지·삽입)이 먼저다. 끝나면 블록 A. **06:30 이 되면** A 를 끊고 마무리(인계 문서)로 간다
5. **ROS 도메인은 130 으로 통일.** 담당자 코드에 도메인 0 이 있다고 했다(export 누락, USD `ROS2Context` 노드, 하드코딩 셋 중 하나). 교육장 GPU PC 시연은 129/130 이어야 한다
6. 담당자 말대로 AMR 코드는 **완벽하지 않다.** 코드 읽기로 찾은 어긋남 10개를 계획서 블록 A 에 적어 두었다 — 확인/기각부터 한다. 원본은 고치지 않고 패치·덮어쓰기 설정으로만 다룬다

## 반영 방법

**1. 계획 파일 교체** (`night_plan_0922_v2` 폴더에서)
- `NIGHTLY_PLAN.md` 는 통째로 교체한다 (규칙 1·5·6 변경 + 블록 A 추가 + 마무리 순서 변경)
- `TODO.md` 는 **통째로 바꾸지 않는다.** 진행 중인 `[x]`·`[~]` 표시를 살리고 아래만 반영한다:
  - 블록 6 과 마무리 사이에 `## 블록 A` 7줄(A0~A6)을 새 TODO.md 에서 복사해 넣는다
  - 마무리 제목, F-3 의 순서(`… → 6 → A`), F-4 의 push 대상(`temp/DELETE-AFTER-night-transfer-0922`)을 새 파일대로 고친다

**2. 브랜치 전환**
- 지금 작업 트리에 커밋 안 한 변경이 있으면 먼저 커밋한다
- `git fetch origin && git checkout temp/DELETE-AFTER-night-transfer-0922`
- 오늘 밤 이미 다른 브랜치에 커밋한 것이 있으면 이 브랜치로 `cherry-pick`
- 야간 루프가 돌고 있으면 **현재 세션이 한 항목을 끝낸 뒤** 전환한다 (Isaac 실행 도중에 브랜치를 바꾸지 않는다)

**3. LEDGER 에 한 줄**: `HH:MM 추가지시 반영 — 브랜치 temp/…, 블록 A 추가, 도메인 130 — 다음: <지금 항목>`

## 데스크탑에서 돌고 있다면

- 저장소 경로는 `~/b1_arm`, 레벨 에셋은 `cli_exchange/night_transfer/README_SETUP.md`(옮긴 뒤 경로)의 심볼릭 링크로 연다
- `/tmp/cl3.sh`·`/tmp/full_run.sh` 가 데스크탑에 없으면 같은 일을 하는 스크립트를 `cli_exchange/scripts/` 에 만들어 쓴다. `pgrep -f` 패턴에 자기 이름을 넣지 않는다
- 데스크탑은 개인 PC 라 **RViz 창 캡처는 허용**한다. 공용 GPU PC 에서는 여전히 시뮬 카메라만
- Nav2 패키지(`nav2_bringup`·`nav2_mppi_controller`·`pointcloud_to_laserscan`)가 없고 설치에 sudo 가 필요하면 **설치하지 않는다.** A0 에서 막힘으로 기록하고, 오프라인 항목(A1·A6)만 한다
