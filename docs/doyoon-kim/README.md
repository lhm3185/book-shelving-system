# D 김도윤 — 로봇팔·그리퍼 / 에셋 문서

| 폴더 | 내용 |
| --- | --- |
| `manipulation/` | 로봇팔 설계·검증 문서 |
| `vision_data/` | 비전 학습데이터 인계 문서(책·서가), 비전 코드 검토 |
| `web_claude/` | 웹 클로드 검토 보고서 v1~v8, v20. 시행착오·학습자료 원자료 |
| `results/20260917_gpu/` | Isaac 시험 원본 로그·결과 JSON (기준선·시행착오 수치의 근거) |
| `proposals/` | 다른 담당자 소유 파일에 대한 **반영 요청안** |

## manipulation/

| 문서 | 읽을 사람 | 핵심 |
| --- | --- | --- |
| `COORDINATE_CONTRACT.md` | **전원** (합의 필요) | 좌표 계약 5줄, 사고 5건, Isaac 카메라 프레임 규약, TargetSlot 필드 의미 |
| `FRAMES_CONTRACT.md` | 전원 | 팀 프레임 ↔ Franka 프레임, 주행 안전 자세(stow) |
| `PLACEBOOK_SIM_LINK.md` | 이현민 | PlaceBook 서버 사용법·검증·트러블슈팅 |
| `VISION_ISAAC_TEST.md` | 윤재민 | 손목 카메라 ↔ vision_manager 연동 시험, 전달 사항 3건 |
| `BASELINE.md` | 전원 (05 문서 입력용) | 사이클 타임·반복성·배치 정확도 |
| `ARM_PLAN.md` | — | 공식 상세 문서 기준 1차 개발 계획 (9/16) |
| **`DEMO_CHECKLIST.md`** | **나 (시연 당일)** | **한 장짜리.** 터미널 4개 순서, 증상→조치, 말할 것/말하지 말 것 |
| `DEMO_20260921.md` | 나 · FSM | 시연 실행 순서 전문 (터미널 단위, 기대 출력 포함) |
| `SETUP_NEW_PC.md` | 나 | 새 PC 에 환경 만들기. git 에 있는 것 / USB 로 가져올 것 구분 |
| `M406_ANALYSIS.md` | 나 | 운반 중 책이 손에서 어긋나는 문제 — 가설과 실험 4가지 |
| `ARMFRAME_MIGRATION.md` | 나 (9/30 준비) | 삽입 경로가 월드 축을 쓴다. 팔 기준 전환 작업 계획 — **주행이 붙기 전에 해야 한다** |
| `INTEGRATION_DEMO.md` | — | **낡음(9/17, Franka).** 부록의 운영 규칙만 유효 |

## scripts/ — 실행 진입점

| 스크립트 | 쓰는 사람 | 무엇 |
| --- | --- | --- |
| `scripts/demo/sim_up.sh` | **FSM · 나** | Isaac 기동 (ssh 자동). `PROFILE=pick\|shelf` |
| `scripts/demo/nodes_up.sh` | FSM · 나 | 비전 + 로봇팔 노드 |
| `scripts/demo/status.sh` | FSM · 나 | 명령 보내기 전 연결 확인 |
| `scripts/demo/pick.sh` | FSM · 나 | 책 한 권 명령 (좌표는 프로파일에서) |
| `scripts/run_tests.sh` | 전원 | 테스트 110개. **Isaac 없이 돈다** |
| `scripts/setup_check.sh` | 새 PC | 무엇이 빠졌는지만 본다 (설치 안 함) |
| `scripts/setup_ubuntu.sh` | 새 PC | ROS 2 Jazzy·빌드 도구·파이썬 의존·Isaac 내려받기 |
| `scripts/regenerate_levels.sh` | 새 PC | 원본에서 `level_yaw0` / `level_shelf01` 재생성 |

`scripts/demo/README.md` 에 FSM 담당자용 4개 명령 안내가 있다.

## results/

| 폴더 | 내용 |
| --- | --- |
| `20260921_gripper_visual/` | **그리퍼가 화면에 안 보이는 문제** — 증거 이미지·원인·수정안 (에셋은 AMR 담당 소유) |

## proposals/

| 파일 | 대상 소유자 | 요청 |
| --- | --- | --- |
| `arm_frames.launch.py` | 이현민 (launch) | `<팔 베이스>→arm_base_link` 정적 TF. PC B launch 에 포함 요청. **부모 이름을 박지 말 것** — franka `panda_link0` / m0609 `base_link` 로 다르다 (2026-09-20 정정). `run_demo_pc.sh` 가 `robot_profiles` 에서 읽는 방식을 그대로 쓰면 된다 |

## 경로 대응

| 문서 속 표기 | 저장소 위치 |
| --- | --- |
| `isaac_sim/…` | `ros2_ws/src/shelving_manipulation/isaac_sim/…` |
| GPU PC `~/arm/…` | 위 `isaac_sim/` 을 GPU PC 에 복사한 위치 (`isaac_sim/README.md`) |
| 에셋 | `simulation/assets/doyoon-kim/` |
