# D 김도윤 — 로봇팔·그리퍼 / 에셋 문서

| 폴더 | 내용 |
| --- | --- |
| `manipulation/` | 로봇팔 설계·검증 문서 |
| `vision_data/` | 비전 학습데이터 인계 문서(책·서가), 비전 코드 검토 |
| `web_claude/` | 웹 클로드 검토 보고서 v1~v8 (v6~v8 로봇팔, v8 은 학습자료 원자료) |
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
