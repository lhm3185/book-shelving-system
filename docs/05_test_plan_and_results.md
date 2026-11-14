# 시험 계획 및 결과

## 1. 목적과 기록 원칙

빌드·단위시험·3-PC 준비 상태·한 권 전체 사이클을 분리해서 검증한다. 실행하지 않은 시험은 `NOT_RUN`, 제한된 환경 때문에 판단할 수 없는 시험은 `BLOCKED`, 요구사항을 만족하면 `PASS`, 실제 결함이 확인되면 `FAIL`로 기록한다.

## 2. 시험 계층

| 계층 | 대상 | 통과 기준 |
| --- | --- | --- |
| 정적 확인 | shell/Python/YAML/launch | 구문 오류와 누락 파일 없음 |
| 단위시험 | planner, FSM, perception 변환, 조작 기하·경로 | pytest 핵심 assertion 통과 |
| ROS 계약 | msg/action, action server, status | 동일 인터페이스로 goal/result 교환 |
| 준비 상태 | supervisor | `SystemStatus.READY` |
| 기능 통합 | tray, navigation, perception, manipulation | 각 action 성공과 안전 실패 처리 |
| 전체 회귀 | 한 권 정상 사이클 | 적재→서가→배치→후퇴→HOME→READY |

## 3. 담당별 핵심 시험

### A 이현민 — 시스템

- TrayJob 배열 길이, 빈 값, 중복 book ID 거부
- 분류코드 `005.7`이 `shelf_01`로 계획되는지 확인
- FSM 허용/금지 전이와 scenario reset 초기화
- Supervisor가 scan, AMCL, TF, action이 빠졌을 때 start를 거부하는지 확인
- `LoadTray` JSON bridge와 timeout/cancel 확인
- 완료 기록이 메모리 범위임을 확인하고 YAML/DB 영속화로 오인하지 않기

### B 윤재민 — 비전

- RGB/depth/camera_info 동기화와 최신 프레임 조건
- 트레이 ROI 안 책 후보와 `GraspObservation` frame/치수/confidence
- shelf YOLO 후보, depth 차이와 광선-서가 평면 교차
- `TargetSlot`의 `arm_base_link`, pose, 크기, 삽입 깊이
- 후보 없음, 잘못된 depth, TF 실패와 cancel 결과

### C 이동준 — AMR

- `/lidar/points_raw`→`/scan`, map, AMCL, `map→base_link`
- waypoint가 있으면 NavigateThroughPoses, 없으면 NavigateToPose 사용
- 접근 반경에서 Nav2 취소 후 정밀 position/yaw 정렬
- 최종 target 재시도와 action cancel
- `shelf_retreat` 0.75 m 안전 제한 및 direct cmd_vel 동작
- RViz의 map, scan, global/local costmap, `/plan` 표시

### D 김도윤 — 조작

- 팔 스윕 관측점과 slot 후보 취합·선택
- 선택 슬롯에 맞춘 navigation action 횡정렬
- 책 관측을 트레이 교정 슬롯과 연결하고 유효성 검사
- IK, 연속 경로, 속도와 충돌 전 검사
- 파지·삽입·해제·후퇴 phase와 배치 검증
- cancel, heartbeat timeout, 책 낙하·삽입 막힘의 안전 결과

## 4. 전체 정상 시나리오 시험

| ID | 확인 내용 | 성공 기준 | 상태 |
| --- | --- | --- | --- |
| E2E-01 | PC B 기동 | Nav2/perception/manipulation/RViz 실행 | NOT_RUN |
| E2E-02 | PC A 기동 | Isaac READY, AMCL 초기화, supervisor READY | NOT_RUN |
| E2E-03 | PC C 관제 | 상태·map·pose 표시, start 버튼 활성 | NOT_RUN |
| E2E-04 | 작업 시작 | start service 수락, 유일한 TrayJob 생성 | NOT_RUN |
| E2E-05 | 반납기 이동 | 경유점·정밀 정렬 성공 | NOT_RUN |
| E2E-06 | 트레이 인수 | 실제 tray runtime 적재 성공 | NOT_RUN |
| E2E-07 | 서가 이동 | `shelf_01` 도착·정렬 성공 | NOT_RUN |
| E2E-08 | 스윕·인식 | 유효 슬롯과 트레이 책 선택 | NOT_RUN |
| E2E-09 | 배치 | `PlaceBook.success`와 `placement_verified` true | NOT_RUN |
| E2E-10 | 복귀 | shelf retreat, HOME, FSM IDLE, system READY | NOT_RUN |

`NOT_RUN`은 문서 갱신 시 저장된 실기 결과표가 저장소에 없다는 뜻이다. 팀이 보유한 영상이나 로그가 있으면 commit/run_id/job_id와 함께 아래 표에 추가한다.

## 5. 대표 실패 시나리오

- PC B action 또는 최신 `/scan`이 없음: `/system/start_cycle` 거부
- Timeline reset: 활성 goal 취소, 0 속도 유지, AMCL 초기 pose 재발행
- unknown classification: 계획 단계 실패
- LoadTray timeout/reject: 서가로 출발하지 않고 FSM 실패
- Nav2 실패: 마지막 목표만 제한된 횟수 재시도 후 정지
- 슬롯 후보 없음: 책 인식과 팔 배치 명령을 진행하지 않음
- base 횡정렬 실패: stale slot으로 삽입하지 않음
- 책 후보 없음/잘못된 frame: error 410/411 계열로 조작 중단
- 삽입 막힘/배치 미검증: 성공 기록 없이 안전 동작 후 실패
- cancel: 시뮬 실행기의 안전 정지 응답을 기다린 뒤 종료

## 6. 자동시험 현황

2026-09-30 문서 갱신 중 다음 명령을 실행했다.

```bash
./scripts/tests/run.sh
```

| 구간 | 관찰 결과 | 판정 |
| --- | --- | --- |
| `isaac_sim/tests` | `test_arm.py`가 오래된 `isaac_sim/controllers` 경로를 추가하여 `arm_mock` import 실패 | FAIL |
| manipulation pytest | 55 passed, 1 skipped | 부분 PASS |
| manipulation lint | flake8 271건, pep257 8건 | FAIL |
| manipulation ROS node 시험 | 제한된 작업 환경에서 `$HOME/.ros/log` 쓰기 불가로 10건 setup error | BLOCKED |
| system/perception/navigation package | 현재 runner가 실행하지 않음 | NOT_RUN |

위 결과는 라이브 PC A/B 전체 동작의 실패를 뜻하지 않는다. 다만 `scripts/tests/run.sh`가 “전부 통과”하는 상태는 아니므로 수정 전까지 자동시험 통과로 보고하면 안 된다.

## 7. 반복 결과 기록표

| 실행일 | Commit | run_id | job_id | 실행 번호 | 결과 | 실패 phase | 주요 측정값 | 증거 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| - | - | - | - | 1 | NOT_RUN | - | - | - |
| - | - | - | - | 2 | NOT_RUN | - | - | - |
| - | - | - | - | 3 | NOT_RUN | - | - | - |
| - | - | - | - | 4 | NOT_RUN | - | - | - |
| - | - | - | - | 5 | NOT_RUN | - | - | - |

## 8. 필수 측정값

| 구간 | 기록 값 |
| --- | --- |
| 준비 | READY까지 걸린 시간, 빠진 component |
| navigation | 최종 position/yaw error, retry count |
| perception | 관측점 수, 후보 수, confidence, 선택 slot pose |
| base alignment | 정렬 전후 slot x 오차 |
| manipulation | tray slot, phase별 시간, error code, placement_verified |
| 전체 | 작업 시작부터 HOME·READY까지 시간 |

## 9. 최종 통과 기준

- 미해결 안전 결함(P0)과 정상 플로우 차단 결함(P1)이 없다.
- 같은 commit과 초기 조건에서 한 권 사이클이 5회 연속 성공한다.
- 실패 입력에서는 다음 위험 단계로 진행하지 않는다.
- RViz로 map, 위치, scan, costmap과 계획 경로를 확인할 수 있다.
- 문서, action 정의, 실행 명령과 실제 코드가 일치한다.
- 자동시험의 FAIL/BLOCKED 항목을 해결하거나 최종 보고서에 제한으로 명시한다.
