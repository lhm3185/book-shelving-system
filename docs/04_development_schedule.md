# 개발 일정 및 역할 분담

## 1. 팀 역할

| 담당자 | 주 기능 | 담당 경로 |
| --- | --- | --- |
| A 이현민 | FSM·통합 | `shelving_system`, `shelving_interfaces`, `shelving_web` |
| B 윤재민 | 비전 | `shelving_perception` |
| C 이동준 | AMR | `shelving_navigation`, PC B 주행 실행 |
| D 김도윤 | 로봇팔 | `shelving_manipulation`, `isaac_sim/lib/controllers` |

역할은 초기 계획에서 변경하지 않는다. 실제 파일별 경계는 [07_file_ownership.md](07_file_ownership.md)를 따른다.

## 2. 개발 결과 기준 마일스톤

| 단계 | 구현 결과 | 현재 상태 |
| --- | --- | --- |
| 구조·공통 계약 | 6 msg, 5 action, 6 ROS 패키지 | 구현됨 |
| 시뮬레이션 기반 | library scene, Ridgeback-Franka, 센서, scenario reset | 구현됨 |
| AMR 통합 | AMCL, Nav2, 경유점, 정밀 정렬, shelf retreat | 구현됨 |
| 트레이 인수 | `LoadTray` bridge와 실제 tray runtime | 구현됨 |
| 비전 통합 | YOLO+depth의 책/슬롯 action server | 구현됨 |
| 조작 통합 | 팔 스윕, 슬롯 선택, base 정렬, 파지·삽입·검증 | 구현됨 |
| 3-PC 운영 | PC별 one-command 실행, supervisor, 웹 관제 | 구현됨 |
| 시각화 | PC B Nav2 RViz 자동 실행 | 구현됨 |
| 영속 저장 | YAML 점유 변경, DB 재고·이력 | 미구현 |
| 최종 회귀 | 동일 초기 조건 반복 성공률과 증거 | 추가 기록 필요 |

## 3. 초기 계획에서 바뀐 점

- 단순 mock 트레이 확인 대신 실제 트레이 이동을 `LoadTray` 액션으로 연결했다.
- Task Manager가 perception을 직접 순차 호출하지 않고 `PlaceBook`이 스윕·인식·정렬·조작을 조정한다.
- 서가 도착 뒤 팔 스윕으로 여러 관측점을 만들고 선택 슬롯에 맞춰 AMR을 횡정렬한다.
- Timeline reset, AMCL 초기 pose 재설정과 잔류 속도 제거가 추가됐다.
- PC C 웹 관제와 `SystemStatus`, `/system/start_cycle` 준비 게이트가 추가됐다.
- 조작은 MoveIt launch가 아니라 Isaac 내부 kinematics/path controller를 사용한다.
- 완료 결과는 메모리에만 기록하며 DB/YAML 점유 저장은 현재 범위에서 제외됐다.

## 4. 최종화 작업

| 작업 | A 이현민 | B 윤재민 | C 이동준 | D 김도윤 | 공통 산출물 |
| --- | --- | --- | --- | --- | --- |
| 통합 확인 | FSM, supervisor, PC A/C | action 결과·좌표 | 주행·정렬·복귀 | 스윕·배치 실행기 | 한 권 end-to-end |
| 시연 점검 | READY/start gate와 reset | RGB-D/YOLO·confidence | map/AMCL/costmap/RViz | 파지·삽입·검증 | 동일 commit 배포 |
| 반복 시험 | job·상태 로그 | 후보와 검출 실패 | 위치·yaw 오차 | phase·오류 코드 | 5회 결과표 |
| 발표 | 전체 흐름 | 비전 설명 | 자율주행·RViz | 로봇팔 안전 | 영상·로그 백업 |

## 5. 남은 우선순위

1. 실제 PC A/B/C에서 한 권 정상 사이클 반복 결과를 기록한다.
2. `shelf_01` 좌표와 0.65 m 후퇴 경로를 최종 고정한다.
3. RGB-D, TF, tray slot과 shelf plane 교정값을 동결한다.
4. 자동시험 runner의 경로 문제와 lint 실패를 정리한다.
5. 필요하면 후속 단계에서 YAML/DB 영속화를 설계한다.

## 6. 변경 통제

- 공통 인터페이스, launch, scene 설정은 관련 담당자가 함께 검토한다.
- 발표용 commit에서는 좌표, 속도, 모델과 네트워크 설정을 동시에 변경하지 않는다.
- P0/P1 수정 외 신규 기능은 반복 성공 결과를 깨뜨리지 않는 경우에만 반영한다.
- 문서의 상태는 `구현됨`, `검증 필요`, `미구현`을 구분한다.
- 각 시험은 commit, `run_id`, `job_id`, 결과와 증거를 남긴다.
