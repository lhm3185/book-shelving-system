# 도서 자동 배치 시스템 개요

## 1. 프로젝트 목표

Isaac Sim의 무인반납기에서 반환된 트레이를 AMR이 인수하고, 분류코드에 맞는 서가로 자율주행한 뒤 RGB-D 비전과 Franka 로봇팔로 책을 빈 공간에 배치한다. 현재 시연 기준은 `book_001` 한 권의 전체 사이클이다.

목표 서가와 AMR 접근 경로는 YAML에서 읽고, 실제 빈 슬롯과 트레이 위 책은 손목 RGB-D 카메라로 판단한다. ArUco 마커는 사용하지 않는다.

## 2. 현재 구현 범위

- Isaac Sim 5.1 기반 도서관, Ridgeback-Franka, LiDAR, RGB-D 카메라 시뮬레이션
- 시나리오 `STOPPED/RESETTING/READY/RUNNING/FAILED` 상태와 Timeline 재시작 처리
- 가상 반납기의 `TrayJob` 생성과 실제 트레이 적재 `LoadTray` 액션
- 분류코드 기반 서가·경유점·책 프로파일 계획
- Nav2 State Lattice/MPPI 기반 경유점 주행, AMCL, 정밀 위치·yaw 정렬
- LiDAR PointCloud2를 `/scan`으로 변환하고 collision monitor를 거친 속도 명령 사용
- 팔 스윕 중 YOLO 및 depth 기반 빈 슬롯 탐지
- 선택 슬롯에 맞춘 AMR 횡정렬 후 트레이 책 재인식
- 트레이 슬롯 교정값을 이용한 파지 목표 보정
- Isaac 조작 실행기의 계획, 파지, 운반, 삽입, 해제, 후퇴, 배치 검증
- 작업 완료 후 서가에서 직접 후퇴하고 HOME으로 복귀
- PC A/B 준비 상태를 통합한 `/system/status`와 작업 시작 게이트
- PC C 웹 대시보드의 상태·위치·지도 표시 및 작업 시작
- PC B에서 Nav2 RViz 화면 자동 실행

## 3. 현재 시연 조건과 제한

- 기본 작업은 `book_001`, RFID `rfid_001`, 분류코드 `005.7` 한 권이다.
- 분류코드 `0`~`4`는 `shelf_01`, `5`~`9`는 `shelf_02`로 계획하지만, 현재 검증 좌표와 조작 범위는 `shelf_01` 중심이다.
- `shelf_map.yaml`의 `calibration_required`가 `true`이므로 좌표 변경 시 현장 재검증이 필요하다.
- 책은 트레이의 교정된 슬롯에 세워져 있고 서가에도 세워서 삽입한다고 가정한다.
- 조작 전체 자동 흐름은 `executor=sim`에서만 지원한다.
- 성공 기록은 현재 실행 중 메모리에만 유지한다. YAML 점유 상태 갱신과 DB 영속화는 구현되어 있지 않다.
- `shelf_02`, 다권 반복 성공률, 복구 후 자동 재개는 최종 검증 대상이다.

## 4. 팀 역할

| 담당자 | 주 역할 | 담당 패키지 | 주요 책임 |
| --- | --- | --- | --- |
| A 이현민 | FSM·통합 | `shelving_system`, `shelving_interfaces`, `shelving_web` | 작업 순서, 공통 계약, 시나리오/트레이 bridge, 준비 상태, 관제 통합 |
| B 윤재민 | 비전 | `shelving_perception` | 트레이 책·서가 빈 슬롯 검출, depth와 TF 기반 3D 결과 |
| C 이동준 | AMR | `shelving_navigation` | Nav2, 경유점 이동, 정밀 정렬, 서가 후퇴, PC B 실행 |
| D 김도윤 | 로봇팔 | `shelving_manipulation`, `isaac_sim/lib/controllers` | 선반 스윕, 파지 계획, 삽입 실행, 검증과 안전 정지 |

팀원 역할은 초기 계획과 동일하며, 공통 launch·인터페이스·Isaac 통합 파일은 관련 담당자가 함께 검토한다.

## 5. 저장소 구조

```text
book-shelving-system/
├── config/                 # PC 간 ROS 2 네트워크 설정
├── docs/                   # 프로젝트 문서
├── isaac_sim/              # 시뮬레이션, 센서·트레이·조작 실행기
├── ros2_ws/src/
│   ├── shelving_interfaces # 공통 msg/action
│   ├── shelving_system     # FSM, planner, bridge, supervisor
│   ├── shelving_perception # RGB-D/YOLO 인식
│   ├── shelving_navigation # Nav2와 정밀 정렬
│   ├── shelving_manipulation # PlaceBook 조정 계층
│   └── shelving_web        # PC C 웹 관제
├── scripts/                # 빌드·PC별 실행·비상 정지
└── tests/                  # 보조 데이터와 시험 자료
```

하나의 Git 저장소와 ROS 2 workspace를 사용한다. PC별 소스 복사본을 만들지 않고 같은 commit을 배포한 뒤 PC별 실행 스크립트만 다르게 사용한다.

## 6. 패키지 책임

### `shelving_interfaces`

6개 메시지(`TrayJob`, `RobotStatus`, `TargetSlot`, `GraspObservation`, `ScenarioState`, `SystemStatus`)와 5개 액션(`NavigateToTarget`, `LoadTray`, `DetectGraspPoint`, `DetectTargetSlot`, `PlaceBook`)을 정의한다.

### `shelving_system`

Task Manager FSM, YAML 작업 계획, 가상 반납기, Isaac 상태·트레이 bridge, 전체 준비 상태 감독과 PC A/B 통합 launch를 제공한다.

### `shelving_perception`

`/rgb`, `/depth`, `/camera_info`를 동기화하고 YOLO 모델과 depth를 이용해 트레이 책의 `GraspObservation`과 서가의 `TargetSlot`을 `arm_base_link` 기준으로 반환한다.

### `shelving_navigation`

map server, AMCL, pointcloud-to-laserscan, Nav2 서버와 `/navigate_to_target` 액션을 제공한다. 경유점이 있으면 `NavigateThroughPoses`, 없으면 `NavigateToPose`를 쓰며 마지막 구간은 저속 정밀 정렬한다.

### `shelving_manipulation`

빈 goal의 `/place_book` 요청 한 번으로 선반 스윕, 슬롯 선택, AMR 횡정렬, 책 인식, 목표 검증, Isaac 조작 명령과 배치 검증까지 조정한다.

### `shelving_web`

`/system/status`와 `/amcl_pose`를 HTTP/WebSocket으로 전달하고 `/system/start_cycle`을 호출하는 PC C 대시보드를 제공한다.

## 7. 컴퓨터별 실행 책임

| 컴퓨터 | 실행 내용 |
| --- | --- |
| PC A | Isaac Sim, scenario/tray/manipulation runtime, `simulation_bridge_node`, `task_manager_node`, `return_machine_node`, `system_supervisor_node` |
| PC B | Nav2/AMCL, perception, manipulation 조정 노드, TF 별칭, RViz |
| PC C | FastAPI 웹 게이트웨이와 운영자 대시보드 |

세 PC는 `ROS_DOMAIN_ID=130`, `rmw_fastrtps_cpp`와 동일한 유선 Fast DDS 설정을 사용한다.

## 8. 현재 정상 완료 조건

1. 시스템 상태가 `READY`다.
2. 작업 시작 요청이 수락되고 `TrayJob`이 한 번 발행된다.
3. AMR이 반납기에서 트레이를 인수하고 목표 서가에 도착한다.
4. 스윕·비전·정렬 후 유효한 슬롯과 책을 선택한다.
5. 조작 실행기가 책을 배치하고 `placement_verified=true`를 반환한다.
6. Task Manager가 완료 책을 메모리에 기록한다.
7. AMR이 서가에서 후퇴해 HOME으로 복귀하고 FSM과 시스템 상태가 다시 `IDLE/READY`가 된다.

## 9. 개발 원칙

- 패키지 간 계약은 `shelving_interfaces`를 사용한다.
- 좌표에는 frame과 timestamp를 유지하고 길이는 m, 각도는 rad를 사용한다.
- 시뮬레이터 raw JSON 토픽은 bridge 또는 manipulation 경계 안에서만 사용한다.
- 안전 검증을 통과하기 전에 로봇팔 명령을 실행하지 않는다.
- 실행 문서에는 구현된 기능과 계획 기능을 구분해서 적는다.
- 영속화되지 않는 값을 YAML/DB에 저장됐다고 표현하지 않는다.
