# 폴더·파일별 담당자와 협업 경계

## 1. 문서 목적

이 문서는 현재 저장소에 실제로 존재하는 폴더와 파일을 기준으로 주 담당자, 구현 범위, 검토자를 정한다. 주 담당자는 해당 파일의 설계와 정상 동작에 대한 책임자이며, 다른 팀원의 수정을 금지한다는 의미는 아니다. 다른 담당자가 수정할 때에는 주 담당자에게 변경 이유와 영향 범위를 알리고 Pull Request 검토를 받아야 한다.

## 2. 담당자 요약

| 구분 | 담당자 | 1차 주 기능 | 기본 소유 경로 |
| --- | --- | --- | --- |
| A | 이현민 | FSM, 시스템 통합, 공통 인터페이스, YAML, 무인반납기 | shelving_system, shelving_interfaces, 통합 문서·시험 |
| B | 윤재민 | RGB-D 비전, 서가·빈 공간·트레이 책 탐지, 좌표 변환 | shelving_perception |
| C | 이동준 | Nav2, waypoint 이동, 서가·반납기 앞 작업 위치 정렬 | shelving_navigation |
| D | 김도윤 | 책 파지 계획, 삽입, 그리퍼, 안전 후퇴 | shelving_manipulation |

## 3. 소유권 적용 규칙

- 주 담당자는 자신의 패키지에 필요한 Python, YAML, package.xml, setup.py와 시험 파일을 함께 관리한다.
- shelving_interfaces 변경은 이현민이 반영하지만 영향을 받는 담당자 전원의 검토가 필요하다.
- launch 파일은 이현민이 관리하고, 각 기능 담당자가 자신의 노드명·파라미터·실행 조건을 검증한다.
- simulation/library_system.usd는 통합 자산이므로 이현민이 병합한다. 개별 USD는 아래에 지정된 담당자가 관리한다.
- tests/integration과 정상 시나리오는 이현민이 취합하지만, 각 단계의 성공 조건은 기능 담당자가 작성한다.
- 한 Pull Request에서 unrelated package를 함께 수정하지 않는다.
- 다른 패키지의 내부 Python 모듈을 직접 import하지 않고 ROS2 메시지와 액션으로 연결한다.

### 3.1 각 Python 패키지의 공통 골격 파일

다음 파일은 shelving_system, shelving_perception, shelving_navigation, shelving_manipulation에 공통으로 존재하며 각 패키지의 주 담당자가 함께 관리한다.

| 파일 | 책임 |
| --- | --- |
| 패키지명/__init__.py | Python package 초기화 파일. 외부에 공개할 API가 없으면 비워 둔다. |
| LICENSE | Apache-2.0 라이선스 원문. 팀 합의 없이 수정하지 않는다. |
| test/test_copyright.py | 소스의 copyright 표기 검사 |
| test/test_flake8.py | Python style과 정적 검사 |
| test/test_pep257.py | docstring 규칙 검사 |

새 단위시험은 같은 test 폴더에 test_기능명.py 형식으로 추가하고 해당 기능 담당자가 소유한다.

## 4. A 이현민 — FSM·시스템 통합

### 4.1 shelving_interfaces

| 파일 | 책임 |
| --- | --- |
| ros2_ws/src/shelving_interfaces/msg/TrayJob.msg | 무인반납기에서 시스템으로 전달되는 job, tray, 책, RFID와 분류코드 정의 |
| ros2_ws/src/shelving_interfaces/msg/RobotStatus.msg | PC B 기능의 상태, 진행률, heartbeat와 오류 형식 정의 |
| ros2_ws/src/shelving_interfaces/msg/TargetSlot.msg | 비전이 반환하는 빈 공간 pose, 크기, 삽입 깊이와 신뢰도 정의 |
| ros2_ws/src/shelving_interfaces/action/NavigateToTarget.action | AMR 목표 이동·정렬 goal, feedback와 result 계약 |
| ros2_ws/src/shelving_interfaces/action/DetectTargetSlot.action | 책 크기에 맞는 빈 공간 탐지 요청과 결과 계약 |
| ros2_ws/src/shelving_interfaces/action/PlaceBook.action | 책 파지부터 배치 확인까지의 요청과 결과 계약 |
| ros2_ws/src/shelving_interfaces/CMakeLists.txt | msg/action 생성 목록과 rosidl 의존성 유지 |
| ros2_ws/src/shelving_interfaces/package.xml | 공통 인터페이스 의존성과 package metadata 유지 |
| ros2_ws/src/shelving_interfaces/LICENSE | 라이선스 유지. 임의 변경 금지 |

인터페이스 필드 변경 시 윤재민은 DetectTargetSlot과 TargetSlot, 이동준은 NavigateToTarget, 김도윤은 PlaceBook을 반드시 검토한다. 필드명·단위·frame 변경은 네 명이 합의한 뒤 반영한다.

### 4.2 shelving_system

| 파일 | 책임 |
| --- | --- |
| shelving_system/task_manager_node.py | 전체 작업의 ROS2 진입점, 액션 client 호출 순서, 상태와 결과 취합 |
| shelving_system/state_machine.py | INITIALIZING부터 IDLE, 이동, 인식, 삽입, 완료·실패까지 상태와 전이 |
| shelving_system/job_planner.py | 분류코드로 목표 서가·waypoint 선택, 책 처리 순서 생성 |
| shelving_system/yaml_data_manager.py | YAML 읽기·검증·갱신, 성공 후 점유 상태 기록 |
| shelving_system/return_machine_node.py | RFID·분류코드와 TrayJob을 발행하는 무인반납기 가상 노드 |
| config/system.yaml | timeout, 재시도 횟수, 상태 전이 관련 공통 설정 |
| config/shelf_map.yaml | 서가 ID, 분류코드, map 기준 접근 waypoint와 관측 조건 |
| config/book_profiles.yaml | 시스템이 작업 계획에 사용하는 책 크기·프로파일 |
| launch/pc_a.launch.py | PC A에서 system 노드와 필요한 설정 실행 |
| launch/pc_b.launch.py | PC B의 perception, navigation, manipulation 노드 실행 |
| launch/all.launch.py | 한 PC 통합 개발용 전체 노드 실행 |
| setup.py | launch·config 설치, console_scripts 등록 |
| setup.cfg | ROS2 Python 실행 파일 설치 경로 |
| package.xml | rclpy, interfaces와 추가 런타임 의존성 |
| resource/shelving_system | ament package marker |
| test/* | lint와 추후 system 단위시험 |

launch 파일 수정 시 pc_b.launch.py는 윤재민·이동준·김도윤이 자신의 노드와 파라미터가 정확한지 검토한다.

### 4.3 통합 파일

| 파일·폴더 | 이현민의 책임 | 협업 |
| --- | --- | --- |
| simulation/library_system.usd | 개별 자산 reference, prim 경로, 전체 stage 통합 | 모든 자산 담당자가 검증 |
| simulation/assets/return_machine.usd | 무인반납기 위치, 트레이 인계 영역 | 김도윤이 파지 접근성 검토 |
| scripts/build.sh | 전체 workspace 빌드 진입점 | 전원 사용성 확인 |
| scripts/run_all.sh | 한 PC 통합 실행 | 전원 노드 실행 확인 |
| scripts/run_pc_a.sh | PC A 실행 | 이현민 검증 |
| scripts/run_tests.sh | 전체 시험 실행과 결과 출력 | 전원 자기 시험 확인 |
| tests/integration/test_normal_flow.py | 정상 플로우 통합시험 취합 | CP별 담당자가 assertion 제공 |
| tests/scenarios/normal_flow.yaml | 정상 시나리오 입력과 초기 상태 | 전원 초기 조건 검토 |
| README.md | 프로젝트 진입점과 문서 목차 | 문서 변경 시 링크 검증 |
| .gitignore, .gitattributes | 생성물 제외와 Git LFS 대상 관리 | LFS 대상 추가 시 전원 공지 |
| .env.example | 환경변수 이름의 예시만 제공 | 실제 비밀번호·토큰 금지 |

## 5. B 윤재민 — 비전

### 5.1 shelving_perception

| 파일 | 책임 |
| --- | --- |
| shelving_perception/perception_node.py | RGB-D 입력 구독, DetectTargetSlot action server, 처리 단계 조정과 RobotStatus 발행 |
| shelving_perception/shelf_slot_detector.py | 서가 ROI, 선반 내부, 빈 공간 후보와 사용 가능한 폭·높이·깊이 계산 |
| shelving_perception/tray_book_detector.py | 트레이 위 책의 위치·자세와 검출 신뢰도 계산 |
| shelving_perception/coordinate_transformer.py | camera_link의 책·빈 공간 pose를 arm_base_link 등 요청 frame으로 변환 |
| config/camera.yaml | RGB/depth topic, intrinsics, 유효 거리, frame 이름 |
| config/perception.yaml | ROI, 최소 공간 크기, safety margin, confidence와 재촬영 기준 |
| setup.py | config 설치와 perception_node console_script 등록 |
| setup.cfg | Python 실행 파일 설치 경로 |
| package.xml | image, depth, OpenCV, TF 등 실제 사용 의존성 |
| resource/shelving_perception | ament package marker |
| test/* | lint와 비전 단위시험 |

### 5.2 비전 관련 시뮬레이션 자산

| 파일 | 책임 |
| --- | --- |
| simulation/assets/bookshelf.usd | 서가 크기·칸 구조가 detector의 가정과 일치하도록 관리 |
| simulation/assets/library_background.usd | 조명·배경·서가 주변 환경이 RGB-D 시험 조건에 맞도록 관리 |

mobile_manipulator.usd의 카메라 prim, optical frame과 장착 pose를 변경할 때에는 이동준과 함께 검토한다. TargetSlot 좌표계를 변경할 때에는 이현민·김도윤의 승인이 필요하다.

## 6. C 이동준 — AMR

### 6.1 shelving_navigation

| 파일 | 책임 |
| --- | --- |
| shelving_navigation/navigation_node.py | NavigateToTarget action server, Nav2 goal 전송, feedback·결과와 RobotStatus 발행 |
| shelving_navigation/docking_controller.py | 무인반납기와 서가 관측 위치의 position/yaw 오차 계산과 미세 정렬 |
| config/nav2.yaml | planner, controller, behavior, costmap과 localization 설정 |
| config/navigation.yaml | timeout, 허용오차, 재시도와 안전정지 정책 |
| config/waypoints.yaml | HOME, RETURN_STATION, SHELF 관측 waypoint |
| maps/library_map.yaml | map 해상도, origin과 이미지 연결 |
| maps/library_map.pgm | 도서관 occupancy map 이미지 |
| setup.py | config·maps 설치와 navigation_node console_script 등록 |
| setup.cfg | Python 실행 파일 설치 경로 |
| package.xml | Nav2, TF, geometry와 실제 사용 의존성 |
| resource/shelving_navigation | ament package marker |
| test/* | lint와 이동·정렬 단위시험 |

### 6.2 AMR 관련 공통 파일

| 파일 | 책임 |
| --- | --- |
| simulation/assets/mobile_manipulator.usd | AMR base, articulation, 센서·팔 장착 기준과 이동 충돌체의 주 관리 |
| scripts/run_pc_b.sh | PC B 실행 환경, source와 pc_b launch 호출의 주 관리 |

mobile_manipulator.usd에서 로봇팔 관절·그리퍼 충돌체는 김도윤이, 카메라 frame은 윤재민이 함께 검토한다. waypoint를 변경하면 shelf_map.yaml과 정상 시나리오의 목표가 일치하는지 이현민과 확인한다.

## 7. D 김도윤 — 로봇팔·그리퍼

### 7.1 shelving_manipulation

| 파일 | 책임 |
| --- | --- |
| shelving_manipulation/manipulation_node.py | PlaceBook action server, 파지·삽입 단계 조정, 취소와 RobotStatus 발행 |
| shelving_manipulation/grasp_planner.py | 책 pose와 치수로 접근·파지 pose, 그리퍼 폭과 안전 여유 생성 |
| shelving_manipulation/book_placer.py | 사전 삽입, 정렬, 저속 삽입, 해제, 후퇴와 배치 확인 |
| config/manipulation.yaml | 속도, 가속도, timeout, 파지·삽입 재시도와 안전 후퇴 |
| config/moveit.yaml | planning group, planner, 충돌 검사와 joint 제한 |
| config/book_profiles.yaml | manipulation이 사용하는 책별 치수, 파지점과 삽입 파라미터 |
| setup.py | config 설치와 manipulation_node console_script 등록 |
| setup.cfg | Python 실행 파일 설치 경로 |
| package.xml | MoveIt, TF, control과 실제 사용 의존성 |
| resource/shelving_manipulation | ament package marker |
| test/* | lint와 파지·삽입 단위시험 |

### 7.2 조작 관련 시뮬레이션 자산

| 파일 | 책임 |
| --- | --- |
| simulation/assets/book.usd | 책 크기, 질량, 마찰과 collision |
| simulation/assets/tray.usd | 책 대기 위치, 파지 접근 공간과 collision |

return_machine.usd의 트레이 인계 위치는 이현민과 함께 정하고, mobile_manipulator.usd의 팔·그리퍼 articulation과 collision은 이동준과 함께 검토한다.

## 8. 파일 충돌 가능성이 큰 공통 영역

| 영역 | 변경 책임 | 필수 검토 |
| --- | --- | --- |
| shelving_interfaces/msg, action | 이현민 반영 | 영향을 받는 기능 담당자 전원 |
| shelving_system/launch | 이현민 반영 | 실행되는 노드 담당자 |
| simulation/library_system.usd | 이현민 병합 | 변경된 asset 담당자 |
| mobile_manipulator.usd | 이동준 병합 | 윤재민·김도윤 |
| tests/integration | 이현민 병합 | 관련 CP 담당자 |
| docs/05_test_plan_and_results.md | 이현민 형식 관리 | 각 담당자가 자기 측정값 입력 |
| scripts/run_pc_b.sh | 이동준 병합 | 윤재민·김도윤 |

공통 파일을 동시에 수정해야 하면 작업 시작 전에 담당자를 정하고 한 명만 최종 병합한다. USD는 텍스트 충돌 해결이 어렵기 때문에 같은 파일을 두 명이 동시에 수정하지 않는다.

## 9. 문서 담당

| 문서 | 주 담당 | 검토 |
| --- | --- | --- |
| 01_system_overview.md | 이현민 | 전원 |
| 02_architecture_and_flow.md | 이현민 | 전원 |
| 03_ros_interfaces.md | 이현민 | 인터페이스별 담당자 |
| 04_development_schedule.md | 이현민 | 전원 일정 확인 |
| 05_test_plan_and_results.md | 이현민 취합 | 각자 자기 기능 결과 기록 |
| 06_runbook.md | 이현민 | 이동준 PC 실행 검토, 전원 절차 검증 |
| 07_file_ownership.md | 이현민 | 전원 소유권 확인 |
| 08_git_and_terminal_guide.md | 이현민 | 전원 실제 명령 검증 |

## 10. Pull Request 검토 기준

### 이현민에게 요청할 검토

FSM 상태, job_id 흐름, YAML 구조, 공통 인터페이스, launch, 통합시험을 변경한 경우 요청한다.

### 윤재민에게 요청할 검토

RGB-D topic, camera frame, TargetSlot 계산, 서가·트레이 인식과 confidence 기준을 변경한 경우 요청한다.

### 이동준에게 요청할 검토

map, waypoint, Nav2, AMR base, 도킹 허용오차와 PC B 실행을 변경한 경우 요청한다.

### 김도윤에게 요청할 검토

책 치수, gripper TCP, MoveIt, 파지·삽입 경로, 속도와 collision을 변경한 경우 요청한다.

## 11. 완료의 정의

담당 파일에 코드를 작성했다는 사실만으로 완료하지 않는다. 다음 조건을 모두 만족해야 한다.

1. 자신의 패키지가 colcon build에 성공한다.
2. 정상 입력과 대표 실패 입력에 대한 시험이 있다.
3. error_code와 RobotStatus가 인터페이스 명세와 일치한다.
4. 설정값을 코드에 하드코딩하지 않고 YAML에서 관리한다.
5. 관련 문서와 실행 방법을 갱신한다.
6. 다른 담당자가 Pull Request를 검토한다.
7. 통합 브랜치에서 최소 정상 플로우를 깨뜨리지 않는다.


---

# 팀원별 담당 폴더 및 파일 구조

## A 이현민 — FSM·시스템 통합

```text
book-shelving-system/
├── README.md
├── .gitignore
├── .gitattributes
├── .env.example
│
├── ros2_ws/
│   └── src/
│       ├── shelving_interfaces/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── LICENSE
│       │   │
│       │   ├── msg/
│       │   │   ├── TrayJob.msg
│       │   │   ├── RobotStatus.msg
│       │   │   └── TargetSlot.msg
│       │   │
│       │   └── action/
│       │       ├── NavigateToTarget.action
│       │       ├── DetectTargetSlot.action
│       │       └── PlaceBook.action
│       │
│       └── shelving_system/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── LICENSE
│           │
│           ├── resource/
│           │   └── shelving_system
│           │
│           ├── shelving_system/
│           │   ├── __init__.py
│           │   ├── task_manager_node.py
│           │   ├── state_machine.py
│           │   ├── job_planner.py
│           │   ├── yaml_data_manager.py
│           │   └── return_machine_node.py
│           │
│           ├── config/
│           │   ├── system.yaml
│           │   ├── shelf_map.yaml
│           │   └── book_profiles.yaml
│           │
│           ├── launch/
│           │   ├── pc_a.launch.py
│           │   ├── pc_b.launch.py
│           │   └── all.launch.py
│           │
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py
│
├── simulation/
│   ├── library_system.usd
│   └── assets/
│       └── return_machine.usd
│
├── scripts/
│   ├── build.sh
│   ├── run_all.sh
│   ├── run_pc_a.sh
│   └── run_tests.sh
│
├── tests/
│   ├── integration/
│   │   └── test_normal_flow.py
│   └── scenarios/
│       └── normal_flow.yaml
│
└── docs/
    ├── 01_system_overview.md
    ├── 02_architecture_and_flow.md
    ├── 03_ros_interfaces.md
    ├── 04_development_schedule.md
    ├── 05_test_plan_and_results.md
    ├── 06_runbook.md
    ├── 07_file_ownership.md
    └── 08_git_and_terminal_guide.md
```

---

## B 윤재민 — 비전

```text
book-shelving-system/
├── ros2_ws/
│   └── src/
│       └── shelving_perception/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── LICENSE
│           │
│           ├── resource/
│           │   └── shelving_perception
│           │
│           ├── shelving_perception/
│           │   ├── __init__.py
│           │   ├── perception_node.py
│           │   ├── shelf_slot_detector.py
│           │   ├── tray_book_detector.py
│           │   └── coordinate_transformer.py
│           │
│           ├── config/
│           │   ├── camera.yaml
│           │   └── perception.yaml
│           │
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py
│
└── simulation/
    └── assets/
        ├── bookshelf.usd
        └── library_background.usd
```

---

## C 이동준 — AMR

```text
book-shelving-system/
├── ros2_ws/
│   └── src/
│       └── shelving_navigation/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── LICENSE
│           │
│           ├── resource/
│           │   └── shelving_navigation
│           │
│           ├── shelving_navigation/
│           │   ├── __init__.py
│           │   ├── navigation_node.py
│           │   └── docking_controller.py
│           │
│           ├── config/
│           │   ├── nav2.yaml
│           │   ├── navigation.yaml
│           │   └── waypoints.yaml
│           │
│           ├── maps/
│           │   ├── library_map.yaml
│           │   └── library_map.pgm
│           │
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py
│
├── simulation/
│   └── assets/
│       └── mobile_manipulator.usd
│
└── scripts/
    └── run_pc_b.sh
```

---

## D 김도윤 — 로봇팔·그리퍼

```text
book-shelving-system/
├── ros2_ws/
│   └── src/
│       └── shelving_manipulation/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── LICENSE
│           │
│           ├── resource/
│           │   └── shelving_manipulation
│           │
│           ├── shelving_manipulation/
│           │   ├── __init__.py
│           │   ├── manipulation_node.py
│           │   ├── grasp_planner.py
│           │   └── book_placer.py
│           │
│           ├── config/
│           │   ├── manipulation.yaml
│           │   ├── moveit.yaml
│           │   └── book_profiles.yaml
│           │
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py
│
└── simulation/
    └── assets/
        ├── book.usd
        └── tray.usd
```
