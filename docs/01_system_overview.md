# 도서 자동 배치 시스템 개요

## 1. 프로젝트 목표

무인반납기에서 반환된 책 묶음을 트레이 단위로 인수하고, 책의 RFID·분류코드를 기준으로 목표 서가를 찾은 뒤 AMR과 로봇팔을 이용하여 빈 공간에 책을 배치한다.

1차 개발에서는 마커를 사용하지 않는다. YAML은 책의 분류코드에 맞는 목표 서가와 AMR 접근 위치를 제공하고, 서가에 도착한 이후의 정확한 빈 공간 위치는 RGB-D 비전이 직접 탐지한다.

## 2. 1차 구현 범위

- 무인반납기 동작을 ROS2 노드로 가상 구현
- RFID와 분류코드를 포함한 트레이 작업 생성
- YAML에서 목표 서가와 접근 waypoint 조회
- AMR의 무인반납기 및 서가 자율주행
- 서가 앞 카메라 관측 위치 정렬
- RGB-D 영상에서 서가 영역과 삽입 가능한 빈 공간 탐지
- 트레이 위 책의 위치와 자세 탐지
- 로봇팔의 책 파지, 사전 삽입, 저속 삽입, 해제와 후퇴
- 배치 완료 확인 및 YAML 상태 갱신
- AMR 주차 위치 복귀

## 3. 1차에서 제한하는 조건

초기 구현의 성공 가능성을 높이기 위해 다음 조건을 둔다.

- 한 종류의 책장 구조를 사용한다.
- 책장은 카메라 정면에 가깝게 배치한다.
- RGB-D 카메라를 사용한다.
- 책은 세워서 삽입한다.
- 기울어지거나 쓰러진 책은 자동 처리 대상에서 제외한다.
- 빈 공간의 폭이 대상 책 두께와 안전 여유보다 큰 경우에만 삽입한다.
- 여러 빈 공간이 있으면 정책에 따라 가장 넓거나 접근하기 쉬운 공간을 선택한다.
- 인식 신뢰도가 낮으면 한 번 재촬영하고, 계속 실패하면 작업을 중단한다.
- AMR 이동 중 로봇팔은 주행 안전 자세를 유지한다.
- 로봇팔 작업 중에는 AMR의 이동 명령을 잠근다.

## 4. 팀 역할

| 담당자 | 주 역할 | 담당 패키지 | 주요 책임 |
| --- | --- | --- | --- |
| A 이현민 | FSM·통합 | shelving_system, shelving_interfaces | 작업 순서, YAML, 무인반납기, 공통 통신, PC A/B 실행 |
| B 윤재민 | 비전 | shelving_perception | 서가·빈 공간 탐지, 트레이 책 탐지, 좌표 변환 |
| C 이동준 | AMR | shelving_navigation | Nav2, waypoint 이동, 서가·반납기 앞 정렬 |
| D 김도윤 | 로봇팔 | shelving_manipulation | 파지 계획, 책 삽입, 그리퍼와 안전 후퇴 |

## 5. 저장소 구조 원칙

이 프로젝트는 하나의 Git 저장소와 하나의 ROS2 workspace를 사용한다. PC별로 소스 폴더를 나누지 않는다. 모든 팀원이 같은 저장소를 받고, PC A와 PC B에서 서로 다른 launch 파일을 실행한다.

~~~text
book-shelving-system
└── ros2_ws
    └── src
        ├── shelving_interfaces
        ├── shelving_system
        ├── shelving_perception
        ├── shelving_navigation
        └── shelving_manipulation
~~~

패키지 분리는 별도 프로젝트를 의미하지 않는다. 각 패키지는 담당자가 독립적으로 개발할 수 있는 큰 기능 단위이며 전체 workspace는 colcon build 한 번으로 함께 빌드한다.

## 6. 패키지 책임

### shelving_interfaces

모든 패키지가 함께 사용하는 ROS2 메시지와 액션을 정의한다. 인터페이스 변경은 네 명이 합의하고 이현민이 반영한다.

### shelving_system

PC A의 핵심 패키지다. Task Manager, FSM, 작업 계획, YAML 데이터 관리, 무인반납기 가상 노드와 PC A/B launch 파일을 포함한다. 기존의 core, simulation, bringup 패키지를 하나로 합쳐 초보자가 실행 흐름을 한곳에서 확인할 수 있게 한다.

### shelving_perception

PC B에서 실행한다. 서가와 빈 공간을 함께 탐지하고, 트레이의 책 pose를 계산하며, 카메라 좌표를 로봇팔 기준 좌표로 변환한다.

### shelving_navigation

PC B에서 실행한다. Nav2 기반 자율주행과 무인반납기·서가 앞 작업 위치 정렬을 담당한다. 서가 슬롯의 정확한 좌표는 결정하지 않고 카메라가 관측하기 좋은 위치까지만 AMR을 이동시킨다.

### shelving_manipulation

PC B에서 실행한다. 트레이의 책을 파지하고, 비전이 제공한 빈 공간 pose로 접근하여 책을 삽입한 뒤 그리퍼를 해제하고 후퇴한다.

## 7. 컴퓨터별 실행 책임

| 컴퓨터 | 실행 내용 |
| --- | --- |
| PC A | Isaac Sim, shelving_system, Task Manager/FSM, YAML, 무인반납기 가상 노드 |
| PC B | shelving_perception, shelving_navigation, shelving_manipulation |
| 개인 PC | ROS2 상태·토픽·로그 확인 |
| 3차 관제 PC | 웹 작업 명령, 상태, 재고와 이력 조회 |

## 8. 단계별 확장

### 1차

YAML과 RGB-D 기반의 정상 플로우를 완성한다. 목표 서가까지는 YAML waypoint를 사용하고 정확한 빈 공간은 비전으로 탐지한다.

### 2차

PostgreSQL, 대체 슬롯 선택, 재시도·복구 FSM, 다양한 서가와 책 배치 조건을 추가한다. 필요할 때만 database 코드를 생성한다.

### 3차

Control API와 HTML 관제 화면을 추가한다. 작업 시작·중지, 현재 상태, 재고, 오류와 과거 작업 이력을 제공한다. 1차 저장소에는 아직 사용하지 않는 DB·대시보드 빈 폴더를 미리 만들지 않는다.

## 9. 개발 원칙

- 패키지 사이에서 다른 담당자의 Python 모듈을 직접 import하지 않는다.
- 패키지 간 데이터는 shelving_interfaces의 ROS2 메시지와 액션으로 전달한다.
- 파일 하나에는 이름으로 설명할 수 있는 하나의 책임만 둔다.
- 1차에서 실제로 사용하는 파일만 만든다.
- 설정값, 좌표와 제한값을 Python 코드에 하드코딩하지 않고 YAML에 둔다.
- 모든 작업 로그에 job_id와 book_id를 포함한다.
- 기능 구현보다 정상 플로우 연결과 반복 재현을 우선한다.

