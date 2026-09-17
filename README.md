# Book Shelving System

무인반납기에서 반환된 책을 AMR과 로봇팔이 서가까지 운반하고, RGB-D 비전으로 삽입 가능한 빈 공간을 찾아 자동 배치하는 ROS2 프로젝트입니다.

## 현재 구현 상태

- ROS2 Jazzy 기준 5개 패키지 골격 생성 완료
- 공통 메시지 3개와 액션 3개 정의
- 전체 workspace 빌드 확인
- Python 노드, launch, YAML과 실행 스크립트의 실제 동작은 담당자별 구현 필요
- 1차 개발에서는 ArUco 마커를 사용하지 않음
- YAML은 목표 서가와 AMR 접근 위치를 제공하고, 정확한 빈 공간은 RGB-D 비전이 탐지

## 패키지

| 패키지 | 기능 | 주 담당 |
| --- | --- | --- |
| shelving_interfaces | 공통 메시지·액션 계약 | 이현민, 변경 시 전원 검토 |
| shelving_system | FSM, 작업 계획, YAML, 무인반납기, launch | 이현민 |
| shelving_perception | 서가·빈 공간·트레이 책 탐지와 좌표 변환 | 윤재민 |
| shelving_navigation | Nav2 이동과 작업 위치 정렬 | 이동준 |
| shelving_manipulation | 책 파지, 삽입, 그리퍼와 안전 후퇴 | 김도윤 |

파일 단위 소유권과 협업 경계는 [docs/07_file_ownership.md](docs/07_file_ownership.md)를 기준으로 합니다.

## 처음 참여할 때

~~~bash
cd /home/hymi
git clone https://github.com/lhm3185/book-shelving-system.git
cd book-shelving-system
git lfs install
git lfs pull
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
~~~

이미 저장소를 받은 팀원은 새 브랜치를 만들기 전에 main을 최신화합니다.

~~~bash
cd /home/hymi/book-shelving-system
git switch main
git pull --ff-only origin main
~~~

전체 Git 작업 절차, 브랜치명, 커밋 규칙과 PC별 실행 명령은 [docs/08_git_and_terminal_guide.md](docs/08_git_and_terminal_guide.md)에 정리되어 있습니다.

## 문서

| 문서 | 내용 |
| --- | --- |
| [01_system_overview.md](docs/01_system_overview.md) | 목표, 범위, 패키지와 PC 역할 |
| [02_architecture_and_flow.md](docs/02_architecture_and_flow.md) | 시스템 아키텍처, 정상 플로우, CP |
| [03_ros_interfaces.md](docs/03_ros_interfaces.md) | 메시지와 액션 계약 |
| [04_development_schedule.md](docs/04_development_schedule.md) | 날짜별 개발·통합 일정 |
| [05_test_plan_and_results.md](docs/05_test_plan_and_results.md) | 시험 항목과 결과 기록 |
| [06_runbook.md](docs/06_runbook.md) | 시연 실행과 장애 대응 |
| [07_file_ownership.md](docs/07_file_ownership.md) | 담당자별 폴더·파일과 검토 책임 |
| [08_git_and_terminal_guide.md](docs/08_git_and_terminal_guide.md) | Git 규칙과 터미널 명령 |

## 중요한 원칙

- main 브랜치에서 직접 개발하지 않습니다.
- 다른 담당자의 패키지 내부 Python 파일을 직접 import하지 않습니다.
- 패키지 간 통신은 shelving_interfaces의 메시지와 액션을 사용합니다.
- build, install, log와 .env는 커밋하지 않습니다.
- USD, PGM 등 LFS 대상 파일을 변경하기 전에 Git LFS가 정상인지 확인합니다.
- 공통 인터페이스, launch, 통합 USD 변경은 관련 담당자의 검토를 받습니다.
