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

lfs 설치
~~~bash
sudo apt update && sudo apt install git-lfs
git lfs install
~~~

rosdep 초기화
~~~bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
~~~

nav2 관련 패키지 설치
~~~bash
sudo apt update

sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-pointcloud-to-laserscan
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


### 설치 및 실행

## 검증 환경

현재 통합 사이클은 다음 환경에서 검증했다.

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- Isaac Sim 5.1
- NVIDIA RTX GPU
- `rmw_fastrtps_cpp`
- 검증 브랜치: `integration/manipulation-standalone`
- 검증 커밋: `0f0adff`

현재 검증 범위는 **책 1권 전체 통합 사이클**이다.

---

## 1. 저장소 받기

Git LFS를 먼저 설치한다.

```bash
sudo apt update
sudo apt install -y git git-lfs

git lfs install
```

현재 검증 브랜치를 직접 clone한다.

```bash
cd ~
git clone \
  --branch integration/manipulation-standalone \
  --single-branch \
  https://github.com/lhm3185/book-shelving-system.git

cd ~/book-shelving-system
git lfs pull
```

커밋을 확인한다.

```bash
git branch --show-current
git rev-parse --short HEAD
git status --short
```

검증 버전이라면 다음과 같이 표시된다.

```text
integration/manipulation-standalone
0f0adff
```

`git status --short`에는 아무것도 출력되지 않아야 한다.

---

## 2. 필수 프로그램

다음 프로그램이 필요하다.

- Ubuntu 24.04
- ROS 2 Jazzy
- Isaac Sim 5.1
- NVIDIA GPU Driver
- Git LFS
- Python 3.12
- colcon
- rosdep

ROS 2 Jazzy 설치가 끝난 후 다음 패키지를 설치한다.

```bash
sudo apt update

sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-venv \
  python3-pip \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-message-filters
```

rosdep을 초기화한다. 이미 초기화돼 있다면 첫 번째 명령의 오류는 무시해도 된다.

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

프로젝트의 ROS 의존성을 설치한다.

```bash
cd ~/book-shelving-system

source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths ros2_ws/src \
  --ignore-src \
  -r \
  -y
```

---

## 3. Isaac Sim 경로 설정

기본 Isaac Sim 설치 경로는 다음과 같다.

```text
~/isaacsim
```

다음 파일이 존재하는지 확인한다.

```bash
test -x "$HOME/isaacsim/python.sh" \
  && echo "Isaac Sim 확인 완료" \
  || echo "Isaac Sim 경로를 확인하십시오"
```

Isaac Sim이 다른 위치에 있다면 실행할 터미널에서 경로를 설정한다.

```bash
export ISAAC_SIM_PATH=/absolute/path/to/isaacsim
```

예:

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
```

---

## 4. Perception Python 환경 구성

통합 실행 스크립트는 프로젝트 루트의 `.venv`를 사용한다.

```bash
cd ~/book-shelving-system

python3 -m venv .venv

./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install \
  "numpy==1.26.4" \
  "ultralytics==8.4.155"
```

설치 결과를 확인한다.

```bash
./.venv/bin/python - <<'PY'
import numpy
import torch
import ultralytics

print("numpy:", numpy.__version__)
print("torch:", torch.__version__)
print("ultralytics:", ultralytics.__version__)
PY
```

YOLO 모델이 Git LFS로 정상 다운로드됐는지 확인한다.

```bash
ls -lh \
  ros2_ws/src/shelving_perception/resource/best.pt \
  ros2_ws/src/shelving_perception/resource/book_tray_best.pt
```

각 파일이 수 MB 크기여야 한다. 파일 크기가 몇백 바이트라면 LFS 포인터만 받은 것이므로 다시 실행한다.

```bash
git lfs pull
```

---

## 5. ROS workspace 빌드

```bash
cd ~/book-shelving-system/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

빌드 완료 후 환경을 적용한다.

```bash
source ~/book-shelving-system/ros2_ws/install/setup.bash
```

핵심 패키지를 확인한다.

```bash
ros2 pkg executables shelving_system
ros2 pkg executables shelving_navigation
ros2 pkg executables shelving_perception
ros2 pkg executables shelving_manipulation
```

---

## 6. 기본 테스트

Franka kinematics 테스트를 실행한다.

```bash
cd ~/book-shelving-system

python3 -m pytest \
  isaac_sim/tests/test_arm_kinematics.py \
  -q
```

검증된 결과:

```text
12 passed
```

---

## 7. TeamViewer 또는 SSH에서 Isaac Sim 창 표시하기

원격 SSH 터미널에서 실행하면서 Isaac Sim 창을 TeamViewer의 데스크톱 화면에 표시하려면 다음 환경 변수를 설정한다.

```bash
export DISPLAY=:0
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export XAUTHORITY="$XDG_RUNTIME_DIR/gdm/Xauthority"
```

파일 접근 가능 여부를 확인한다.

```bash
test -r "$XAUTHORITY" \
  && echo "GUI 권한 확인 완료" \
  || echo "XAUTHORITY 경로를 확인하십시오: $XAUTHORITY"
```

현재 검증 PC의 사용자가 UID 1000이라면 실제 경로는 다음과 같다.

```text
/run/user/1000/gdm/Xauthority
```

물리 데스크톱에서 직접 터미널을 연 경우에는 위 환경 변수가 이미 설정돼 있을 수 있다.

---

## 8. 전체 통합 실행

프로젝트 루트에서 실행한다.

```bash
cd ~/book-shelving-system

export DISPLAY=:0
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export XAUTHORITY="$XDG_RUNTIME_DIR/gdm/Xauthority"

export ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
export ROS_DOMAIN_ID=130
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

./scripts/run.sh full \
  2>&1 | tee /tmp/full-ridgeback-one-book.log
```

`full` 모드는 다음 구성요소를 자동으로 실행한다.

- Isaac Sim
- Ridgeback-Franka 시뮬레이션
- Nav2
- Navigation Node
- Simulation Bridge
- Task Manager
- Return Machine
- Perception
- Manipulation
- 트레이 적재 Action
- RGB/Depth 카메라
- LiDAR 및 costmap

모든 구성요소가 준비되면 스크립트가 자동으로 책 1권 작업을 발행한다.

```text
모든 구성요소가 준비됐습니다.
반납기 테스트 작업을 발행합니다.
테스트 작업을 발행했습니다.
```

> 실행 모드 이름은 `full`이다.  
> `full-ridgeback-one-book`은 지원되는 실행 모드가 아니다.

---

## 9. Navigation만 테스트하기

실제 Perception과 Manipulation 대신 Mock Manipulation을 사용한다.

```bash
cd ~/book-shelving-system

export DISPLAY=:0
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export XAUTHORITY="$XDG_RUNTIME_DIR/gdm/Xauthority"

./scripts/run.sh navigation-test \
  2>&1 | tee /tmp/navigation-test.log
```

---

## 10. 실행 로그 확인

전체 실행 중 핵심 상태를 확인한다.

```bash
tail -n 300 -F /tmp/full-ridgeback-one-book.log \
  | grep --line-buffered -aE \
'Published TrayJob|Job plan created|PlaceBook 성공|PlaceBook 실패|Book placement succeeded|Shelf retreat completed|RETURN_HOME -> COMPLETED|COMPLETED -> IDLE|\[(ERROR|FATAL)\]|Traceback|Isaac Sim 프로세스가 종료'
```

Manipulation 세부 로그는 다음 명령으로 확인한다.

```bash
tail -n 300 -F /tmp/full-ridgeback-one-book.log \
  | grep --line-buffered -aE \
'비전 관측을 실제 트레이 슬롯으로 교정|슬롯 후보|R&D 빈칸 선택|계획 OK|\[놓음\]|\[밀기완료\]|\[밀기후개방\]|\[후퇴후\]|배치 확인|IK_FAILED|GRASP_FAILED|BOOK_DROPPED|MOTION_TIMEOUT|TARGET_INVALID|PLACEMENT_NOT_VERIFIED'
```

---

## 11. 성공 판정

전체 사이클 성공 시 다음 로그가 출력돼야 한다.

```text
PlaceBook 성공
Book placement succeeded
Shelf retreat completed. Sending the home navigation goal.
FSM transition completed: RETURN_HOME -> COMPLETED
FSM transition completed: COMPLETED -> IDLE
```

배치 검증 결과는 다음 다섯 항목이 모두 `true`여야 한다.

```text
upright: true
depth: true
spine: true
x: true
floor: true
```

Nav2 동작 전환 과정에서 다음 메시지가 출력될 수 있다.

```text
Failed to get result for follow_path in node halt!
```

이 메시지만으로 전체 실패로 판단하지 않는다. 정밀 정렬 전환이나 Action 취소 과정에서도 발생할 수 있다. 최종적으로 TaskManager의 실패 로그가 있는지와 `COMPLETED → IDLE` 전환 여부를 함께 확인한다.

---

## 12. 종료 방법

정상 종료는 통합 실행 터미널에서 `Ctrl+C`를 누른다.

실행 스크립트가 ROS launch와 Isaac Sim 프로세스 그룹을 함께 종료한다.

잔여 프로세스가 남은 경우 다음 스크립트를 실행한다.

```bash
cd ~/book-shelving-system
./scripts/emergency_stop.sh
```

종료 후 확인:

```bash
pgrep -af \
'run_simulation.py|ros2 launch shelving_system|nav2_|pointcloud_to_laserscan'
```

아무것도 출력되지 않으면 정상적으로 종료된 것이다.

---

## 13. 자주 발생하는 문제

### `ultralytics`가 없다고 나오는 경우

```bash
cd ~/book-shelving-system

python3 -m venv .venv

./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install \
  "numpy==1.26.4" \
  "ultralytics==8.4.155"
```

### USD 또는 모델 파일이 없다고 나오는 경우

```bash
cd ~/book-shelving-system
git lfs install
git lfs pull
```

### Isaac Sim 터미널에는 `App ready`가 나오지만 창이 보이지 않는 경우

```bash
export DISPLAY=:0
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export XAUTHORITY="$XDG_RUNTIME_DIR/gdm/Xauthority"
```

그 후 같은 터미널에서 다시 실행한다.

```bash
./scripts/run.sh full
```

### 이전 Isaac Sim 프로세스가 남아 있다고 나오는 경우

```bash
cd ~/book-shelving-system
./scripts/emergency_stop.sh
```

### ROS workspace가 빌드되지 않았다고 나오는 경우

```bash
cd ~/book-shelving-system/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 14. 현재 제한사항

- 현재 검증된 작업 수는 책 1권이다.
- 다권 연속 배치는 아직 최종 검증되지 않았다.
- 트레이 양 끝 책은 현재 Franka 작업영역 밖에 있을 수 있다.
- 파지 후 책 추종은 Isaac Sim 안정화를 위한 시뮬레이션 전용 처리다.
- Stop/Play만으로 전체 장면을 완전히 초기화하는 기능은 아직 최종 완료되지 않았다.
- `scripts/setup_check.sh`에는 Carter/M0609 기준의 오래된 검사 항목이 남아 있으므로 Ridgeback-Franka용으로 수정하기 전에는 결과를 신뢰하지 않는다.