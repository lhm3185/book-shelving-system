# Git 관리 규칙과 터미널 명령 가이드

## 1. 기준 경로

현재 PC의 저장소 경로는 다음과 같다.

```text
/home/rokey/isaac_ws/book-shelving-system
```

명령 예시는 저장소 루트에서 실행한다. 다른 PC에서는 자신의 clone 경로로 바꾼다.

## 2. 작업 시작

```bash
cd /home/rokey/isaac_ws/book-shelving-system
git status --short
git branch --show-current
git fetch origin
```

새 작업은 최신 main에서 기능 브랜치를 만든다.

```bash
git switch main
git pull --ff-only origin main
git switch -c docs/update-current-architecture
```

main에 직접 기능 commit을 만들지 않는다.

## 3. 브랜치와 commit

| 작업 | 브랜치 예시 | commit 예시 |
| --- | --- | --- |
| 시스템 | `feature/system-supervisor` | `feat(system): gate cycle start on readiness` |
| 비전 | `feature/perception-slot-depth` | `feat(perception): project slot onto shelf plane` |
| AMR | `feature/navigation-fine-align` | `feat(navigation): add final pose alignment` |
| 조작 | `feature/manipulation-sweep` | `feat(manipulation): scan shelf before placement` |
| 문서 | `docs/current-runtime` | `docs: align architecture with current runtime` |
| 수정 | `fix/rviz-display` | `fix(runtime): launch rviz on pc b display` |

의도한 파일만 stage한다.

```bash
git diff
git add docs/01_system_overview.md docs/02_architecture_and_flow.md
git diff --cached
git commit -m "docs: align architecture with current runtime"
```

## 4. push와 Pull Request

```bash
git push -u origin HEAD
```

PR에는 변경 목적, 영향 파일, msg/action/topic/frame 변화, 시험 명령과 결과, 남은 제한을 기록한다. 인터페이스·launch·Isaac scene·network 변경은 [07_file_ownership.md](07_file_ownership.md)의 관련 담당자가 검토한다.

## 5. main 반영과 충돌

```bash
git fetch origin
git rebase origin/main
```

충돌 시:

```bash
git status
# 파일을 수정한 뒤
git add <resolved-file>
git rebase --continue
```

중단하려면 `git rebase --abort`를 사용한다. `git reset --hard`, `git clean -fd`, `git push --force`는 팀 작업에서 임의로 사용하지 않는다.

## 6. 최초 환경 준비

```bash
sudo apt update
sudo apt install -y git-lfs python3-colcon-common-extensions python3-rosdep python3-venv
git lfs install
git lfs pull

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

Perception 환경:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install "numpy==1.26.4" "ultralytics==8.4.155"
```

PC C:

```bash
sudo apt install -y python3-fastapi python3-uvicorn python3-yaml
```

## 7. 빌드

권장 명령:

```bash
./scripts/ros/build.sh
```

수동 명령:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

특정 package만 빌드할 때도 의존 package를 함께 고려한다.

```bash
colcon build --symlink-install --packages-select shelving_navigation
colcon build --symlink-install --packages-up-to shelving_manipulation
```

## 8. 시험

현재 자동시험 진입점:

```bash
./scripts/tests/run.sh
./scripts/tests/run.sh sim
./scripts/tests/run.sh ros
```

2026-09-30 기준 runner는 sim import 경로와 manipulation lint 문제가 남아 있다. 자세한 결과는 [05_test_plan_and_results.md](05_test_plan_and_results.md)를 확인한다.

package별 직접 실행:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 -m pytest ros2_ws/src/shelving_system/test -q
python3 -m pytest ros2_ws/src/shelving_perception/test -q
python3 -m pytest ros2_ws/src/shelving_navigation/test -q
python3 -m pytest ros2_ws/src/shelving_manipulation/test -q
```

## 9. PC별 실행

### PC B

```bash
./scripts/ros/run_pc_b.sh
```

Nav2, perception, manipulation과 RViz를 실행한다. RViz는 PC B `DISPLAY=:1`에 표시한다.

### PC A

```bash
./scripts/ros/run_pc_a.sh
```

Isaac Sim을 시작하고 PC B 준비 후 Task Manager, bridge, return machine과 supervisor를 실행한다.

### PC C

```bash
./scripts/ros/run_pc_c.sh
```

웹 주소는 `http://localhost:8080` 또는 유선망의 `http://10.10.0.3:8080`이다.

### 한 PC 통합

```bash
./scripts/run.sh navigation-test
./scripts/run.sh full
```

`navigation-test`는 mock manipulation, `full`은 perception과 실제 Isaac manipulation backend를 사용한다. 이 스크립트는 준비 확인 후 테스트 TrayJob을 자동 발행한다.

## 10. 작업 시작과 상태 확인

3-PC 실행에서는 웹 버튼 또는 다음 명령으로 시작한다.

```bash
ros2 service call /system/start_cycle std_srvs/srv/Trigger '{}'
```

```bash
ros2 topic echo /system/status --once
ros2 topic echo /system/state --once
ros2 topic echo /scenario/state --once
ros2 action list -t
ros2 topic hz /scan
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
```

## 11. 종료

일반 종료는 실행 터미널의 `Ctrl+C`를 사용한다. 전체 프로세스를 정리해야 하면:

```bash
./scripts/emergency_stop.sh
```

종료 후 확인:

```bash
pgrep -af 'run_simulation.py|ros2 launch|rviz2|web_gateway_node'
```

## 12. 커밋 금지 항목

- `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`
- 프로젝트 `.venv`
- ROS 로그, `/tmp` runtime state와 RViz 로그
- 비밀번호, 토큰, 개인 인증서와 실제 DDS 개인 설정
- 실행 중 생성된 임시 이미지·영상·bag 파일

USD, PNG, PT 등 대용량 파일은 기존 Git LFS 정책을 확인한 뒤 추가한다.

## 13. 공통 체크리스트

### 작업 전

- 같은 base commit과 network 설정인지 확인
- 담당 파일과 공통 검토자 확인
- `git status --short`로 기존 변경 보호

### commit 전

- diff와 생성물 확인
- 관련 build/test 실행
- topic/action/frame/config 변경 문서화

### 시연 전

- 세 PC 동일 commit
- PC B → PC A → PC C 순서 기동
- SystemStatus READY와 RViz map/scan/costmap 확인
- 작업 시작은 한 번만 요청
