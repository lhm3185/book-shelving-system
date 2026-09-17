# Git 관리 규칙과 터미널 명령 가이드

## 1. 목적

네 명이 하나의 저장소와 ROS2 workspace를 공유할 때 main 손상, 파일 덮어쓰기와 대용량 USD 충돌을 방지하기 위한 공통 절차다. 모든 명령은 특별한 설명이 없으면 저장소 루트인 /home/hymi/book-shelving-system에서 실행한다.

## 2. 최초 1회 설정

### 2.1 Git 사용자 정보

각자 자신의 이름과 이메일을 설정한다. 다른 팀원의 계정을 복사하지 않는다.

~~~bash
git config --global user.name "본인 이름"
git config --global user.email "본인 이메일"
git config --global init.defaultBranch main
~~~

설정 확인:

~~~bash
git config --global --get user.name
git config --global --get user.email
~~~

### 2.2 Git LFS

이 저장소는 USD, PGM, PNG와 JPG를 Git LFS로 관리한다. Git LFS가 없으면 대용량 자산 대신 pointer만 받거나 push가 실패할 수 있다.

~~~bash
git lfs install
git lfs env
~~~

저장소를 새로 받는 경우:

~~~bash
cd /home/hymi
git clone https://github.com/lhm3185/book-shelving-system.git
cd book-shelving-system
git lfs pull
git status
~~~

이미 clone한 경우에도 다음 명령으로 LFS 파일 상태를 확인한다.

~~~bash
cd /home/hymi/book-shelving-system
git lfs ls-files
git lfs pull
~~~

## 3. 브랜치 운영 규칙

main은 항상 빌드 가능하고 통합 가능한 상태로 유지한다. main에서 직접 코드를 작성하거나 commit하지 않는다.

| 작업 | 브랜치 예시 |
| --- | --- |
| FSM·시스템 | feature/system-fsm |
| 인터페이스 | feature/interfaces-target-slot |
| 비전 | feature/perception-empty-slot |
| AMR | feature/navigation-docking |
| 로봇팔 | feature/manipulation-place-book |
| 통합시험 | test/normal-flow |
| 문서 | docs/file-ownership |
| 버그 수정 | fix/perception-frame-id |

브랜치 이름은 영문 소문자와 하이픈을 사용한다. 개인 이름보다 기능을 적어 다른 팀원이 목적을 이해할 수 있게 한다.

## 4. 매일 작업 시작 절차

새 작업을 시작할 때는 main을 먼저 최신화하고 그 상태에서 기능 브랜치를 만든다.

~~~bash
cd /home/hymi/book-shelving-system
git status
git switch main
git pull --ff-only origin main
git switch -c feature/perception-empty-slot
~~~

이미 만든 브랜치에서 계속 작업하는 경우:

~~~bash
cd /home/hymi/book-shelving-system
git switch feature/perception-empty-slot
git fetch origin
git rebase origin/main
git status
~~~

작업 중인 변경이 남아 있으면 무작정 다른 브랜치로 이동하지 않는다. 먼저 git status와 git diff를 확인하고 commit하거나, 정말 임시 변경일 때만 git stash를 사용한다.

## 5. 작업 중 확인 명령

~~~bash
git status
git diff
git diff --stat
git branch --show-current
git log --oneline --decorate -10
~~~

특정 파일만 확인:

~~~bash
git diff -- ros2_ws/src/shelving_perception/shelving_perception/shelf_slot_detector.py
~~~

추적하지 않는 파일까지 확인:

~~~bash
git status --short
~~~

build, install, log는 .gitignore 대상이다. 이 디렉터리가 status에 나타난다면 .gitignore와 실행 위치를 확인한다.

## 6. 파일 추가와 Commit

모든 변경을 한 번에 추가하는 git add .보다 자신이 의도한 파일만 지정하는 방식을 권장한다.

~~~bash
git add ros2_ws/src/shelving_perception/shelving_perception/shelf_slot_detector.py
git add ros2_ws/src/shelving_perception/config/perception.yaml
git diff --cached
git commit -m "feat(perception): detect empty shelf slots"
~~~

권장 commit 형식:

~~~text
type(scope): summary
~~~

| type | 사용 시점 | 예 |
| --- | --- | --- |
| feat | 기능 추가 | feat(navigation): add shelf docking action |
| fix | 오류 수정 | fix(manipulation): stop insertion on collision |
| docs | 문서만 수정 | docs: document file ownership |
| test | 시험 추가·수정 | test(system): add normal flow scenario |
| refactor | 동작을 바꾸지 않는 구조 개선 | refactor(system): separate yaml validation |
| chore | 설정·빌드·정리 | chore: update package dependencies |

한 commit에는 하나의 설명 가능한 변경만 넣는다. 인터페이스 변경, 기능 구현과 무관한 formatting을 한 commit에 섞지 않는다.

## 7. Push와 Pull Request

첫 push:

~~~bash
git push -u origin feature/perception-empty-slot
~~~

이후 push:

~~~bash
git push
~~~

GitHub에서 feature 브랜치에서 main으로 Pull Request를 생성한다. Pull Request 설명에는 다음을 적는다.

- 무엇을 구현하거나 수정했는지
- 관련 담당 패키지와 파일
- 시험한 명령과 결과
- ROS2 topic, action, frame 또는 YAML 변경
- 다른 담당자에게 미치는 영향
- 스크린샷·로그·영상이 있으면 증거
- 남은 제한사항

최소 한 명의 관련 담당자가 검토하고, 공통 인터페이스·launch·통합 USD 변경은 영향을 받는 담당자의 검토를 모두 받은 뒤 merge한다.

## 8. main 최신 변경 반영

Pull Request 전에 기능 브랜치에 최신 main을 반영한다.

~~~bash
git switch feature/perception-empty-slot
git fetch origin
git rebase origin/main
~~~

충돌이 없으면 시험 후 push한다. rebase 후 이미 원격에 올린 개인 기능 브랜치를 갱신해야 할 때만 다음 명령을 사용한다.

~~~bash
git push --force-with-lease
~~~

force-with-lease도 main에는 사용하지 않는다. 여러 사람이 공동으로 쓰는 브랜치에서는 먼저 팀원과 합의한다.

## 9. 충돌 해결

rebase 중 충돌이 발생하면 다음 순서로 처리한다.

~~~bash
git status
~~~

충돌 파일을 열어 현재 브랜치와 main의 변경을 비교하고 필요한 내용만 남긴다. 수정 후:

~~~bash
git add 충돌을_해결한_파일
git rebase --continue
~~~

해결 방향이 불분명하면 억지로 선택하지 말고 담당자와 확인한다. rebase를 시작 전 상태로 되돌리려면:

~~~bash
git rebase --abort
~~~

다음 명령은 팀 저장소에서 임의로 사용하지 않는다.

~~~text
git reset --hard
git push --force
git clean -fd
~~~

특히 USD 충돌은 줄 단위로 안전하게 합치기 어렵다. 양쪽 파일을 억지로 merge하지 말고 주 담당자의 정상 자산을 기준으로 다시 적용한다.

## 10. Pull 사용 시 주의

단순 git pull은 자동 merge commit을 만들 수 있으므로 main에서는 다음을 사용한다.

~~~bash
git switch main
git pull --ff-only origin main
~~~

기능 브랜치에서는 fetch와 rebase를 권장한다.

~~~bash
git fetch origin
git rebase origin/main
~~~

작업 파일이 수정된 상태에서 pull 또는 rebase하지 않는다. git status로 clean 상태인지 먼저 확인한다.

## 11. ROS2 최초 환경 준비

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
~~~

rosdep가 설치하지 못한 패키지는 오류 메시지를 기록하고 package.xml의 의존성이 올바른지 담당자와 확인한다.

## 12. 전체 빌드

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
colcon build --symlink-install
source install/setup.bash
~~~

빌드 결과 확인:

~~~bash
colcon list
ros2 pkg list | grep shelving
ros2 interface show shelving_interfaces/msg/TrayJob
ros2 interface show shelving_interfaces/action/DetectTargetSlot
~~~

현재 scripts/build.sh는 빈 골격이므로 내용이 구현되기 전에는 위의 colcon 명령을 직접 사용한다.

## 13. 담당 패키지만 빌드

### 이현민

~~~bash
cd /home/hymi/book-shelving-system/ros2_ws
colcon build --symlink-install --packages-select shelving_interfaces shelving_system
source install/setup.bash
~~~

interfaces를 변경했다면 영향을 받는 전체 패키지를 다시 빌드한다.

~~~bash
colcon build --symlink-install
~~~

### 윤재민

~~~bash
cd /home/hymi/book-shelving-system/ros2_ws
colcon build --symlink-install --packages-select shelving_perception
source install/setup.bash
~~~

### 이동준

~~~bash
cd /home/hymi/book-shelving-system/ros2_ws
colcon build --symlink-install --packages-select shelving_navigation
source install/setup.bash
~~~

### 김도윤

~~~bash
cd /home/hymi/book-shelving-system/ros2_ws
colcon build --symlink-install --packages-select shelving_manipulation
source install/setup.bash
~~~

## 14. 시험 명령

전체 시험:

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
source install/setup.bash
colcon test
colcon test-result --verbose
~~~

담당 패키지만 시험:

~~~bash
colcon test --packages-select shelving_perception
colcon test-result --verbose
~~~

통합시험 파일이 구현된 후 저장소 루트에서 실행:

~~~bash
cd /home/hymi/book-shelving-system
pytest -v tests/integration/test_normal_flow.py
~~~

현재 scripts/run_tests.sh와 통합시험은 빈 골격이므로 실제 시험 코드가 채워지기 전에는 lint scaffold의 결과만 확인할 수 있다.

## 15. PC별 실행 명령

현재 launch 파일과 run 스크립트는 빈 골격이다. 노드 entry point와 launch 내용이 구현된 이후 다음 명령을 사용한다.

### PC A 터미널

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch shelving_system pc_a.launch.py
~~~

### PC B 터미널

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch shelving_system pc_b.launch.py
~~~

두 PC는 같은 ROS_DOMAIN_ID를 사용해야 한다. 30은 예시이므로 팀이 정한 값으로 통일한다.

### 한 PC 통합 실행

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/hymi/book-shelving-system/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch shelving_system all.launch.py
~~~

## 16. ROS2 상태 확인 명령

새 터미널마다 ROS 환경과 workspace를 source한 뒤 실행한다.

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/hymi/book-shelving-system/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
~~~

상태와 작업 확인 예시:

~~~bash
ros2 topic echo /robot/status
ros2 topic echo /return_machine/tray_job
ros2 action info /navigate_to_target
ros2 action info /detect_target_slot
ros2 action info /place_book
~~~

TF 확인:

~~~bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo camera_link arm_base_link
~~~

실제 topic과 action 이름은 노드 구현 시 docs/03_ros_interfaces.md와 함께 확정한다.

## 17. 두 PC 통신 확인

PC A와 PC B에서 각각 다음을 확인한다.

~~~bash
echo $ROS_DOMAIN_ID
ros2 node list
ros2 topic list
~~~

노드가 서로 보이지 않으면 다음 항목을 확인한다.

- 같은 네트워크와 ROS_DOMAIN_ID인지
- ROS_LOCALHOST_ONLY가 0인지
- 방화벽이 DDS 통신을 막는지
- 양쪽 shelving_interfaces commit이 같은지
- 양쪽에서 workspace install/setup.bash를 source했는지

환경변수 확인:

~~~bash
printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
~~~

## 18. 작업 종료 절차

기능 브랜치 작업을 마치기 전에 다음을 수행한다.

~~~bash
cd /home/hymi/book-shelving-system
git status
git diff
~~~

빌드와 시험 후 필요한 파일만 stage한다.

~~~bash
git add 담당한_파일
git diff --cached
git commit -m "feat(scope): concise summary"
git push
~~~

GitHub Pull Request를 만들고 검토자를 지정한다. merge된 후 로컬 정리:

~~~bash
git switch main
git pull --ff-only origin main
git branch -d 작업이_끝난_브랜치
~~~

원격 브랜치 삭제는 Pull Request merge 여부를 확인한 뒤 GitHub에서 수행하거나 다음 명령을 사용한다.

~~~bash
git push origin --delete 작업이_끝난_브랜치
~~~

## 19. 절대 커밋하지 않을 항목

- ros2_ws/build
- ros2_ws/install
- ros2_ws/log
- __pycache__와 pyc
- .env와 비밀번호·API key·개인 토큰
- 개인 IDE 설정과 임시 파일
- 재생성 가능한 대용량 로그와 rosbag
- 담당자가 확인하지 않은 충돌 해결 결과

추가 제외가 필요하면 개인 PC에서만 필요한 파일인지 팀 전체가 제외해야 하는 파일인지 확인한 뒤 .gitignore를 수정한다.

## 20. 팀 공통 체크리스트

### 작업 시작

- main 최신화
- 올바른 기능 브랜치 생성 또는 이동
- Git LFS와 관련 USD 최신화
- 담당 파일과 인터페이스 영향 확인

### Commit 전

- git status와 git diff 확인
- 담당 패키지 빌드
- 관련 시험 실행
- YAML·frame·단위와 error_code 확인
- 문서가 실제 구현과 일치하는지 확인

### Pull Request 전

- origin/main rebase
- 전체 또는 영향 패키지 재빌드
- 시험 결과 기록
- 공통 파일 검토자 지정
- 비밀정보와 생성물이 포함되지 않았는지 확인

### Merge 후

- main에서 동일 commit 확인
- PC A/B가 같은 interfaces 버전인지 확인
- 통합시험 결과와 발견한 결함 기록

