# 실행 및 운영 Runbook

## 1. 현재 상태

이 저장소의 다섯 패키지는 ROS2 Jazzy의 ros2 pkg create로 생성했고, package.xml, setup.py, setup.cfg, resource marker와 공통 메시지·액션 생성 설정을 갖춘다. colcon build로 다섯 패키지와 여섯 인터페이스가 정상 생성되는 것을 확인했다. 다만 담당 기능의 Python 알고리즘, launch 내용, YAML 값과 실행 스크립트는 아직 빈 구현 골격이므로 실제 로봇 동작은 각 담당자가 개발해야 한다.

## 2. 실행 구성

| 실행 대상 | 패키지·기능 |
| --- | --- |
| PC A | Isaac Sim, shelving_system, Task Manager, YAML, 무인반납기 노드 |
| PC B | shelving_perception, shelving_navigation, shelving_manipulation |
| 통합 개발 PC | all.launch.py로 전체 노드 |
| 모니터 PC | ROS2 토픽, action, TF와 로그 확인 |

## 3. 실행 전 확인

- PC A와 PC B가 같은 ROS2 배포판과 interfaces 버전을 사용한다.
- 두 PC의 ROS_DOMAIN_ID와 DDS 설정이 같다.
- PC A와 PC B가 네트워크에서 서로 도달 가능하다.
- Isaac Sim stage와 ROS2 bridge가 정상 로드된다.
- RGB-D 카메라 frame이 존재한다.
- map→odom→base_link→arm_base_link와 camera_link TF가 연결된다.
- YAML의 목표 서가와 waypoint가 존재한다.
- AMR은 HOME, 로봇팔은 주행 안전 자세다.

## 4. 전체 빌드

~~~bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
~~~

하나의 workspace 안에 다섯 패키지가 있으므로 별도 workspace를 만들거나 담당자별로 따로 빌드할 필요가 없다. 특정 패키지만 확인할 때는 packages-select 옵션을 사용할 수 있지만 통합 전에는 전체 빌드를 수행한다.

## 5. PC A 실행

~~~bash
./scripts/run_pc_a.sh
~~~

또는 구현된 launch 파일을 직접 실행한다.

~~~bash
ros2 launch shelving_system pc_a.launch.py
~~~

다음 항목을 확인한다.

- Isaac Sim과 ROS2 bridge
- return_machine_node 준비
- task_manager_node 준비
- YAML 로드 성공
- FSM이 INITIALIZING에서 IDLE로 전이
- TrayJob 발행 가능

## 6. PC B 실행

~~~bash
./scripts/run_pc_b.sh
~~~

또는 다음 launch 파일을 사용한다.

~~~bash
ros2 launch shelving_system pc_b.launch.py
~~~

다음 항목을 확인한다.

- perception node와 RGB-D 카메라 입력
- navigation node와 Nav2 준비
- manipulation node와 MoveIt·joint state
- NavigateToTarget, DetectTargetSlot, PlaceBook action server
- RobotStatus heartbeat

## 7. 한 컴퓨터 통합 실행

~~~bash
./scripts/run_all.sh
~~~

또는 다음을 사용한다.

~~~bash
ros2 launch shelving_system all.launch.py
~~~

이 방식은 통합 개발과 반복시험용이며, 실제 두 PC 시험에서는 pc_a와 pc_b launch를 나누어 실행한다.

## 8. 정상 시연 순서

1. library_system.usd를 Isaac Sim에서 연다.
2. AMR, 로봇팔, 카메라, 서가, 트레이와 책 초기 위치를 확인한다.
3. PC A를 실행하고 FSM IDLE을 확인한다.
4. PC B를 실행하고 세 기능의 READY를 확인한다.
5. TF와 RGB-D 입력을 확인한다.
6. 정상 시나리오의 TrayJob을 한 번 발행한다.
7. CP1 트레이 확인을 관찰한다.
8. AMR의 서가 관측 위치 이동과 CP2 정렬을 확인한다.
9. CP3에서 선택한 빈 공간의 폭, pose와 confidence를 확인한다.
10. CP4 파지·삽입·해제·후퇴를 확인한다.
11. 성공 후 YAML 상태가 갱신되는지 확인한다.
12. AMR HOME 복귀와 FSM IDLE을 확인한다.
13. job_id 기준 로그와 시험 결과를 저장한다.

## 9. 중지 조건

- 필수 node 또는 heartbeat가 없으면 TrayJob을 시작하지 않는다.
- 트레이와 작업 정보가 일치하지 않으면 서가로 이동하지 않는다.
- 서가가 카메라 시야나 로봇팔 범위에 없으면 빈 공간 인식을 확정하지 않는다.
- 빈 공간이 책보다 좁거나 confidence가 낮으면 삽입하지 않는다.
- 좌표 변환이 실패하거나 TF가 오래되었으면 TargetSlot을 사용하지 않는다.
- IK 또는 충돌 검사가 실패하면 로봇팔을 움직이지 않는다.
- 배치 확인이 실패하면 점유 상태를 변경하지 않는다.

## 10. 장애 대응

### PC A와 PC B 통신 불가

ROS_DOMAIN_ID, DDS 설정, 네트워크 인터페이스와 방화벽을 확인한다. 양쪽의 shelving_interfaces 버전이 같은지 확인한다. 진행 중 작업이 있었다면 양쪽 job_id와 로봇의 물리 상태를 확인하기 전 자동 재개하지 않는다.

### 서가 또는 빈 공간 미검출

RGB-D 입력, 노출, depth 유효 범위, 관측 거리와 카메라 각도를 확인한다. AMR 자세를 한 번 재정렬하고 재촬영한다. 계속 실패하면 삽입하지 않고 작업을 종료한다.

### 잘못된 좌표

camera_link와 arm_base_link transform, timestamp와 단위를 확인한다. RViz와 Isaac Sim에서 TargetSlot을 표시하여 실제 빈 공간과 일치하는지 확인한다. 좌표 검증 전에는 로봇팔 삽입을 수행하지 않는다.

### AMR 이동 또는 정렬 실패

localization, map, 목표 frame과 장애물을 확인한다. 허용된 횟수만 재계획·재정렬하고 계속 실패하면 안전 정지한다.

### 파지 또는 삽입 실패

책을 잡고 있는지 먼저 확인한다. 책을 잡은 상태에서는 임의로 그리퍼를 열지 않는다. 삽입 중 문제가 생기면 전진을 멈추고 짧게 후퇴한 뒤 pose, 책 크기와 충돌 모델을 확인한다.

## 11. 로그와 증거

- 로그에 job_id, book_id, component, FSM state와 timestamp를 포함한다.
- 통합시험 시 주요 토픽과 TF를 rosbag으로 저장한다.
- CP2 위치·yaw 오차, CP3 공간 크기·confidence, CP4 파지 횟수와 삽입 결과를 기록한다.
- 결과는 docs/05_test_plan_and_results.md에 남긴다.
- 영상과 로그 이름에 날짜, scenario와 run 번호를 포함한다.

## 12. 종료 절차

1. 새 TrayJob 접수를 중지한다.
2. 진행 중 작업을 완료하거나 안전 취소 상태로 이동한다.
3. 로봇팔을 주행 안전 자세로 이동한다.
4. AMR을 HOME으로 복귀시킨다.
5. PC B 노드를 종료한다.
6. PC A의 Task Manager와 무인반납기 노드를 종료한다.
7. Isaac Sim을 종료한다.
8. YAML 상태와 로그 저장 여부를 확인한다.

## 13. 발표 당일

검증된 commit과 설정을 발표 직전에 변경하지 않는다. 정상 시나리오, YAML, USD와 실행 순서를 고정한다. 마지막 성공 영상과 주요 로그를 백업으로 준비하되 라이브 시연 실패를 성공으로 표현하지 않고 안전정지와 원인을 설명한다.
