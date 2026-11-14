# 실행 및 운영 Runbook

## 1. 실행 구성

| 대상 | 역할 | 한 줄 실행 |
| --- | --- | --- |
| PC B | Nav2, AMCL, perception, manipulation, RViz | `./scripts/ros/run_pc_b.sh` |
| PC A | Isaac Sim, Task Manager, bridge, supervisor | `./scripts/ros/run_pc_a.sh` |
| PC C | 웹 대시보드 | `./scripts/ros/run_pc_c.sh` |
| 한 PC 개발 | Isaac+ROS 통합 | `./scripts/run.sh full` |

PC A 스크립트가 PC B 준비를 기다리므로 PC B를 먼저 실행하는 것이 가장 명확하다.

## 2. 공통 사전 조건

- Ubuntu 24.04, ROS 2 Jazzy, Isaac Sim 5.1
- 세 PC에 동일한 repository commit과 빌드 결과
- `config/ros_network.env`: `ROS_DOMAIN_ID=130`, `rmw_fastrtps_cpp`
- PC A/B/C의 `$HOME/.ros/fastdds_whitelist.xml` 존재
- PC B의 `.venv/lib/python3.12/site-packages/ultralytics` 존재
- PC C의 `fastapi`, `uvicorn`, `yaml` Python 패키지 존재
- 유선 ROS 네트워크에서 서로 도달 가능

## 3. 빌드

저장소 루트에서 실행한다.

```bash
./scripts/ros/build.sh
```

수동 빌드는 다음과 같다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 4. PC B 실행

```bash
cd /home/rokey/isaac_ws/book-shelving-system
./scripts/ros/run_pc_b.sh
```

스크립트가 network env와 workspace를 source하고 `pc_b.launch.py`를 시작한다. 이어서 Nav2 기본 RViz를 PC B의 `DISPLAY=:1`에 띄우며 `/run/user/1000/gdm/Xauthority`를 사용한다.

정상 메시지:

```text
PC B를 시작합니다. ROS_DOMAIN_ID=130
RViz를 DISPLAY=:1에 시작했습니다.
```

RViz가 실패해도 ROS 노드는 계속 실행되며 원인은 다음 파일에 남는다.

```text
/tmp/book-shelving-system-rviz-1000.log
```

RViz에서 확인할 항목은 fixed frame `map`, `/map`, `/scan`, `/amcl_pose`, global/local costmap과 `/plan`이다.

## 5. PC A 실행

```bash
cd /home/rokey/isaac_ws/book-shelving-system
./scripts/ros/run_pc_a.sh
```

스크립트는 다음 순서로 동작한다.

1. GNOME 그래픽 세션을 찾고 Isaac Sim을 시작한다.
2. `/clock`, `/odom`을 기다린다.
3. PC B의 navigation/perception/manipulation 준비를 무기한 기다린다.
4. `pc_a.launch.py`를 시작한다.
5. `/system/status`가 READY가 될 때까지 기다린다.
6. 자동으로 작업을 시작하지 않고 운영자 명령을 기다린다.

## 6. PC C 실행

```bash
cd /home/rokey/isaac_ws/book-shelving-system
./scripts/ros/run_pc_c.sh
```

브라우저 주소:

- PC C 자체: `http://localhost:8080`
- 프로젝트 유선망: `http://10.10.0.3:8080`

대시보드에서 상태, phase, 준비 여부, 로봇 pose와 map을 확인하고 READY일 때 사이클을 시작한다.

## 7. CLI로 작업 시작

PC C 없이 시작하려면 ROS 환경이 설정된 터미널에서 다음 service만 호출한다.

```bash
ros2 service call /system/start_cycle std_srvs/srv/Trigger '{}'
```

Supervisor가 READY가 아니면 요청을 거부하고 빠진 component를 message로 알려준다. `/return_machine/publish_job`을 직접 호출하는 것은 supervisor 준비 게이트를 우회하므로 정상 운영에서는 사용하지 않는다.

## 8. 정상 시연 체크

1. PC B 터미널과 RViz에서 Nav2, map, scan, costmap을 확인한다.
2. PC A의 Isaac Sim에서 로봇, 반납기, 트레이, 책과 서가 초기 상태를 확인한다.
3. `/system/status` 또는 웹 화면이 READY인지 확인한다.
4. 작업을 한 번 시작한다.
5. 반납기 이동과 실제 트레이 적재를 확인한다.
6. `shelf_01` 이동과 정밀 정렬을 확인한다.
7. 팔 스윕, 빈 슬롯 선택과 AMR 횡정렬을 확인한다.
8. 트레이 책 재인식, 파지·삽입·해제·후퇴를 확인한다.
9. 배치 검증 후 서가 후퇴와 HOME 복귀를 확인한다.
10. FSM IDLE, SystemStatus READY 복귀와 job/run ID를 기록한다.

## 9. 상태 확인 명령

```bash
ros2 topic echo /system/status --once
ros2 topic echo /system/state --once
ros2 topic echo /scenario/state --once
ros2 action list
ros2 topic hz /scan
ros2 topic hz /rgb
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
```

PC 간 discovery 확인:

```bash
ros2 node list
ros2 topic list -t
ros2 action list -t
```

## 10. 장애 대응

### PC A가 PC B를 계속 기다림

PC B에서 `run_pc_b.sh`가 실행 중인지 확인한다. `/navigate_to_target`, `/place_book`, `/detect_grasp_point`, `/detect_target_slot`, `/scan`, `/amcl_pose`, `map→base_link` 중 빠진 항목을 확인한다. 양쪽 `config/ros_network.env`와 Fast DDS whitelist가 같아야 한다.

### RViz 창이 뜨지 않음

```bash
cat /tmp/book-shelving-system-rviz-1000.log
echo "$DISPLAY"
echo "$XAUTHORITY"
xdpyinfo -display :1 >/dev/null && echo OK
```

PC B 스크립트는 `DISPLAY=:1`과 GDM Xauthority를 내부 설정하므로 추가 export는 필요 없다.

### SystemStatus가 WAITING_FOR_PC_B

status message의 missing 목록을 확인한다. scan은 3초 이내 최신이어야 하고 AMCL pose는 현재 scenario run에서 최소 한 번 받아야 한다.

### 슬롯 또는 책 미검출

`/rgb`, `/depth`, `/camera_info`, TF와 `/perception/debug_image`를 확인한다. 차체 정렬 전 프레임은 `not_before` 때문에 거부되는 것이 정상이다. ROI, YOLO 모델과 shelf plane 값을 확인한다.

### 주행·정렬 실패

RViz에서 localization, costmap, footprint와 계획 경로를 확인한다. navigation은 마지막 목표를 제한적으로 재시도하고 접근 반경 안에서는 정밀 정렬로 전환한다. `shelf_retreat` 목표가 0.75 m를 넘으면 안전상 거부된다.

### 조작 실패

`PlaceBook`의 `failed_phase`, error code, message와 `/manipulation/sim/state`를 함께 확인한다. 책을 잡았을 가능성이 있으면 임의로 gripper를 열거나 scenario를 재개하지 말고 Isaac 상태를 먼저 확인한다.

## 11. Reset과 종료

Timeline Stop/Play는 scenario reset이며 자동 작업 재시작 명령이 아니다. reset 뒤 supervisor가 READY로 돌아온 것을 확인하고 새 사이클을 시작한다.

정상 종료는 각 실행 터미널에서 `Ctrl+C`를 사용한다. 전체 프로젝트 프로세스를 즉시 정리해야 하면 저장소 루트에서 다음을 실행한다.

```bash
./scripts/emergency_stop.sh
```

이 스크립트는 등록된 PC A/B ROS, Isaac, RViz와 웹 프로세스 그룹에 순차적으로 INT, TERM, KILL을 적용한다.

## 12. 발표 당일

- 세 PC에 같은 commit과 `ROS_DOMAIN_ID=130`을 배포한다.
- 좌표, 모델, map과 Fast DDS 설정을 직전에 변경하지 않는다.
- 먼저 PC B, 다음 PC A, 마지막 PC C를 실행한다.
- READY와 RViz 화면을 확인한 뒤 사이클을 한 번만 시작한다.
- 실패하면 성공으로 표현하지 말고 phase/error/message와 안전 정지 상태를 설명한다.
