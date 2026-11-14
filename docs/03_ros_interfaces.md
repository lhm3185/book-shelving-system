# ROS 2 인터페이스 명세

## 1. 공통 규칙

- 실제 정의의 기준은 `ros2_ws/src/shelving_interfaces`다.
- 길이는 m, 각도는 rad를 사용한다.
- pose 결과는 `std_msgs/Header`로 frame과 timestamp를 전달한다.
- 장시간 동작은 action, 운영자 트리거는 service, 연속 상태는 topic을 사용한다.
- Isaac 전용 JSON 토픽은 bridge 내부 계약이다.

## 2. 메시지

### `TrayJob.msg`

`job_id`, `tray_id`, `created_at`, `book_ids`, `rfid_tags`, `classification_codes`로 구성한다. 세 배열은 길이가 같고 책 ID는 중복되지 않아야 한다. 기본 topic은 `/return_machine/tray_job`이다.

### `RobotStatus.msg`

`header`, `component`, `state`, `active_job_id`, `progress`, `error_code`, `message`, `heartbeat_time`으로 실행 상태를 나타낸다. Task Manager 상태는 `/system/state`, component 상태는 `/robot/status`를 사용한다.

### `TargetSlot.msg`

`header`, `pose`, `available_width`, `available_height`, `insertion_depth`, `pre_insert_offset`, `confidence`로 구성한다. 현재 최종 frame은 `arm_base_link`다.

### `GraspObservation.msg`

`header`, `top_center`, `spine_yaw`, `thickness`, `width`, `height`, `confidence`로 구성한다. `top_center`는 `arm_base_link` 기준 책 윗면 중심이다.

### `ScenarioState.msg`

상수 `UNKNOWN`, `STOPPED`, `RESETTING`, `READY`, `RUNNING`, `FAILED`와 `run_id`, `state`, `message`를 가진다. bridge가 `/simulation/scenario/state` JSON을 `/scenario/state`로 변환한다.

### `SystemStatus.msg`

상수 `BOOTING`, `WAITING_FOR_SIM`, `WAITING_FOR_PC_B`, `READY`, `RUNNING`, `STOPPING`, `STOPPED`, `RESETTING`, `ERROR`, `EMERGENCY_STOPPED`를 제공한다. run/phase/job/progress, PC·기능별 readiness, error와 heartbeat가 포함되며 `/system/status`로 발행된다.

## 3. 액션

### `NavigateToTarget.action` — `/navigate_to_target`

Goal은 `job_id`, `target_type`, `target_id`, `waypoints`, `target_pose`, `enable_fine_alignment`다. Feedback은 phase, waypoint index/count, 남은 거리, position/yaw error와 retry count다. Result는 성공 여부, 최종 pose, 허용오차 충족 여부, error와 message다.

경유점이 있으면 `NavigateThroughPoses`, 없으면 `NavigateToPose`를 쓴다. `target_type=shelf_retreat`은 Nav2를 우회하고 제한된 직접 `cmd_vel` 정렬을 수행한다.

### `LoadTray.action` — `/load_tray`

Goal은 `job_id`, `tray_id`, feedback은 `phase`, `progress`, result는 `success`, `error_code`, `message`다. Bridge가 Isaac tray JSON 계약으로 변환한다.

### `DetectGraspPoint.action` — `/detect_grasp_point`

Goal의 `not_before`보다 오래된 영상은 사용할 수 없다. Result는 성공 여부, `GraspObservation`, 후보 수, error와 message다.

### `DetectTargetSlot.action` — `/detect_target_slot`

Goal에는 책의 width/height/thickness, Isaac 스윕의 `shelf_plane_y`, `shelf_floor_z`, 최신 영상 하한 `not_before`가 포함된다. Result는 성공 여부, `TargetSlot`, 후보 수, error와 message다.

### `PlaceBook.action` — `/place_book`

Goal에는 필드가 없다. Task Manager는 사이클 시작만 요청하며 manipulation이 perception을 호출해 책과 슬롯을 선택한다. Feedback은 `phase`, `progress`, result는 `success`, `failed_phase`, `placement_verified`, `error_code`, `message`다.

```text
SCANNING_SHELF → DETECTING_SLOT → ALIGNING_BASE → DETECTING_BOOK
→ PLANNING_GRASP → APPROACHING_BOOK → GRASPING
→ MOVING_TO_PRE_INSERT → INSERTING → RELEASING → RETREATING → VERIFYING
```

## 4. 서비스

| 서비스 | 형식 | 역할 |
| --- | --- | --- |
| `/system/start_cycle` | `std_srvs/Trigger` | 전체 상태가 READY일 때만 새 사이클 수락 |
| `/return_machine/publish_job` | `std_srvs/Trigger` | 설정된 한 개의 TrayJob 발행 |

웹 대시보드는 start service만 호출하고 Supervisor가 내부적으로 return machine service를 호출한다.

## 5. 주요 표준 ROS 인터페이스

| 이름 | 형식 | 사용처 |
| --- | --- | --- |
| `/rgb`, `/depth`, `/camera_info` | sensor_msgs | perception 입력 |
| `/lidar/points_raw`, `/scan` | PointCloud2, LaserScan | Isaac LiDAR와 Nav2 입력 |
| `/odom`, `/amcl_pose`, `/tf` | 표준 navigation | 위치 추정과 웹 위치 |
| `/cmd_vel_nav`, `/cmd_vel` | Twist | Nav2 입력과 collision monitor 이후 출력 |
| `/navigate_to_pose`, `/navigate_through_poses` | nav2_msgs action | navigation 내부 Nav2 호출 |

## 6. Isaac 내부 JSON 토픽

| 토픽 | 방향 | 역할 |
| --- | --- | --- |
| `/simulation/scenario/state` | Isaac → bridge | scenario state와 run_id |
| `/simulation/tray/command` | bridge → Isaac | 트레이 LOAD/CANCEL |
| `/simulation/tray/state` | Isaac → bridge | 트레이 phase와 결과 |
| `/manipulation/sim/command` | manipulation → Isaac | scan/place/cancel |
| `/manipulation/sim/state` | Isaac → manipulation | 스윕 관측과 조작 결과 |

## 7. 오류 코드

| 범위 | 실제 사용 |
| --- | --- |
| `1001` | 작업 계획 실패 |
| `2001`~`2007` | navigation server/reject/failure/result/TF/fine alignment/busy |
| `3001`~`3004` | tray bridge 오류. perception도 현재 `3002`~`3004`를 사용 |
| `4001`~`4004` | Task Manager가 본 manipulation 연결·결과 오류 |
| `401`~`412` | manipulation/Isaac의 IK, 경로, 속도, timeout, grasp, drop, insert, release, verify, invalid target, not ready, cancelled |

코드 범위가 완전히 분리되어 있지 않으므로 번호만 보지 말고 action, component, phase, message를 같이 기록한다.

## 8. QoS와 시간

- `/scenario/state`와 `/system/status`는 reliable + transient local이다.
- 카메라와 LaserScan은 sensor-data QoS를 사용한다.
- 작업 노드는 주로 simulation time, Supervisor와 웹은 system time을 사용한다.
- perception action의 `not_before`로 이동 전의 오래된 RGB-D 프레임 사용을 막는다.

## 9. 변경 절차

1. `.msg` 또는 `.action` 원본을 먼저 수정한다.
2. producer, consumer, launch/config 영향을 함께 검토한다.
3. 전체 build 후 양쪽 PC에 동일 commit을 배포한다.
4. `ros2 interface show`, action 준비와 실제 한 사이클을 확인한다.
5. 이 문서와 Runbook을 함께 갱신한다.
