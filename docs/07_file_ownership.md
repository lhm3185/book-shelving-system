# 폴더·파일별 담당자와 협업 경계

## 1. 담당자 요약

| 담당자 | 주 책임 | 주 경로 |
| --- | --- | --- |
| A 이현민 | FSM·통합·공통 계약·관제 | `shelving_system`, `shelving_interfaces`, `shelving_web` |
| B 윤재민 | RGB-D/YOLO 비전 | `shelving_perception` |
| C 이동준 | Nav2·AMR·PC B 주행 | `shelving_navigation` |
| D 김도윤 | 로봇팔·그리퍼·Isaac 조작 | `shelving_manipulation`, `isaac_sim/lib/controllers` |

소유권은 다른 사람의 수정을 금지한다는 뜻이 아니라 최종 판단과 검토 책임을 뜻한다.

## 2. A 이현민 — 시스템 통합

### `shelving_interfaces`

| 경로 | 책임 |
| --- | --- |
| `msg/*.msg`, `action/*.action` | 공통 ROS 계약 |
| `CMakeLists.txt`, `package.xml` | rosidl 생성과 의존성 |

인터페이스 변경은 해당 producer/consumer 담당자가 반드시 함께 검토한다.

### `shelving_system`

| 파일 | 현재 책임 |
| --- | --- |
| `task_manager_node.py` | TrayJob부터 HOME 복귀까지 action 순서와 상태 취합 |
| `state_machine.py` | 실제 top-level FSM 상태와 전이 |
| `job_planner.py` | 분류코드별 서가와 책 작업 계획 |
| `yaml_data_manager.py` | config 읽기·검증. 현재 점유 상태 영속 저장은 하지 않음 |
| `return_machine_node.py` | 기본 한 권 TrayJob 생성과 publish service |
| `simulation_bridge_node.py` | scenario/초기 pose/tray JSON과 ROS 계약 변환 |
| `system_supervisor_node.py` | 3-PC readiness, SystemStatus, start gate |
| `mock_manipulation_server.py` | navigation-test 모드의 mock PlaceBook |
| `config/system.yaml` | timeout, topic/action, frame 정책 |
| `config/shelf_map.yaml` | HOME, 반납기, 서가 pose와 경유점 |
| `config/book_profiles.yaml` | 작업 계획용 책 규격 |
| `launch/pc_a.launch.py` | PC A ROS 노드 |
| `launch/pc_b.launch.py` | PC B 기능 묶음 |
| `launch/all.launch.py` | 한 PC navigation-test/full 통합 |

### `shelving_web`

| 파일 | 책임 |
| --- | --- |
| `web_gateway_node.py` | SystemStatus/AMCL을 HTTP·WebSocket으로 제공, start service 호출 |
| `static/index.html`, `app.js`, `styles.css` | 운영자 대시보드 |
| `launch/pc_c.launch.py` | PC C 포트·바인드 설정 |

## 3. B 윤재민 — 비전

| 파일 | 현재 책임 |
| --- | --- |
| `shelving_perception/vision_manager.py` | RGB-D 동기화, 두 perception action server, TF와 결과 발행 |
| `target_detector.py` | 서가/빈 슬롯 후보 처리 |
| `book_detector.py` | 트레이 책 후보 처리 |
| `config/perception.yaml` | camera topic/frame, ROI, confidence, depth와 삽입 설정 |
| `resource/best.pt` | 서가/슬롯 관련 YOLO 모델 |
| `resource/book_tray_best.pt` | 트레이 책 YOLO 모델 |
| `test/*` | 영상 변환과 lint 시험 |

카메라 prim, optical frame, ROI와 모델 변경은 C·D와 실제 스윕 자세에서 함께 확인한다.

## 4. C 이동준 — AMR

| 파일 | 현재 책임 |
| --- | --- |
| `navigation_node.py` | `/navigate_to_target` action server |
| `navigation_controller.py` | Nav2 action 선택, retry, 정밀 정렬, 직접 shelf retreat |
| `launch/navigation.launch.py` | map/AMCL/Nav2/LiDAR 변환 전체 기동 |
| `config/nav2_ridgeback_franka.yaml` | planner/controller/costmap/collision monitor |
| `config/navigation.yaml` | 허용오차, 속도, retry와 frame |
| `config/pointcloud_to_laserscan.yaml` | PointCloud2→LaserScan |
| `maps/library_map.yaml`, `library_map.png` | localization/navigation map |
| `scripts/ros/run_pc_b.sh` | PC B ROS·RViz one-command 실행 |

YAML 서가 pose는 A와, 슬롯 횡정렬 목표는 D와, 카메라 시야는 B와 함께 검토한다.

## 5. D 김도윤 — 로봇팔·Isaac 조작

### ROS 조정 계층

| 파일 | 현재 책임 |
| --- | --- |
| `manipulation_node.py` | PlaceBook, 스윕, perception/nav 호출, Isaac command/state 연결 |
| `grasp_planner.py` | 책/슬롯 검증, tray slot 선택과 command 생성 |
| `book_placer.py` | executor 상태 추적, phase/progress/error 변환 |
| `config/manipulation.yaml` | action, alignment, timeout, 유효 영역 |
| `config/book_profiles.yaml` | 조작용 책·tray slot 교정값 |
| `test/*` | 계약, planner, placer, node 시험 |

### Isaac 실행 계층

| 파일 | 현재 책임 |
| --- | --- |
| `isaac_sim/lib/controllers/manipulation_executor.py` | scan/place/cancel 명령 실행 |
| `book_scene.py` | 책·트레이·서가 scene, 경로와 배치 검증 |
| `arm_kinematics.py`, `arm_planning.py`, `arm_primitives.py` | FK/IK와 안전 경로 |
| `arm_geometry.py`, `arm_errors.py`, `shelf_gap.py` | 기하·오류·빈 공간 정책 |
| `isaac_sim/config/arm.yaml` | pose, 속도, gripper, 삽입과 오차 |

## 6. 공통 Isaac·실행 파일

| 경로 | 주 담당 | 필수 검토 |
| --- | --- | --- |
| `isaac_sim/run_simulation.py` | A·D | 센서 B, base C |
| `isaac_sim/lib/scenario_runtime.py` | A | C·D |
| `isaac_sim/lib/tray_runtime.py` | A·D | C |
| `isaac_sim/lib/ros_bridge.py` | A | B·C·D |
| `isaac_sim/lib/sensors/camera_bridge.py` | B | D |
| `isaac_sim/config/scenes.yaml` | A | B·C·D |
| `isaac_sim/assets/robots/franka_nav.usda` | C·D | B |
| `isaac_sim/assets/worlds/library/*` | A·C | B·D |
| `config/ros_network.env` | A·C | 전원 |
| `scripts/emergency_stop.sh` | A·C | 전원 |

## 7. 실행 스크립트 소유권

| 파일 | 주 담당 | 기능 |
| --- | --- | --- |
| `scripts/ros/build.sh` | A | workspace build |
| `scripts/ros/run_pc_a.sh` | A | Isaac+PC A 기동과 PC B 대기 |
| `scripts/ros/run_pc_b.sh` | C | navigation/perception/manipulation/RViz |
| `scripts/ros/run_pc_c.sh` | A | 웹 관제 |
| `scripts/run.sh` | A·D | 한 PC navigation-test/full |
| `scripts/tests/run.sh` | D, A 취합 | 현재 sim+manipulation 자동시험 |

## 8. 충돌 가능성이 큰 변경

| 변경 | 최종 반영 | 함께 확인할 사람 |
| --- | --- | --- |
| msg/action 필드 | A | 모든 producer/consumer 담당자 |
| `pc_b.launch.py` | A | B·C·D |
| camera frame/prim | B | C·D |
| shelf observation pose | C | A·B·D |
| manipulation alignment 목표 | D | B·C |
| robot USD/articulation | C·D | B |
| network/domain/DDS | A·C | 전원 |

## 9. 완료의 정의

1. 관련 package와 전체 workspace가 build된다.
2. 정상 입력과 대표 실패 입력을 시험한다.
3. action/message/frame 변경을 producer와 consumer가 함께 확인한다.
4. 라이브 시연에 필요한 설정을 YAML과 launch에 반영한다.
5. 현재 동작과 제한을 문서에 업데이트한다.
6. 공통 파일은 관련 담당자가 검토한다.
7. 기존 한 권 정상 플로우를 깨뜨리지 않는다.
