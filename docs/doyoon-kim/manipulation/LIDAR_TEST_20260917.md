# 라이다 점검 — franka_camera.usd (AMR 담당 이동준 전달용)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17 20:10, D 김도윤 |
| 대상 | `franka_camera.usd` 의 `front_laser/Lidar` (Isaac `Example_Rotary`), 레벨 v5 |
| 증상 | 스캔맵이 잘 안 나옴 |
| 결론 | **원인 5가지 확인. 시험 실행에서만 보완하자 2D 스캔맵이 정상 생성됨.** USD 파일은 수정하지 않음 |

## 1. 원인 (실측)

| # | 원인 | 실측 | 영향 | 보완 (시험 실행에서만) |
| --- | --- | --- | --- | --- |
| 1 | **`fullScan` 꺼짐** | 메시지 1개 = 방위 약 30° 조각, 점 3,187개, **76 Hz** | SLAM 이 30° 조각을 한 바퀴 스캔으로 받아 지도가 조각남 | `LaserScanPublish.inputs:fullScan = True` → **한 바퀴 12.8~14.7만 점, 13.9 Hz** |
| 2 | **TF·odom 네임스페이스** | `/World/ridgeback_franka/tf`, `/World/ridgeback_franka/odom` 으로만 발행, 표준 `/tf`·`/odom` 비어 있음 | SLAM·Nav2 가 odom→base_link 를 못 받음 | TF·odom 노드 `nodeNamespace` 를 `""` |
| 3 | **TF 고리 2개** (`TFRobot` 이 로봇 전체 발행) | `base_link→panda_link2→panda_link1→panda_link0→arm_mount_link→base_link`, `base_link→dummy_base_y→dummy_base_x→world→odom→base_link` | tf2 "tree contains a loop" → **모든 변환 실패** | `TFRobot.targetPrims` = `[panda_link0, front_laser/Lidar]` (부모 base_link) |
| 4 | **`Lidar` 프레임이 TF 에 없음** | 포인트클라우드 frame_id `Lidar` | 스캔을 로봇 좌표로 못 옮김 | 3번 보완으로 `base_link → Lidar` (0.393, 0, 0.233) 발행 |
| 5 | **3D 32채널 라이다** (2D 아님) | z 값 32단, 아래 빔이 바닥을 찍어 **동심원 호** | 그대로 2D 로 바꾸면 바닥이 벽으로 지도에 찍힘 | `pointcloud_to_laserscan` 높이 필터 (라이다 기준 −0.18 ~ +0.77 m = 바닥 +5 cm ~ 1 m) |

추가로 알아둘 것:

| 항목 | 값 |
| --- | --- |
| 최소 거리 | **1.0 m** — 로봇 바로 앞 서가(약 0.45 m)는 안 잡힌다 |
| 가림 | 정지 상태 스캔에서 로봇 몸체·팔·트레이에 가려 방위의 절반 이상이 비어 있음 (`lidar_fullscan_topdown.png`) |
| `/clock` | 원본 USD 에 없음 → `use_sim_time:=true` 노드가 멈춤 (로봇팔 실행기가 보탬) |
| Jazzy `slam_toolbox` | **lifecycle 노드** — `configure`·`activate` 를 안 하면 `unconfigured` 로 조용히 대기, `/map` 안 나옴 |

## 2. 보완 후 결과

| 확인 | 결과 |
| --- | --- |
| TF | `world → odom → base_link → {Lidar, panda_link0 → 카메라}`, 고리 없음, `tf2_echo odom Lidar` 정상 |
| `/odom` | 83 Hz |
| `/point_cloud` | 한 바퀴, 13.8~13.9 Hz, 거리 1.0 ~ 11.4 m |
| `/scan` (변환) | 26 Hz |
| **스캔맵** | `slam_toolbox` 로 `map.pgm` 생성·저장 (148×271, 0.05 m). 벽·선반 직선 확인 (`map_x3.png`) — 로봇 정지 상태라 보이는 부채꼴만 |

## 3. 재현

```bash
# GPU PC — 보완 옵션 포함
ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --amr-test-overrides
# 이 PC
cd ~/ws_cobot_pjt/book-shelving-system/ros2_ws/src/shelving_manipulation/isaac_sim/tools
ROS_DOMAIN_ID=130 python3 lidar_check.py --seconds 10          # 주기·프레임·TF·위에서 본 그림
ROS_DOMAIN_ID=130 ./lidar_scanmap_test.sh 30                    # /scan 변환 + slam_toolbox + 지도 저장 (/tmp/b1_demo/scanmap)
```

## 4. USD 에 반영을 제안하는 값 (이동준 판단)

| 노드 | 속성 | 현재 | 제안 |
| --- | --- | --- | --- |
| `ros2_lidar_graph/LaserScanPublish` | `inputs:fullScan` | False | **True** |
| `ros2_odom_graph/TFWorld2Odom`, `TFOdom2Robot`, `TFRobot`, `PublisherOdometry` | `inputs:nodeNamespace` | `/World/ridgeback_franka` | **`""`** (또는 모든 노드가 같은 네임스페이스를 쓰도록 통일) |
| `ros2_odom_graph/TFRobot` | `inputs:targetPrims` | 로봇 루트 | **필요한 프레임만** (`panda_link0`, `front_laser/Lidar` 등) |
| (추가) | `ROS2PublishClock` | 없음 | 추가 |
| 카메라 그래프 | `frameId` | `sim_camera` (TF 에 없음) | TF 에 있는 프레임과 일치 (`INTEGRATION_TEST_20260917.md` 3절) |

원자료: `results/20260917_lidar/` (`lidar_partial_original_topdown.png` 보완 전 조각, `lidar_fullscan_topdown.png` 보완 후 한 바퀴 — 빨강은 라이다 높이 단면, `map.pgm/.yaml`, `map_x3.png`)
