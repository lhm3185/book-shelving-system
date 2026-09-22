# 비전팀 전달 — 픽앤플레이스 코드, 쓰는 파일과 순서

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-21, D 로봇팔 (김도윤) |
| 대상 | 비전 담당 2인 (빈 공간 인식 · 파지점 인식 구현 중) |
| 브랜치 | **`feature/robot_control`** (main 아님) |
| 로봇 | **Ridgeback-Franka** (M0609 는 보류) |

---

## 0. 먼저 — 어느 커밋을 쓸 것인가

| 커밋 | 상태 |
| --- | --- |
| **`2e93943`** | **Franka 픽앤플레이스가 마지막으로 검증된 상태** (9/20, 4권 중 2권 삽입 성공) |
| `733bf8b` 이후 | **전부 M0609 작업이다. Franka 로는 아직 안 돌려봤다** |

**비전 연동만 할 것이면 최신을 써도 된다** — 카메라·ROS 발행 쪽(`sensors/camera_bridge.py`,
`run_simulation.py` 의 카메라 인자)은 9/17 이후 **바뀌지 않았다.**

**로봇팔 동작까지 같이 볼 것이면** 아래 둘 중 하나로:

```bash
git checkout 2e93943                      # 검증된 상태 그대로
# 또는 최신에서 M0609 작업을 꺼두고
SIM_TRAY_FOLLOW=0 SIM_BRANCH_GUARD=0 ...
```

> 솔직하게: 최신 코드는 오늘 IK·홈·경로 쪽을 크게 고쳤고 **Franka 로 한 번도 안 돌렸다.**
> GPU PC 가 비는 대로 회귀부터 돌리고 결과를 알리겠다.

---

## 1. 쓰는 파일 (이것만 보면 된다)

### 실행

| 파일 | 하는 일 |
| --- | --- |
| `scripts/run_isaac_sim.sh` | Isaac 실행 껍데기. ROS 환경·USD 경로·트레이 경로를 맞춰 준다 |
| `isaac_sim/isaac/run_simulation.py` | 본체. USD 를 열고 장면을 만들고 ROS 를 띄운다 |
| `isaac_sim/isaac/world_loader.py` | USD 경로 결정 (`인자 > SIM_USD > 저장소 기본값`) |

### 카메라·ROS (비전이 실제로 볼 곳)

| 파일 | 하는 일 |
| --- | --- |
| **`isaac_sim/isaac/sensors/camera_bridge.py`** | 손목 카메라 생성 + ROS 그래프. **발행 토픽이 여기서 정해진다** |
| `isaac_sim/isaac/ros_bridge.py` | rclpy 노드·실행기 |

### 로봇팔 동작 (참고용 — 비전이 고칠 일은 없다)

| 파일 | 하는 일 |
| --- | --- |
| `isaac_sim/isaac/controllers/book_scene.py` | 장면·IK·경로 계획 |
| `isaac_sim/isaac/controllers/manipulation_executor.py` | `/manipulation/sim/command` 를 받아 실행, `/manipulation/sim/state` 발행 |
| `isaac_sim/isaac/config/robot_profiles.py` | 로봇별 prim 이름·프레임 (`FRANKA` / `M0609`) |
| `isaac_sim/isaac/config/arm.yaml` | Franka 자세·속도·허용오차 |

---

## 2. 발행되는 것 (계약)

`camera_bridge.py` 머리말 그대로다.

| 토픽 | 형식 | 비고 |
| --- | --- | --- |
| `/rgb` | `sensor_msgs/Image` (rgb8) | |
| `/depth` | `sensor_msgs/Image` (**32FC1, 단위 m**) | |
| `/camera_info` | `sensor_msgs/CameraInfo` | fx = fy = 317.2, 640×480, 가로 화각 90.5° |
| `/tf` | `panda_link0 → wrist_camera` | Isaac 이 발행 |
| `/clock` | sim time | **노드를 `use_sim_time:=true` 로 띄울 것** |

- 이미지 `frame_id` = **`wrist_camera_optical_frame`** (+Z 앞, +Y 아래 — 광학 규약)
- 카메라 위치: `panda_hand` 에서 손 x 축 **0.06 m**, 광축 = 그리퍼 접근축
- `--camera-ns /arm` 을 주면 `/arm/rgb` 처럼 앞에 붙는다 (AMR 카메라와 겹칠 때)

### 이 PC 에서 띄워야 하는 정적 TF 2개

Isaac 은 `panda_link0 → wrist_camera` 까지만 낸다. 나머지는 비전 쪽에서 잇는다.

```
panda_link0        → arm_base_link                 (항등)
wrist_camera       → wrist_camera_optical_frame    (항등)
```

> `arm_base_link` 가 **좌표 계약의 기준 프레임**이다. `panda_link0` 과 같은 것이지만
> 이름을 분리해 둔 이유는 로봇이 바뀌어도 계약이 안 바뀌게 하려는 것이다.

---

## 3. 실행 순서

### ① GPU PC — Isaac (약 4분)

```bash
cd ~/b1_arm                      # 또는 각자의 작업본
ROS_DOMAIN_ID=130 \
SIM_USD=<Franka 가 들어있는 레벨.usd> \
  ./scripts/run_isaac_sim.sh --camera --camera-hz 10 --sensor-policy always
```

- `--camera` : 레벨에 카메라가 없을 때 **손목 카메라를 만들어** 붙인다
- `--camera-prim <경로>` : 레벨에 이미 카메라가 있으면 이걸로 지정 (둘 중 하나만)
- `--sensor-policy always` : 작업 중에도 센서를 계속 켠다 (기본 `gated` 는 작업 중 끈다)
- `--gui` : 화면을 보고 싶을 때. **공용 PC 전체 화면 녹화는 하지 않는다**
- `준비 완료 (step N) — 명령 대기` 가 뜨면 준비 끝

### ② 이 PC — 수신 확인

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=130 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros2 topic list --no-daemon      # /rgb /depth /camera_info /tf /clock 이 보여야 한다
```

> **`ROS_LOCALHOST_ONLY` 는 쓰지 말 것.** 화이트리스트(`fastdds_whitelist.xml`)가
> UDP 를 `10.10.0.x` 인터페이스로만 묶고 내장 전송을 꺼 두기 때문에, 둘을 같이 주면
> **discovery 가 아예 안 된다.** 오늘 확인했다 (v21 §2-9).

### ③ 이 PC — 정적 TF 2개 + 비전 노드

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 panda_link0 arm_base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 wrist_camera wrist_camera_optical_frame &
ros2 run shelving_perception vision_manager --ros-args -p use_sim_time:=true
```

### ④ 로봇팔에 명령 (동작까지 볼 때)

```bash
./scripts/demo/status.sh          # 액션 서버 확인
./scripts/demo/pick.sh            # 트레이 책 파지
```

`ros2 topic pub` 으로 직접 JSON 을 보내는 것은 **되지 않는다** — YAML 파서가 따옴표를
먹는다. 꼭 필요하면 rclpy 로 보낼 것.

---

## 4. 좌표 계약 (꼭 지킬 것)

`COORDINATE_CONTRACT.md` 다섯 줄 그대로다.

```
1. 기준 프레임  arm_base_link      (base_link, sim_camera, world 아님)
2. 물체 기준점  AABB 중심          (에셋 원점 아님, 윗면 한 점 아님)
3. 축 규약      REP-103  +X 앞, +Y 왼쪽, +Z 위
4. 단위         m, rad
5. 변환 책임    발행하는 쪽이 변환해서 낸다. TF 로 한다
```

로봇팔은 `frame_id != arm_base_link` 면 **M410 으로 거절**한다.

**2번이 특히 중요하다.** 9/17 에 "박스 중심 픽셀의 깊이(=윗면)" ↔ "AABB 중심" 불일치로
**8 cm**, 박스 중심 픽셀이 책 사이 틈에 떨어져 바닥 깊이를 읽어 **0.67 m** 가 어긋난 적이 있다.

> **새 흐름(빈 공간 + 파지점)의 계약은 아직 확정 전이다.** 파지점을 "AABB 중심 + 치수" 로
> 줄지, "보이는 윗면 중심 + 높이" 로 줄지 정해야 한다. 비전 쪽에서 **실제로 잴 수 있는 것**이
> 무엇인지 알려주면 거기에 맞추겠다 — 못 재는 것을 계약으로 정하면 추정값이 섞여 더 위험하다.

---

## 5. 알려진 것 (미리 알고 시작하면 좋은 것)

| 항목 | 내용 |
| --- | --- |
| 손목 카메라 화면 | 가운데에 **손가락과 그 그림자**가 걸린다. `docs/doyoon-kim/manipulation/img/wrist_home_rgb.jpg` |
| 홈 자세 | 트레이 위 **30 cm** 에서 내려다본다 — 트레이 관측용 |
| **책장 관측 자세** | **아직 없다.** 새 흐름에 필요해서 지금 설계 중 |
| 그리퍼가 화면에 안 보임 | 비주얼 에셋 참조가 깨진 것. **물리는 정상**이다 |
| 센서를 켜면 | Isaac 이 느려진다. `--sensor-policy always` 는 그걸 감수하는 설정이다 |

---

## 6. 막히면

| 증상 | 확인 |
| --- | --- |
| 토픽이 안 보임 | ① 도메인이 양쪽 같은가 ② `ROS_LOCALHOST_ONLY` 를 껐는가 ③ 화이트리스트 파일 경로 |
| TF 조회 실패 | `use_sim_time:=true` 를 줬는가 (`/clock` 기준으로 맞춰야 한다) |
| 깊이가 이상함 | 단위가 **m** 다 (mm 아님). `32FC1` |
| 좌표가 0.8 m 씩 틀림 | 광학 프레임 180° 를 **두 번** 적용한 것 — 9/17 사고 #4 |

무엇이든 막히면 바로 알려주세요. 계약 쪽은 제가 맞추겠습니다.
