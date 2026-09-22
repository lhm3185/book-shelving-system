# 비전 파지 시연 — 실행 순서 (촬영용)

| 항목 | 값 |
| --- | --- |
| Isaac | **10.10.0.1** (이 PC 에서 ssh 로 켠다) |
| 비전·로봇팔 노드 | **이 PC** |
| 도메인 | **129** |
| 레벨 | `ing_library_env_v4.usd` (Franka 전용. v5 는 결합체) |
| 로봇 | Ridgeback-Franka |

> **촬영 포인트는 4번 한 줄이다.** 1~3 번은 준비이고, 1번은 **약 4분** 걸린다.

---

## 0. 먼저 — 남은 것 정리 (안 하면 액션 서버가 2개가 된다)

```bash
# 이 PC
pkill -f rqt_image_view ; pkill -f "run_demo_pc" ; pkill -f vision_manager
pkill -f manipulation_node ; pkill -f static_transform_publisher
[ -f /tmp/b1_demo/trigger.pid ] && kill "$(cat /tmp/b1_demo/trigger.pid)"
```

> `rqt_image_view` 는 **SIGTERM 을 씹는다.** 안 죽으면 `pkill -9 -f rqt_image_view`.
> `/place_book` 서버가 2개면 결과가 뒤섞인다 — `ros2 action info /place_book` 로 **1개**인지 확인.

---

## 1. Isaac 켜기 (10.10.0.1) — **약 4분**

```bash
ssh rokey@10.10.0.1 'cd ~/b1_arm && \
  ROS_DOMAIN_ID=129 SIM_USD=$HOME/Desktop/assets/level/ing_library_env_v4.usd \
  setsid nohup ./scripts/run_isaac_sim.sh --gui \
    --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color \
    --amr-test-overrides > /tmp/t1_vision.log 2>&1 < /dev/null & sleep 5; echo 기동'
```

**준비될 때까지 기다린다** (이 줄이 뜨면 끝):

```bash
ssh rokey@10.10.0.1 'until grep -aq "준비 완료" /tmp/t1_vision.log; do sleep 10; done; \
  grep -a "준비 완료" /tmp/t1_vision.log | tail -1'
# → ### 준비 완료 (step 802) — 명령 대기 /manipulation/sim/command
```

카메라가 나오는지 확인:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=129 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros2 topic list --no-daemon | grep -E "^/(rgb|depth|camera_info)$"
```

---

## 2. 비전·로봇팔 노드 (이 PC)

```bash
cd ~/ws_cobot_pjt/book-shelving-system/simulation/isaac/tools
ROS_DOMAIN_ID=129 ./run_demo_pc.sh
```

- 마지막 줄 `실행됨 (ROS_DOMAIN_ID=129)` 이 뜨면 **정상이다.** 그 뒤로 아무것도 안 나오는 게 맞다 (붙잡고 있는 중)
- 이 터미널은 **그대로 둔다.** `Ctrl+C` 로 전부 종료
- 팔 기준 프레임이 **`panda_link0 → arm_base_link`** 로 찍히는지 확인 (`base_link` 면 로봇 설정이 틀린 것)

---

## 3. 비전 창 + 검출 요청 (이 PC, 새 터미널 2개)

```bash
# 터미널 A — 검출 요청 (비전은 요청이 있을 때만 검출한다)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=129 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros2 topic pub -r 1 /perception/detect_request std_msgs/Bool "{data: true}"
```

```bash
# 터미널 B — 검출 화면
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=129 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

화면에서 볼 것:

| 색 | 뜻 |
| --- | --- |
| **노란 상자** | 트레이 ROI (팔 기준 3D 상자를 투영) |
| **초록 상자 + 빨간 점** | **신뢰도 1등 책.** 빨간 점이 로봇팔에 보내는 좌표 |
| 회색 얇은 상자 | ROI 밖이라 버린 책 |

---

## 4. 파지 — **여기가 촬영 시점**

```bash
cd ~/ws_cobot_pjt/book-shelving-system
source /opt/ros/jazzy/setup.bash; source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=129 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml

python3 simulation/isaac/tools/pick_from_vision.py
```

출력:

```
비전 좌표(윗면 중심)  frame=arm_base_link  (-0.4360, +0.0990, +0.1930)
→ 로봇팔이 쓸 AABB 중심  (-0.4360, +0.0990, +0.1120)   (책 폭 0.1631 의 절반을 뺀 값)
꽂을 곳  (-0.3497, +0.5495, +0.3399)
보냄 — 진행 상황:
  DETECTING_BOOK → PLANNING_GRASP → ... → VERIFYING
결과: success=True code=0 phase= verified=True
  배치 확인
```

**팔 동작은 10.10.0.1 모니터**에서, **검출 화면은 이 PC 창**에서 보인다.

### 자주 쓰는 변형

```bash
python3 ... /pick_from_vision.py --dry-run              # 좌표만 보고 안 보냄 (리허설용)
python3 ... /pick_from_vision.py --goal-x -0.4297       # 다른 칸에 꽂기
python3 ... /pick_from_vision.py --goal-x -0.5097
python3 ... /pick_from_vision.py --goal-x -0.2697
```

꽂을 칸 4곳 (팔 기준 x): **−0.3497 / −0.4297 / −0.5097 / −0.2697**

---

## 5. 막히면

| 증상 | 확인 |
| --- | --- |
| `비전 좌표가 20초 안에 오지 않았다` | 3번 터미널 A(검출 요청)가 돌고 있는가. 없으면 비전은 검출하지 않는다 |
| 카메라 토픽이 안 보임 | 도메인이 **전부 129** 인가. `ROS_LOCALHOST_ONLY` 를 **쓰지 않았는가** |
| `파지 좌표가 칸 중심에서 N mm 어긋났다` | **정상 거절이다.** 여유는 8.3 mm 뿐이라 그 이상이면 손가락이 옆 책에 닿는다 |
| `관측 높이가 책 규격과 다르다` | 비전이 윗면이 아닌 다른 점을 보냈다 |
| 결과가 뒤섞임 | `ros2 action info /place_book` → **Action servers: 1** 이어야 한다 |
| 팔 기준 프레임이 `base_link` | `ARM_ROBOT=franka` 인지 확인 (M0609 기본값이 남아 있었다) |

---

## 6. 끝나고

```bash
# 이 PC : 2번 터미널에서 Ctrl+C, 그리고
pkill -9 -f rqt_image_view

# 10.10.0.1 : PID 로만 끈다 (남의 프로세스를 건드리지 않는다)
ssh rokey@10.10.0.1 'pgrep -f "isaac/run_simulation"'
ssh rokey@10.10.0.1 'kill <위 PID>'
```
