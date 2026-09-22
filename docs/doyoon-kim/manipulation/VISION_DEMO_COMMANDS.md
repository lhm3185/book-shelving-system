# 비전 작업용 실행 명령 — 최초 시연 그대로, 경로만 새 구조로

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-21, D 로봇팔 (김도윤) |
| 대상 | 비전 2인 |
| GPU PC | **10.10.0.1** |
| 도메인 | **129** |
| 로봇 | **Ridgeback-Franka** |
| 브랜치 | `feature/robot_control` |

> **비전 코드만 고쳐 끼울 수 있다.** `shelving_perception` 만 다시 빌드하고
> t2~t4 를 다시 띄우면 된다 — t1(Isaac)은 그대로 둬도 된다 (§4).

---

## 0. 왜 경로가 바뀌었나

9/18 에 시뮬 코드를 `ros2_ws/src/shelving_manipulation/isaac_sim/` 에서
저장소 최상위 `isaac_sim/` 으로 옮겼다 (`SIM_RESTRUCTURE_20260918.md`).
**파일은 그대로다** — `git mv` 로 옮겨 이력도 남아 있다.

| 최초 시연 때 | 지금 |
| --- | --- |
| `~/arm/isaac/run_demo_gpu.sh` | `scripts/run_isaac_sim.sh` (아래 t1 에 인자 그대로 풀어 뒀다) |
| `…/isaac_sim/tools/run_demo_pc.sh` | `isaac_sim/isaac/tools/run_demo_pc.sh` |
| `…/isaac_sim/tools/detect_view.py` | `isaac_sim/isaac/tools/detect_view.py` |
| `…/isaac_sim/tools/place_books.sh` | `isaac_sim/isaac/tools/place_books.sh` |

---

## 1. t1 — GPU PC (10.10.0.1) : Isaac

```bash
cd ~/b1_arm            # 저장소 작업본 (각자 clone 한 곳)
ROS_DOMAIN_ID=129 \
SIM_USD=$HOME/Desktop/ing_library_env_v5.usd \
  ./scripts/run_isaac_sim.sh --gui \
    --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color \
    --amr-test-overrides
```

`준비 완료 (step N) — 명령 대기` 가 뜰 때까지 기다린다 (약 4분).

> 옛 `run_demo_gpu.sh --amr-test-overrides` 와 **같은 동작**이다.
> 그 스크립트는 이미 `scripts/run_isaac_sim.sh` 로 넘기는 껍데기였고,
> 카메라 prim·USD·`--gui` 를 위처럼 채워 넣던 것뿐이다.

---

## 2. t2 — 이 PC : 정적 TF + 비전 노드 + 로봇팔 노드

```bash
cd ~/ws_cobot_pjt/book-shelving-system/isaac_sim/isaac/tools
ROS_DOMAIN_ID=129 ./run_demo_pc.sh
```

`Ctrl+C` 로 전부 종료된다. 로그는 `/tmp/b1_demo/` 에 쌓인다.

| 쓸 만한 환경변수 | |
| --- | --- |
| `VISION_EXTRA="-p show_debug_window:=true"` | 비전 CV 창 띄우기 |
| `VISION_CONF=0.6` | 임계값 바꾸기 (기본 0.75) |
| `MODEL_PATH=…` / `SHELF_MODEL=…` | 모델 직접 지정. 없으면 패키지 동봉본을 쓴다 |
| `ARM_ROBOT=m0609` | 6축을 다시 쓸 때만 |

> **`ARM_ROBOT` 기본값을 `franka` 로 되돌렸다** (오늘). `m0609` 로 두면 TF 별칭 부모가
> `base_link` 가 되어 `arm_base_link` 가 트리에 안 붙고, **비전의 모든 좌표 조회가
> 조용히 실패한다.**

---

## 3. t3 — 이 PC : 검출 화면

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/book-shelving-system/ros2_ws/install/setup.bash
python3 ~/ws_cobot_pjt/book-shelving-system/isaac_sim/isaac/tools/detect_view.py
```

창에서 **`d`** = 비전 노드에 검출 요청, **`s`** = 화면 저장, **`q`** = 종료.

- 창의 박스는 **이 도구가 같은 모델로 직접 돌린 결과** (눈으로 보기용)
- 오른쪽 위 글자는 **`vision_manager` 가 `/perception/books` 로 보낸 최근 좌표**
  (`arm_base_link` 기준)

두 개가 다르면 **모델은 맞는데 좌표 변환이 틀린 것**이다 — 거기서부터 보면 된다.

---

## 4. t4 — 이 PC : 책 넣기

```bash
cd ~/ws_cobot_pjt/book-shelving-system/isaac_sim/isaac/tools
ROS_DOMAIN_ID=129 ./place_books.sh 1        # 1권. 숫자를 바꾸면 그만큼
```

---

## 5. 비전 코드만 바꿔 끼우기

```bash
cd ~/ws_cobot_pjt/book-shelving-system/ros2_ws
colcon build --packages-select shelving_perception
# t2 를 Ctrl+C 로 끄고 다시:
cd ../isaac_sim/isaac/tools && ROS_DOMAIN_ID=129 ./run_demo_pc.sh
```

**t1(Isaac)은 끄지 않아도 된다.** Isaac 은 토픽만 내고 있어서, 이 PC 쪽 노드만
다시 띄우면 붙는다. 4분짜리 로드를 매번 기다릴 필요가 없다.

---

## 6. 막히면

| 증상 | 확인 |
| --- | --- |
| 토픽이 안 보임 | ① 도메인이 t1~t4 **전부 129** 인가 ② `ROS_LOCALHOST_ONLY` 를 **쓰지 않았는가** (화이트리스트와 배타적이라 통신이 아예 안 된다) |
| `화이트리스트에 없다` 경고 | 이 PC 주소가 `~/.ros/fastdds_whitelist.xml` 에 없다. 한 PC 안에서만 쓸 거면 `FASTRTPS_DEFAULT_PROFILES_FILE= ./run_demo_pc.sh` |
| 노드는 떴는데 좌표가 안 나옴 | `tail -f /tmp/b1_demo/vision.log` — 모델 경로로 죽는 경우가 많았다 |
| TF 조회 실패 | `use_sim_time:=true` 가 맞는지. 시각은 `/clock` 기준이다 |
| `/place_book` 결과가 뒤섞임 | `ros2 action info /place_book` → **Action servers: 1** 이어야 한다. 남은 노드가 먼저 응답한 적이 있다 |
| 깊이가 이상함 | 단위는 **m**, `32FC1` (mm 아님) |

---

## 7. 좌표 계약 (한 번 더)

```
1. 기준 프레임  arm_base_link      (base_link, sim_camera, world 아님)
2. 물체 기준점  AABB 중심          (에셋 원점 아님, 윗면 한 점 아님)
3. 축 규약      REP-103  +X 앞, +Y 왼쪽, +Z 위
4. 단위         m, rad
5. 변환 책임    발행하는 쪽이 변환해서 낸다. TF 로 한다
```

> **Franka 에서 `base_link` 는 Ridgeback 몸체다.** 팔 기준이 아니다.
> 9/20 에 비전 쪽이 `base_link` 를 직접 쓰게 바꾼 적이 있는데, 그건 M0609 때
> 별칭이 안 붙어서 생긴 우회였다. 지금은 `arm_base_link` 가 정상으로 붙는다.

새 흐름(빈 공간 + 파지점)의 계약 초안은 `PICKPLACE_CONTRACT_V2.md` 에 있다 —
**`top_center` 를 실제로 잴 수 있는지** 확인 부탁드린다.
