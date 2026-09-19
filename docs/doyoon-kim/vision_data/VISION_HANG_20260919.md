# 비전 노드가 **첫 검출 요청 뒤 멈춥니다** — 원인 추적 결과 (로봇팔 → 비전 담당)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-19, D 김도윤 |
| 환경 | Isaac(GPU PC) + 이 PC 비전 노드, `ROS_DOMAIN_ID=130`, `vision` 브랜치 `477d0f6` |
| 결론 | 빈 칸 판정이 안 되는 것이 아니라, **노드가 첫 요청을 처리하다 빠져나오지 못합니다** |

이틀(9/18·9/19) 동안 "검출 요청에 반응이 없다" 로만 보이던 것의 정체입니다.
**코드가 틀렸다는 뜻이 아니라, 재현 조건과 증거를 정리해 드리는 것**입니다. 고칠 방법은 담당자가 정해 주세요.

## 1. 무엇이 관찰되는가

노드에 진단 로그를 넣고 재현했습니다.

```
[INFO] [진단] 동기 프레임 60개 수신, pending=False     ← 콜백 정상
[INFO] [진단] 검출 요청 수신 data=True                  ← 트리거 정상
[WARN] Could not transform slot frame arm_base_link to sim_camera:
       Requested time 691.25 ... 14.7초 뒤에 찍힘
(이후 로그 없음)
```

그 뒤 상태:

| 확인 | 결과 |
| --- | --- |
| `ros2 node list` | `/vision_manager` **살아 있음** |
| 프로세스 CPU | **91.3 %** (주 스레드 78.5 %), 상태 `R (running)` |
| 이후 동기 프레임 로그 | **더 이상 안 찍힘** (60프레임마다 찍게 해 두었는데도) |
| 두 번째 검출 요청 | **수신 로그조차 안 찍힘** |

→ 노드는 죽지 않았고, **CPU를 태우며 첫 처리에서 못 빠져나옵니다.** 그래서 이후 모든 요청이 무시됩니다.

## 2. 그 앞에 있는 두 번째 문제 — 첫 추론이 14.7초

```
트리거 수신    1789799587.36
TF 조회 실패   1789799602.06   ← 14.7초 뒤
```

추론 자체는 빠릅니다(이 PC CPU 기준 **0.13~0.15초**, 첫 호출만 1.75초).
그런데 **트리거에서 TF 조회까지 14.7초**가 걸립니다. 프레임 timestamp 로 TF 를 찾는데
그 시각이 이미 TF 버퍼(기본 10초)를 벗어나서 **반드시 실패**합니다.

```
Requested time 466.05 but the earliest data is at time 4xx.xx
```

이 PC 에는 GPU 가 없습니다. **담당자 PC(GPU)에서는 이 14.7초가 훨씬 짧아 안 보일 수 있습니다.**

## 3. 저희가 해 본 것 (제안이지 요구가 아닙니다)

우리 브랜치에서만 시험했고, `vision` 브랜치는 건드리지 않았습니다.

| # | 바꾼 것 | 결과 |
| --- | --- | --- |
| ① | rgb/depth/camera_info 구독을 **센서 QoS**(BEST_EFFORT, depth 1)로 | 프레임이 항상 최신이 됨 |
| ② | `max_frame_age`(1.0초)보다 묵은 프레임은 버리고 다음 프레임 대기 | 묵은 프레임 처리 사라짐 |
| ③ | **TF 실패 시 요청을 소모하지 않고 다음 프레임에서 재시도** | **판정 단계까지 도달** (debug 화면 발행 확인) |

③ 이 특히 컸습니다. 지금 코드는 `rgb_callback` 이 TF 조회 **전에** `pending_detection = False` 로
내려 두기 때문에, 기동 직후 한 번 실패하면 그 요청이 조용히 사라집니다.
(재시도 횟수는 30회로 막아 무한 재시도는 안 되게 했습니다.)

③ 까지 넣어도 **1번의 멈춤은 그대로**입니다.

## 4. 멈추는 지점을 좁히려면

`py-spy` 가 이 PC 에 설치가 안 돼(외부 관리 환경) 파이썬 스택을 못 떴습니다.
담당자 PC 에서 아래 한 줄이면 **정확히 어느 줄에서 도는지** 바로 나옵니다.

```bash
pip install py-spy            # 또는 pipx install py-spy
py-spy dump --pid $(pgrep -f vision_manager | tail -1)
```

처리 순서상 `_publish_debug_image` 까지는 갔습니다(화면이 발행됨). 그 뒤 구간을 보시면 됩니다.

## 5. 재현 방법

```bash
# GPU PC
./scripts/run_isaac_sim.sh --headless --book-variants mixed \
  --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color \
  --sensor-policy always

# 이 PC
ROS_DOMAIN_ID=130 ./simulation/isaac/tools/run_demo_pc.sh
ros2 topic pub --once /perception/detect_request std_msgs/msg/Bool "{data: true}"
# 두 번째 요청을 보내고 CPU 를 봅니다
top -p $(pgrep -f vision_manager | tail -1)
```

## 6. 덧 — 설정 경로 하나

`config/perception.yaml` 의 `shelf_model_path` 가 담당자 PC 경로(`/home/rokey/livrary_datas/...`)로
박혀 있어 다른 PC 에서는 노드가 조용히 실패합니다. 저희는 실행 스크립트에서 덮어쓰고 있습니다.
팀 공용으로 쓰려면 상대경로나 파라미터 기본값 쪽이 편할 것 같습니다.
