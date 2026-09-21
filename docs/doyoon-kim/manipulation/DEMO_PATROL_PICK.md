# 순회 → 비전 → 파지 → 반납 (한 명령)

도서관 테두리를 따라 한 바퀴(32.04 m) 돈 뒤, 돌아온 자리에서 비전으로 책을 찾아
집어서 서가에 꽂는다. **Isaac·비전·로봇팔을 한 PC 에서** 돌린다.

## 터미널 1 — 이것 하나면 된다

```bash
cd ~/b1_arm                                   # 저장소 경로 (연구실 GPU PC 기준)
DISPLAY=:0 ./scripts/demo/patrol_and_pick.sh --speed 0.6
```

ssh 로 들어와 돌린다면 `DISPLAY=:0` 이 있어야 Isaac 창과 비전 창이 그 PC 화면에 뜬다.
끝내려면 `Ctrl+C` — 띄운 것을 전부(Isaac 포함) 내린다.

스크립트가 순서대로 하는 일:

| 단계 | 하는 일 | 걸리는 시간 |
| --- | --- | --- |
| 1/4 | Isaac 시작, 시뮬 실행기가 명령 토픽을 구독할 때까지 대기 | 30초~4분 |
| 2/4 | TF 두 개, `vision_manager`, `manipulation_node`, 검출 요청, `rqt_image_view` | 15초 |
| 3/4 | `library_loop` 한 바퀴 → 출발 자리 복귀 + **팔 기준 복귀 보정** | 약 32.04/속도 초 |
| 4/4 | 비전 좌표로 파지 → 운반 → 서가 삽입 → 배치 검증 | 30초 |

성공하면 마지막 줄이 `**끝. 한 바퀴 돌고 파지·반납까지 성공했다**` 다.

### 옵션

| 플래그 | 쓸 때 |
| --- | --- |
| `--keep-sim` | Isaac 이 이미 떠 있다. 4분을 아낀다 |
| `--no-patrol` | 주행 빼고 파지만 — **삽입이 깨졌을 때 주행 탓인지 가르는 기준선** |
| `--speed 0.6` | 주행 속도 m/s (기본 0.5) |
| `--goal-x` | 꽂을 칸 x (팔 기준) |
| `SIM_LEVEL=<경로>` | 레벨을 직접 지정 |

## 터미널 2 — 보면서 확인 (선택)

비전 창은 1번이 알아서 띄운다. 따로 보고 싶을 때만:

```bash
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/b1_arm/config/fastdds_local.xml
source /opt/ros/jazzy/setup.bash && source ~/b1_arm/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

노란 상자가 트레이 ROI, 초록 상자가 고른 책, 빨간 점이 로봇팔에 보내는 좌표다.

## 터미널 3 — 로그 (막혔을 때)

```bash
tail -f /tmp/b1_demo/run.log          # 전체 진행
tail -f /tmp/b1_demo/isaac.log        # 시뮬 쪽 판단 (파지/놓음/검증)
tail -f /tmp/b1_demo/manipulation.log # 목표 검사·좌표 보정
tail -f /tmp/b1_demo/vision.log       # 검출
```

## 집 PC 에서

**클론만 하면 돌아간다.** 레벨은 `~/Desktop/assets/level/` 에 없으면
저장소 사본(`simulation/assets/level/ing_library_env_v4.usd`)으로 떨어지고,
YOLO 모델 두 개도 저장소 안에 있다. Isaac Sim 5.1 만 따로 깔면 된다
(설치 전체는 `SETUP_NEW_PC.md`).

DDS 는 손대지 않아도 된다 — 스크립트가 `config/fastdds_local.xml`(빈 프로파일)을 쓴다.
**연구실 화이트리스트를 집에서 쓰면 통신이 전부 막힌다.**

## 막혔을 때

| 증상 | 뜻 | 볼 곳 |
| --- | --- | --- |
| `Isaac 이 준비되지 않았다` | 시뮬이 명령 토픽을 구독하지 못함 | 도메인·프로파일 파일. **그 PC 안에서** `ros2 topic list` 가 비면 화이트리스트 문제다 |
| `비전 좌표가 20초 안에 오지 않았다` | `vision_manager` 가 죽었다 | `vision.log` 끝. OpenCV/NumPy 판 문제면 `cvbridge` 계열 예외가 보인다 |
| `410 ... 칸 중심에서 N mm 어긋났다` | 비전 x 가 손가락 여유(8.3 mm) 밖 | 칸 중심 스냅이 도는지 `manipulation.log` 에서 `파지 x 를 칸 중심에 맞췄다` 확인 |
| `411 트레이 칸에 책 없음` | 찾는 좌표에 책이 없다 | `isaac.log` 가 **찾는 곳과 가장 가까운 책을 월드·팔 기준으로 같이 찍는다** |
| `406 운반 중 손 안에서 N cm 어긋남` | 책이 손에서 미끄러짐 | **미해결.** `--no-patrol` 로 돌려 주행 탓인지 먼저 가를 것 |
| `409 배치 확인 실패` (항목 전부 false) | 책이 엉뚱한 자세로 끝남 (대개 바닥) | `isaac.log` 의 `[놓음]` AABB 가 세워진 책 치수와 다르면 기울어진 채 놓인 것 |

### 알아 둘 함정

- **주행 뒤에는 팔 기준 좌표계가 틀어진다.** 루트는 웨이포인트에 정확히 서는데 팔 베이스가
  8.1 cm·4.37° 밀린다 (관절로 매달린 몸통이 루트를 그대로 따라오지 않는다).
  `navigation_executor` 가 도착 시 위치와 yaw 를 되돌린다.
  *알아채는 법*: 책들이 월드에서는 y 가 전부 같은데 **팔 기준 y 만 칸마다 벌어지면**
  책이 흐트러진 게 아니라 좌표계가 돌아간 것이다.
- **cv_bridge 는 OpenCV 5 / NumPy 2 PC 에서 못 쓴다** (KeyError / 세그폴트).
  `vision_manager` 는 자체 변환기를 쓴다 — 되돌리지 말 것.
- **디버그 영상 그리기가 노드를 죽이면 안 된다.** `_publish_debug_image` 의 try/except 를 빼지 말 것.
- 프로세스를 정리할 때 `pkill -f <패턴>` 을 **명령줄에 직접 쓰지 말 것**. 자기 명령줄이 걸려
  셸까지 죽는다. 패턴은 스크립트 파일 안에 두고 PID 로 죽인다.

## 남은 일

`406` 이 주행 뒤에만 난다. 복귀 보정 뒤에도 책이 기준보다 2.9 cm 남아 있어
`follow_tray` 가 보정을 따라오는지부터 확인해야 한다. 주행 없이는 `verified=True` 로 성공한다.
