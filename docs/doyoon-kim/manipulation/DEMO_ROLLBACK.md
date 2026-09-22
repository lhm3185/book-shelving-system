# 시연 롤백 지점 — 되돌리는 방법 (실제로 해 보고 적음)

> **롤백은 시연 시작 전에만 쓴다.** 되돌리기 + 재시작 + 워밍업에 최소 몇 분이 걸리므로
> **시연 도중에는 쓸 수 없다.** 시작 전이면 되돌리고, 시작한 뒤면 그대로 간다.
> 당일에 "롤백할까" 를 고민하며 시간을 쓰는 것이 가장 나쁘다.

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-18 저녁, D 김도윤 |
| 왜 | 9/18 에 Isaac 실행 구조를 `isaac_sim/` 으로 옮겼다. 문제가 생기면 **구조 변경 이전으로** 되돌려 시연한다 |
| 확인 | 아래 절차를 GPU PC 에서 **실제로 실행해 파지 1/1 성공까지 확인**했다 (적어두기만 한 절차가 아니다) |

## 1. 롤백 지점

| 대상 | 커밋 | 확인된 것 |
| --- | --- | --- |
| `feature/robot_control` | **`d1db708`** | 로봇팔 단독 파지 4/4·2/2, 여러 종류 책, 관측 자세 35/48 |
| `test/fsm-arm-integration` (로컬) | **`c8a07c8`** | **FSM 4권 4/4 관통** (mock 칸 4곳 순환 포함) |
| 구조 변경 시작 | `b9586b7` | 이 커밋부터 `isaac_sim/` 실행기 |

**주의**: 예전 실행 경로(`run_place_book_server.sh`, `run_demo_gpu.sh`)는 지금 **호환용 껍데기**라, 그대로 부르면 **새 코드**가 돈다. 진짜 롤백은 git 체크아웃뿐이다.

## 2. 되돌리는 절차 (GPU PC)

```bash
# 1) 저장소를 롤백 지점으로
cd ~/book-shelving-system
git checkout d1db708

# 2) 예전 실행 경로는 ~/arm 사본을 쓴다 — 사본도 그 시점으로 되돌린다  ★ 빠뜨리기 쉬움
rsync -a --delete --exclude=__pycache__ --exclude=/docs --exclude=/results \
      ros2_ws/src/shelving_manipulation/isaac_sim/ ~/arm/
rsync -a --exclude=__pycache__ \
      ros2_ws/src/shelving_manipulation/shelving_manipulation/ ~/shelving_manipulation_py/shelving_manipulation/

# 3) 실행 (예전 방식)
ROS_DOMAIN_ID=130 ~/arm/isaac/run_place_book_server.sh \
    --usd ~/Desktop/ing_library_env_v5.usd --book-variants mixed
#   시연처럼 카메라까지: ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --book-variants mixed
```

이 PC 쪽(`manipulation_node`, `place_books.sh`)은 그대로 쓰면 된다.

**2번을 빠뜨리면** `place_book_server.py 를 찾을 수 없다` 로 실패한다 (오늘 실제로 겪었다). 오늘 구조 변경 뒤 동기화로 `~/arm` 에서 그 파일이 사라졌기 때문이다.

## 3. 되돌린 뒤 원상 복귀

```bash
cd ~/book-shelving-system && git checkout feature/robot_control
```

새 구조는 `~/arm` 사본이 필요 없다. 저장소에서 바로 실행한다.

```bash
SIM_USD=~/Desktop/ing_library_env_v5.usd ./scripts/run_isaac_sim.sh --gui --book-variants mixed
```

## 4. 9/19 에 판정할 것

| 순서 | 항목 | 통과 기준 |
| --- | --- | --- |
| 1 | **새 실행기로 FSM 4권 관통** (아직 미확인 — 4/4 는 구조 변경 이전 값이다) | 4/4 |
| 2 | 반복 3회 | 간헐 실패 없음 |
| 3 | 비전 켠 채 4권 길이 측정 | 3분 이하 |

셋 다 통과하면 **새 경로로 시연**, 하나라도 실패하면 **1절 롤백 지점으로 체크아웃해 그 상태로 시연**한다. 판정은 **9/19 저녁**에 내린다.

전제: **GPU PC 사용 시간을 비전 담당과 나눠야 한다** (토요일에 학습이 돌고 있으면 위 시험을 못 한다).
