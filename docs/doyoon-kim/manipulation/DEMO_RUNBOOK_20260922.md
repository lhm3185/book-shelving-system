# 시연 실행 순서 — 2026-09-22 교육장 (1차 통합 전체 사이클 1회)

보여줄 것: **주행 한 바퀴(32.04 m) → 비전 검출 → 파지 → 서가 반납**, 한 명령으로.

동결본: 브랜치 `feature/amr_patrol_pickplace`, 태그 `v1-integration-20260921`, 커밋 `ccf755a`.
집 데스크탑 검증: 주행 5판 중 5판, 기준선 3판 중 3판 성공.

---

## 0. 시연 전 준비 (GPU PC 에서, 약 5분)

GPU PC 의 저장소가 **동결본**이어야 한다. 야간 작업 브랜치가 아니다.

```bash
cd ~/b1_arm
git fetch origin
git checkout feature/amr_patrol_pickplace
git pull                                  # ccf755a 인지 확인
git log --oneline -1                      # → ccf755a
```

> **git-lfs 가 없다는 오류가 나면**: `GIT_LFS_SKIP_SMUDGE=1 git pull` 로 넘긴다.
> 모델 파일은 이미 실제 파일이라 받을 것이 없다.

빌드:

```bash
cd ~/b1_arm/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select shelving_interfaces shelving_perception shelving_manipulation
```

확인 (3개 다 있어야 한다):

```bash
ls ~/b1_arm/ros2_ws/install/shelving_perception/share/shelving_perception/resource/*.pt
ls ~/Desktop/assets/level/ing_library_env_v4.usd \
   || ls ~/b1_arm/simulation/assets/level/ing_library_env_v4.usd    # 둘 중 하나면 된다
ls ~/b1_arm/config/fastdds_local.xml
```

---

## 시연 — 터미널 1 하나면 된다

```bash
cd ~/b1_arm
DISPLAY=:0 ./scripts/demo/patrol_and_pick.sh --speed 0.6
```

`DISPLAY=:0` 은 ssh 로 들어와 돌릴 때 필요하다 (GPU PC 앞에서 직접 치면 없어도 된다).

| 단계 | 화면에 나오는 것 | 시간 |
| --- | --- | --- |
| 1/4 | Isaac 창이 뜨고 `준비 완료` | 30초~4분 |
| 2/4 | **비전 창**(`rqt_image_view`) — 노란 ROI · 초록 상자 · 빨간 점 | 15초 |
| 3/4 | 로봇이 도서관 테두리를 한 바퀴, 경유점마다 진행 상황 | 약 55초 |
| 4/4 | 돌아온 자리에서 파지 → 운반 → 서가 삽입 | 30초 |

성공하면 마지막 줄: **`끝. 한 바퀴 돌고 파지·반납까지 성공했다`**

끝내려면 `Ctrl+C` — 띄운 것을 전부(Isaac 포함) 내린다.

### 설명할 때 짚을 지점

- **3/4 에서 로봇이 회전하지 않는다.** Ridgeback 이 전방향이라 평행이동만 한다 —
  로봇팔과 트레이가 같은 방향을 유지해서, 돌아온 즉시 파지할 수 있다 (1단계 설계).
- **4/4 의 빨간 점**이 비전이 로봇팔에게 보내는 좌표다. 초록 상자는 가장 신뢰도 높은 책 하나.
- 노란 ROI 는 트레이 영역만 보게 자른 것 — 로봇팔 베이스나 바닥이 책으로 잡히는 것을 막는다.

---

## 막히면

| 증상 | 즉시 확인 | 대처 |
| --- | --- | --- |
| `Isaac 이 준비되지 않았다` | 그 PC **안에서** `ros2 topic list` | 비면 DDS 문제. `FASTRTPS_DEFAULT_PROFILES_FILE=~/b1_arm/config/fastdds_local.xml` |
| `비전 좌표가 20초 안에 오지 않았다` | `tail /tmp/b1_demo/vision.log` | 노드가 죽었는지. 모델 경로 오류면 `MODEL_PATH=` 로 지정 |
| 파지 x 스냅이 **10 mm 이상** | `grep 칸 중심에 맞췄 /tmp/b1_demo/manipulation.log` | 트레이가 설정 자리에서 벗어난 것 (정상은 0.3 mm) |
| 앞 실행이 안 죽어 토픽 충돌 | `pgrep -af run_simulation` | **`pkill -f` 금지** — PID 로 죽인다 |

### 시간이 없을 때 — 주행을 빼고 파지만

```bash
DISPLAY=:0 ./scripts/demo/patrol_and_pick.sh --no-patrol
```

파지·삽입만 30초 안에 끝난다. Isaac 이 이미 떠 있으면 `--keep-sim` 을 더한다.

---

## 시연 뒤 정리

```bash
# Ctrl+C 로 스크립트를 끄면 프로세스 그룹째 내려간다. 남았는지만 확인
pgrep -af "run_simulation|vision_manager|manipulation_node" | head
```

로그는 `/tmp/b1_demo/` 에 남는다 — 시연이 잘 됐든 아니든 **가져올 것**.

---

## 한 줄 요약 (물어보면)

> 주행은 AMR 담당이 오늘 Nav2 + 라이다 맵핑까지 끝낸 것을 넘겨받아 바꿀 예정이고,
> 지금 보여주는 주행은 **로봇팔 쪽에서 임시로 만든 순간이동 주행**이다.
> 그래도 파지·반납 구간은 실제 IK·물리로 도는 것이라 그대로 쓴다.
