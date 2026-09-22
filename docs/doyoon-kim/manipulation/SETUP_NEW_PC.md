# 새 PC 에 작업 환경 만들기 (로봇팔 파트)

> 대상: 우분투를 새로 깐 데스크탑에서 Isaac Sim 까지 돌리려는 경우.
> 무엇이 빠졌는지는 **`./scripts/setup_check.sh`** 가 한 번에 알려준다. 이 문서는 그걸 채우는 방법이다.

---

## 0. 왜 저장소만 받으면 안 되는가

| 종류 | 어디에 | 크기 |
| --- | --- | --- |
| 코드·설정·문서 | **git** | 작음 |
| **책 USD 6종·트레이·서가·바닥** | **git** (`simulation/assets/book_dataset/`) | 작음 |
| 비전 모델 `.pt` | **git LFS** (`git lfs pull`) | 6MB × 2 |
| **레벨 USD (통합 씬)** | **git 에 없다** — USB 나 `scp` | 130MB × N |
| **M0609 기술서·URDF** | **git 에 없다** (외부 소유) | 작음 |
| Isaac Sim 본체 | NVIDIA 에서 별도 설치 | 큼 |

> **2026-09-20 정정**: 책·트레이·서가는 **저장소에 들어 있다.** 그런데 스크립트가
> 개인 홈 경로(`~/book_dataset/...`)만 보고 있어서, 새로 클론한 PC 에서는
> **저장소에 있는 파일을 두고 "없다"** 가 됐다. 지금은 저장소 사본을 먼저 본다.

git 에 없는 것은 **통합 레벨 USD(용량)와 M0609 기술서·URDF(외부 소유)** 둘뿐이다.
**이 둘이 없으면 Isaac 을 깔아도 시뮬을 못 돌린다.**

---

## 1. 운영체제 — **Ubuntu 24.04**

| | |
| --- | --- |
| Isaac Sim 5.1 공식 지원 | Ubuntu 22.04 / 24.04, Windows 11 (Windows 10 미지원) |
| ROS 2 Jazzy 기준 | **Ubuntu 24.04** |

22.04 로 깔면 Isaac 은 되지만 Jazzy 를 소스 빌드해야 한다. **24.04 로 맞추면 둘 다 바이너리로 끝난다.**

Windows 에서 Isaac Sim 을 돌리는 선택지도 있으나, 우리 실행 경로가 전부 bash 스크립트 +
`ros2` CLI + FastDDS XML 이라 WSL2 를 끼워야 하고 GPU·DDS 가 WSL 경계를 넘으며 문제가 생긴다.
**우분투 네이티브를 권한다.**

---

## 2. 기본 도구

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions git git-lfs
git lfs install
```

ROS 2 Jazzy 는 공식 문서대로 설치한다 (`ros-jazzy-desktop` 권장).

---

## 3. Isaac Sim 5.1

NVIDIA 에서 받아 설치한 뒤, 기본 위치가 아니면 경로를 알려준다:

```bash
export ISAAC_SIM_PATH=/설치/경로        # 기본값은 ~/isaacsim
```

`$ISAAC_SIM_PATH/python.sh` 가 있어야 우리 스크립트가 동작한다.
GPU 드라이버는 Isaac Sim 5.1 요구사항을 따른다 (RTX 계열 필요).

---

## 4. 저장소

```bash
git clone <저장소> book-shelving-system
cd book-shelving-system
git checkout feature/robot_control
git lfs pull                              # 비전 모델 (안 하면 5MB 가 아니라 몇 백 바이트다)
cd ros2_ws && colcon build --symlink-install && cd ..
```

---

## 5. 시뮬레이션 자산

M0609의 Lula 입력은 저장소에 포함된다. 별도 강의 폴더나 다른 팀원 홈 디렉터리를
복사할 필요가 없다.

| 무엇 | 위치 |
| --- | --- |
| M0609 기술서 | `simulation/assets/cobot3_ws/isaacpjt/M0609/descriptor/m0609_description.yaml` |
| M0609 URDF | `simulation/assets/cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609.urdf` |
| 최종 레벨 | `simulation/assets/ing_library_env_v5-test.usd` |
| 결합 로봇 | `simulation/assets/Nova_Carter_ROS.usd` 및 하위 M0609/RG2 USD |

### M0609 기술서·URDF

Lula(IK)가 이 두 파일을 읽는다 (`robot_profiles.py` 의 `lula=("files", ...)`).
Franka 는 Isaac 내장 설정(`lula=("supported", "Franka")`)을 써서 파일이 필요 없지만,
M0609는 이 파일이 필요하다. USD 변환본은 형상·물리용이라 Lula 입력을 대신할 수 없다.
`robot_profiles.py`는 위 저장소 경로를 코드 파일 위치에서 계산하므로 프로젝트를 어디에
클론해도 동작한다.

없이도 되는 것 / 안 되는 것:

| | |
| --- | --- |
| 되는 것 | 설치, 빌드, `run_tests.sh`, **레벨 재생성**(`set_robot_yaw.py`, `place_robot_at_shelf.py` — Lula 를 안 쓴다) |
| 안 되는 것 | `check_reach.py`, 시뮬 실행 전체 (IK 가 필요한 모든 것) |

<<<<<<< HEAD
(책 USD·트레이·최종 레벨·결합 로봇·M0609 Lula 입력은 모두 저장소에 있으므로
별도 강의 폴더나 바탕화면 자산을 옮길 필요가 없다. USD와 텍스처는 Git LFS 대상이므로
새 PC에서는 반드시 `git lfs pull`을 실행한다.)
=======
> **할 일 (연구실에서)**: 이 두 파일은 작다. **저장소에 넣어** 다시는 이것 때문에
> 막히지 않게 하는 편이 낫다. doosan-robot2 는 공개 저장소이므로 라이선스만 확인하고 반영할 것.

(책 USD·트레이는 저장소에 있으므로 옮길 필요가 없다)

GPU PC 에서 가져오려면:

```bash
scp -r rokey@10.10.0.2:~/Desktop/Collected_ing_library_env_v5-firstFinal ~/Desktop/
scp -r rokey@10.10.0.2:~/Isaac_Sim_b-1 ~/
```

레벨은 **개당 130MB** 다.

### 원본 하나만 있으면 된다 — 나머지는 만들 수 있다

| 파일 | 어디서 | 쓰임 |
| --- | --- | --- |
| `ing_library_env_v5.usd` | **AMR 담당 원본** (USB·GPU PC) | 이것만 있으면 아래를 만든다 |
| `level_yaw0.usd` | **아래 명령으로 생성** | 파지 시연 (검증됨) |
| `level_shelf01.usd` | **아래 명령으로 생성** | 서가 삽입 (미해결) |

`level_*.usd` 는 우리가 원본에서 만든 파생본이다. 옮겨 오지 않았어도
Isaac 만 깔려 있으면 **저장소 도구로 재생성**된다.

```bash
./scripts/regenerate_levels.sh --dry-run   # 무엇을 할지 먼저 확인
./scripts/regenerate_levels.sh             # 실제 생성
```

사전 확인(Isaac·원본 레벨·참조 레이어)을 먼저 하고, 각 단계 산출물이 실제로 생겼는지
검사한 뒤 넘어간다. 서가와 선반 높이를 바꾸려면 `--shelf` / `--row-z` 를 쓴다.

아래는 그 스크립트가 실제로 부르는 명령이다 (직접 칠 일이 있을 때 참고).

```bash
cd ~/<저장소>
LV=~/Desktop/Collected_ing_library_env_v5-firstFinal

# ① 로봇 yaw 를 0 으로 — 경로 계산이 월드 축을 쓰기 때문 (임시 조치, 도구 주석 참조)
ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/set_robot_yaw.py \
  ./scripts/run_isaac_tool.sh --usd $LV/ing_library_env_v5.usd --out $LV/level_yaw0.usd --yaw 0

# ② 서가 앞으로 로봇 배치 (서가 삽입용). ①의 결과를 입력으로 쓴다
ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/place_robot_at_shelf.py \
  ./scripts/run_isaac_tool.sh --usd $LV/level_yaw0.usd --out $LV/level_shelf01.usd \
  --shelf /World/bookshelves/shelf_brown__book_shelf_01 --row-z 1.042
```

②는 자기검증(왕복 일치·오답 주입·0 가정 깨기)을 스스로 돌린다.
**`오차 0.0 mm`, `여유 +4.8 cm` 가 나와야 맞게 선 것이다.**
>>>>>>> origin/feature/amr_patrol_pickplace

---

## 6. DDS 네트워크 — **가장 많이 걸리는 함정**

`~/.ros/fastdds_whitelist.xml` 은 **연구실 유선망(10.10.0.x)만 허용**하도록 만든 파일이다.
집이나 다른 망에서 그대로 쓰면 **DDS 통신이 전부 막힌다.** 노드는 뜨는데 토픽이 안 보인다.

확인:

```bash
hostname -I                    # 이 PC 주소
grep address ~/.ros/fastdds_whitelist.xml
```

주소가 목록에 없으면 둘 중 하나:

```bash
# (A) 한 PC 안에서만 쓸 때 — 저장소의 빈 프로파일을 가리킨다
export FASTRTPS_DEFAULT_PROFILES_FILE=<저장소>/config/fastdds_local.xml

# (B) 여러 PC 를 쓸 때 — 이 PC 주소를 <interfaceWhiteList> 에 추가
```

**`unset` 으로는 안 된다.** 스크립트들이 `${FASTRTPS_DEFAULT_PROFILES_FILE:-~/.ros/fastdds_whitelist.xml}`
로 기본값을 주기 때문에, 지워 두면 도로 켜진다. 빈 문자열도 안 된다 — FastDDS 가
`realpath failed` 를 뱉고 `:-` 가 또 기본값으로 바꾼다. 그래서 **있지만 아무 것도 안 하는**
파일(`config/fastdds_local.xml`)을 가리킨다. 스크립트에서 기본값을 줄 때는 `:-` 가 아니라 `-` 를 쓴다.

### 화이트리스트는 **같은 PC 안**도 막는다

`useBuiltinTransports=false` 로 공유메모리·로컬호스트 전송이 꺼지기 때문이다.
2026-09-21 GPU PC 에서 Isaac 은 멀쩡히 돌고 **다른 PC 에서는 토픽이 보이는데**
정작 그 PC 안에서 `ros2 topic list` 하면 `/parameter_events`, `/rosout` 뿐이었다.
한 PC 에서 다 돌리는 시연은 반드시 (A) 로 할 것.

`./scripts/setup_check.sh` 가 이것도 같이 본다.

---

## 7. 확인

```bash
./scripts/setup_check.sh        # 빠진 것
./scripts/run_tests.sh          # 시뮬 54 + 로봇팔 46 (Isaac 없이 돈다)
```

Isaac 까지 왔으면 게이트 하나를 돌려 본다 — **팔이 좌표에 닿는지**만 IK 로 본다 (약 1분):

```bash
ARM_ROBOT=m0609 ISAAC_ENTRY=simulation/isaac/tools/check_reach.py \
  ./scripts/run_isaac_tool.sh --shelf-z 0.5097 --shelf-x-shift 0.04
```

`트레이 6/6   서가 4/4` 가 나오면 환경이 제대로 선 것이다.

그다음은 `DEMO_PATROL_PICK.md` (순회+파지 한 명령) 또는 `DEMO_20260921.md` 의 터미널 순서를 따른다.
**순회·파지 시연은 클론만 하면 돈다** — 레벨은 `~/Desktop` 에 없으면 저장소 사본으로 떨어지고,
YOLO 모델도 저장소에 들어 있다.
Isaac 을 이 PC 에서 직접 돌린다면 `SIM_HOST=local ./scripts/demo/sim_up.sh`.

---

## 8. 알아 둘 것

| | |
| --- | --- |
| `--start-home snap` 필수 | `move` 는 900스텝에서 매번 실패하고, 그 뒤 동작이 연쇄로 시간 초과된다 |
| 서가 콜라이더 | 16개 중 `shelf_brown__book_shelf_01` 하나만 물리적으로 존재한다 |
| 한 세션 6권 | 트레이 책은 Isaac 한 세션에 6권. 더 하려면 재시작 |
| 도메인 | 터미널·ssh 를 넘지 않는다. 팀 공통 `130` |
