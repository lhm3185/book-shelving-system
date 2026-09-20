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

## 5. git 에 없는 자산 — 이 경로 그대로 둬야 한다

경로는 `robot_profiles.py` 와 스크립트 기본값이다. 다른 곳에 두려면 환경변수로 알려줘야 한다.

| 무엇 | 놓을 곳 |
| --- | --- |
| M0609 기술서 | `~/Isaac_Sim_b-1/src_pra/M0609/descriptor/m0609_description.yaml` |
| M0609 URDF | `~/Isaac_Sim_b-1/src_pra/M0609/doosan-robot2/urdf/m0609.urdf` |
| 레벨 USD 폴더 | `~/Desktop/Collected_ing_library_env_v5-firstFinal/` |

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
# (A) 한 PC 안에서만 쓸 때 — 화이트리스트를 쓰지 않는다
unset FASTRTPS_DEFAULT_PROFILES_FILE

# (B) 여러 PC 를 쓸 때 — 이 PC 주소를 <interfaceWhiteList> 에 추가
```

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
  SIM_USD=~/Desktop/Collected_ing_library_env_v5-firstFinal/level_shelf01.usd \
  ./scripts/run_isaac_tool.sh --shelf-z 0.5097 --shelf-x-shift 0.04
```

`트레이 6/6   서가 4/4` 가 나오면 환경이 제대로 선 것이다.

그다음은 `DEMO_20260921.md` 의 터미널 순서를 따른다.
Isaac 을 이 PC 에서 직접 돌린다면 `SIM_HOST=local ./scripts/demo/sim_up.sh`.

---

## 8. 알아 둘 것

| | |
| --- | --- |
| `--start-home snap` 필수 | `move` 는 900스텝에서 매번 실패하고, 그 뒤 동작이 연쇄로 시간 초과된다 |
| 서가 콜라이더 | 16개 중 `shelf_brown__book_shelf_01` 하나만 물리적으로 존재한다 |
| 한 세션 6권 | 트레이 책은 Isaac 한 세션에 6권. 더 하려면 재시작 |
| 도메인 | 터미널·ssh 를 넘지 않는다. 팀 공통 `130` |
