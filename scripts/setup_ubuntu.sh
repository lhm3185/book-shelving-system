#!/usr/bin/env bash
# 새로 깐 Ubuntu 24.04 에 B-1 로봇팔 작업 환경을 만든다.
#
#   ./scripts/setup_ubuntu.sh --dry-run    # 무엇을 할지 보기만 한다 (권장: 먼저 이것부터)
#   ./scripts/setup_ubuntu.sh              # 실제 설치
#   ./scripts/setup_ubuntu.sh ros          # 한 단계만
#
# 단계: base | ros | py | ws   (인자 없으면 전부)
#
# **Isaac Sim 은 여기서 안 깐다.** NVIDIA 로그인이 필요한 수동 내려받기라, 설치 여부만 확인하고
# 안내한다. 나머지(ROS 2 Jazzy, 빌드 도구, 파이썬 의존, 워크스페이스 빌드)는 자동이다.
#
# sudo 를 쓰는 곳은 실행 전에 화면에 그대로 찍는다. 되돌릴 수 없는 일은 하지 않는다.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRY=0
STEPS=()
for a in "$@"; do
    case "$a" in
        --dry-run|-n) DRY=1 ;;
        base|ros|py|ws) STEPS+=("$a") ;;
        *) echo "모르는 인자: $a  (base | ros | py | ws | --dry-run)"; exit 2 ;;
    esac
done
[ "${#STEPS[@]}" -gt 0 ] || STEPS=(base ros py ws)

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
note() { printf '   %s\n' "$*"; }
run()  {
    printf '   $ %s\n' "$*"
    [ "$DRY" -eq 1 ] && return 0
    "$@" || { echo "   **실패**: $*"; return 1; }
}
has()  { command -v "$1" >/dev/null 2>&1; }

# --------------------------------------------------------------- 사전 확인
say "0. 이 PC 확인"
. /etc/os-release 2>/dev/null || true
note "OS      ${PRETTY_NAME:-알 수 없음}"
note "커널    $(uname -r)"
note "아키텍처 $(uname -m)"
if [ "${VERSION_ID:-}" != "24.04" ]; then
    note ""
    note "**Ubuntu 24.04 가 아니다.** ROS 2 Jazzy 는 24.04 기준이라 그대로는 안 깔린다."
    note "계속하려면 Ctrl+C 로 멈추고 상황을 먼저 확인할 것."
    [ "$DRY" -eq 1 ] || exit 1
fi
has nvidia-smi && nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader \
    | sed 's/^/   GPU     /' || note "GPU     nvidia-smi 없음 — 드라이버부터 설치할 것"

# --------------------------------------------------------------- base
if [[ " ${STEPS[*]} " == *" base "* ]]; then
    say "1. 기본 도구"
    run sudo apt-get update
    # terminator: 창 하나를 4분할로 쓴다. 시연 때 터미널 A~D 를 동시에 봐야 해서 편하다
    run sudo apt-get install -y \
        git git-lfs curl gnupg lsb-release terminator \
        python3-pip python3-colcon-common-extensions python3-rosdep
    run git lfs install
fi

# --------------------------------------------------------------- ros
if [[ " ${STEPS[*]} " == *" ros "* ]]; then
    say "2. ROS 2 Jazzy"
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        note "이미 설치돼 있다 — 건너뛴다"
    else
        note "ROS 2 apt 저장소를 등록하고 ros-jazzy-desktop 을 설치한다"
        run sudo install -d -m 0755 /usr/share/keyrings
        if [ "$DRY" -eq 0 ]; then
            curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
                | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg || exit 1
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
                | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null || exit 1
        else
            note '$ curl … ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg'
            note '$ echo "deb … packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list'
        fi
        run sudo apt-get update
        run sudo apt-get install -y ros-jazzy-desktop ros-jazzy-tf2-tools
    fi
    # rosdep 은 이미 초기화돼 있으면 에러를 내므로 한 번만
    if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
        run sudo rosdep init
    else
        note "rosdep 이미 초기화됨"
    fi
    run rosdep update
    say "2-1. 셸 설정"
    if grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
        note ".bashrc 에 이미 있다"
    else
        note "**자동으로 넣지 않는다.** Isaac 은 자체 파이썬을 쓰는데 터미널에 ROS 가 source 돼"
        note "있으면 Isaac 이 그 rclpy 를 먼저 import 하다 죽는다 (2026-09-17 실측)."
        note "ROS 를 쓰는 터미널에서만 직접 치는 쪽을 권한다:"
        note "    source /opt/ros/jazzy/setup.bash"
    fi
fi

# --------------------------------------------------------------- py
if [[ " ${STEPS[*]} " == *" py "* ]]; then
    say "3. 파이썬 의존 (비전 노드용)"
    note "Ubuntu 24.04 는 시스템 파이썬이 PEP668 로 잠겨 있어 --break-system-packages 가 필요하다."
    note "--user 로 ~/.local 에 깔아 시스템 패키지를 건드리지 않는다 (랩탑도 같은 방식)."
    run python3 -m pip install --user --break-system-packages \
        torch --index-url https://download.pytorch.org/whl/cu130
    run python3 -m pip install --user --break-system-packages ultralytics
    if [ "$DRY" -eq 0 ]; then
        python3 - <<'PY' || true
import torch, ultralytics
print(f"   torch {torch.__version__} / cuda 사용가능 {torch.cuda.is_available()}")
print(f"   ultralytics {ultralytics.__version__}")
PY
        note "cuda 사용가능 이 False 면 드라이버나 휠 종류를 확인할 것 (GPU 가 있는 PC 기준)"
    fi
fi

# --------------------------------------------------------------- ws
if [[ " ${STEPS[*]} " == *" ws "* ]]; then
    say "4. 워크스페이스 빌드"
    if [ ! -f /opt/ros/jazzy/setup.bash ]; then
        note "ROS 가 없어 건너뛴다. 먼저 'ros' 단계를 돌릴 것"
    else
        note "LFS 모델부터 받는다 (안 받으면 몇 백 바이트 포인터라 비전 노드가 죽는다)"
        run git -C "$REPO_ROOT" lfs pull
        if [ "$DRY" -eq 0 ]; then
            set +u; . /opt/ros/jazzy/setup.bash; set -u
            ( cd "$REPO_ROOT/ros2_ws" \
              && rosdep install --from-paths src --ignore-src -y \
              && colcon build --symlink-install ) || echo "   **빌드 실패**"
        else
            note '$ cd ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install'
        fi
    fi
fi

# --------------------------------------------------------------- Isaac 안내
say "5. Isaac Sim 5.1.0 — 수동 설치"
ISAAC="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
if [ -x "$ISAAC/python.sh" ]; then
    note "설치돼 있다: $ISAAC"
else
    note "아직 없다. NVIDIA 에서 **Isaac Sim 5.1.0 리눅스 바이너리(zip)** 를 받아 푼다."
    note "  - 기본 위치는 ~/isaacsim (다르면 ISAAC_SIM_PATH 로 알려준다)"
    note "  - \$ISAAC_SIM_PATH/python.sh 가 있어야 우리 스크립트가 동작한다"
    note "  - 문서 검증 드라이버: 595.58.03 / 최소 요건 VRAM 16GB · RAM 32GB · 저장공간 50GB"
    note "  - pip·컨테이너 설치는 쓰지 않는다 (우리 실행 경로가 python.sh 를 전제로 한다)"
fi

say "다음 순서"
note "1) Isaac Sim 설치 (위 5번)"
note "2) 레벨 USD 복원 + 파생 레벨 생성  → docs/doyoon-kim/manipulation/SETUP_NEW_PC.md §5"
note "3) ./scripts/setup_check.sh   — 빠진 것 확인"
note "4) ./scripts/run_tests.sh     — Isaac 없이 도는 테스트 100개"
note ""
note "터미널은 terminator 를 깔아 두었다. Ctrl+Shift+E 세로분할 / Ctrl+Shift+O 가로분할 —"
note "시연 때 터미널 A(Isaac)·B(노드)·C(명령)·D(로그) 를 한 창에서 보면 편하다"
[ "$DRY" -eq 1 ] && note "" && note "(--dry-run 이었다. 실제로 하려면 인자 없이 다시 실행)"
