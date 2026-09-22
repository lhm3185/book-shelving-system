#!/usr/bin/env bash
# B-1 Isaac Sim 통합 실행 — 팀 공식 실행 명령.
#   ./scripts/run_isaac_sim.sh --gui        # 화면 있음
#   ./scripts/run_isaac_sim.sh --headless   # 화면 없음 (기본)
#   SIM_USD=~/Desktop/ing_library_env_v5.usd ./scripts/run_isaac_sim.sh --gui --book-variants mixed
#
# 저장소 코드를 그대로 실행한다 (~/arm 으로 복사하지 않는다).
# Isaac 설치 위치만 시스템마다 다르므로 ISAAC_SIM_PATH 로 받는다.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
[ -x "$ISAAC_SIM_PATH/python.sh" ] || {
    echo "Isaac 을 찾을 수 없다: $ISAAC_SIM_PATH"
    echo "  export ISAAC_SIM_PATH=/설치/경로  로 지정할 것"
    exit 1
}
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"

# PC 마다 다른 경로(레벨 USD 등)는 이 파일에 둔다. git 에 올리지 않는다.
[ -f "$REPO_ROOT/isaac_sim/isaac/config/sim_local.env" ] && . "$REPO_ROOT/isaac_sim/isaac/config/sim_local.env"

# 통합 USD 가 아직 자리 파일이면, 이 PC 에 있는 시험 레벨로 자동 대체한다.
# (시연 당일 다른 터미널에서 SIM_USD 를 빠뜨려 실행이 막히는 것을 막기 위함 — 2026-09-18)
if [ -z "${SIM_USD:-}" ]; then
    REPO_USD="$REPO_ROOT/isaac_sim/library_system.usd"
    if [ ! -s "$REPO_USD" ] || [ "$(stat -c%s "$REPO_USD" 2>/dev/null || echo 0)" -lt 4096 ]; then
        for cand in "$HOME/Desktop/ing_library_env_v5.usd" "$HOME/b1_assets/level/ing_library_env_v5.usd"; do
            if [ -f "$cand" ]; then
                export SIM_USD="$cand"
                echo "### 통합 USD 가 아직 비어 있어 시험 레벨을 쓴다: $SIM_USD" >&2
                break
            fi
        done
    fi
fi
# 트레이는 **저장소 안에 있다** (isaac_sim/assets/book_dataset/). 예전에는 개인 홈 경로만 봐서,
# 새로 클론한 PC 에서는 파일이 저장소에 있는데도 "없다" 가 됐다 (2026-09-20).
# 저장소 사본을 먼저 보고, 없으면 홈 경로로 넘어간다.
if [ -z "${SIM_TRAY:-}" ]; then
    for _t in "$REPO_ROOT/isaac_sim/assets/book_dataset/assets/tray/tray_v1.usdc" \
              "$HOME/book_dataset/assets/tray/tray_v1.usdc"; do
        [ -f "$_t" ] && { export SIM_TRAY="$_t"; break; }
    done
fi

# 터미널에 시스템 ROS(Python 3.12)가 source 돼 있으면 Isaac(3.11)이 그 rclpy 를 먼저 import 하다 죽는다
# (2026-09-17 시연 준비 중 실제 발생). 시스템 ROS 경로를 걷어내고 Isaac 내장 jazzy 만 쓴다
strip_ros() { printf '%s' "$1" | tr ':' '\n' | grep -v -e '^/opt/ros/' -e '/ros2_ws/install' -e '^$' | paste -sd: -; }
export PYTHONPATH="$(strip_ros "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_ros "${LD_LIBRARY_PATH:-}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ":-" 가 아니라 "-" 다. ":-" 는 빈 값도 기본값으로 바꿔서,
# FASTRTPS_DEFAULT_PROFILES_FILE= 로 끄려 해도 도로 켜진다.
# **한 PC 안에서 다 돌릴 때는 꺼야 한다** — 화이트리스트가 내장 전송(공유메모리·
# 로컬호스트)을 끄기 때문에 같은 PC 의 노드끼리 서로를 못 찾는다 (2026-09-21 실측).
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE-$HOME/.ros/fastdds_whitelist.xml}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/jazzy/lib"

# 원격(ssh)에서 창을 이 PC 모니터에 띄울 때
ARGS=()
GUI=0
for a in "$@"; do
    case "$a" in
        --gui) GUI=1; ARGS+=("$a") ;;
        --headless) GUI=0 ;;              # run_simulation.py 는 --gui 가 없으면 headless
        *) ARGS+=("$a") ;;
    esac
done
if [ "$GUI" = "1" ] && [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:1
    export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
fi

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$REPO_ROOT/isaac_sim/run_simulation.py" "${ARGS[@]}"
