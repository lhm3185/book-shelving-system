#!/usr/bin/env bash
# B-1 Isaac Sim 통합 실행 — 팀 공식 실행 명령.
#   ./scripts/run_isaac_sim.sh --gui        # 화면 있음
#   ./scripts/run_isaac_sim.sh --headless   # 화면 없음 (기본)
#   ./scripts/run_isaac_sim.sh --gui --book-variants mixed
#
# 저장소 코드를 그대로 실행한다 (~/arm 으로 복사하지 않는다).
# Isaac 설치 위치만 시스템마다 다르므로 ISAAC_SIM_PATH 로 받는다.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
[ -x "$ISAAC_SIM_PATH/python.sh" ] || {
    echo "Isaac 을 찾을 수 없다: $ISAAC_SIM_PATH"
    echo "  export ISAAC_SIM_PATH=/설치/경로  로 지정할 것"
    exit 1
}
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"

# PC 마다 다른 경로(레벨 USD 등)는 이 파일에 둔다. git 에 올리지 않는다.
[ -f "$REPO_ROOT/simulation/isaac/config/sim_local.env" ] && . "$REPO_ROOT/simulation/isaac/config/sim_local.env"

# SIM_USD를 따로 주지 않으면 world_loader가 저장소의 최종 v5-test 월드를 연다.
# 외부 바탕화면 파일을 자동 선택하지 않는다. 다른 월드를 시험할 때만 SIM_USD를 명시한다.
# 트레이는 **저장소 안에 있다** (simulation/assets/book_dataset/). 예전에는 개인 홈 경로만 봐서,
# 새로 클론한 PC 에서는 파일이 저장소에 있는데도 "없다" 가 됐다 (2026-09-20).
# 저장소 사본을 먼저 보고, 없으면 홈 경로로 넘어간다.
if [ -z "${SIM_TRAY:-}" ]; then
    for _t in "$REPO_ROOT/simulation/assets/book_dataset/assets/tray/tray_v1.usdc" \
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
# 설정 파일이 실제로 있을 때만 지정한다. 존재하지 않는 기본 경로를 넘기면 FastDDS가
# XMLPARSER_ERROR를 내므로, 새 PC의 단일 머신 테스트에서는 설정 없이 자동 검색을 쓴다.
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE+x}" ]; then
    _DDS_PROFILE="$HOME/.ros/fastdds_whitelist.xml"
    [ -f "$_DDS_PROFILE" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$_DDS_PROFILE"
elif [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "FASTRTPS_DEFAULT_PROFILES_FILE 파일이 없다: $FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    exit 1
fi
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
exec ./python.sh "$REPO_ROOT/simulation/isaac/run_simulation.py" "${ARGS[@]}"
