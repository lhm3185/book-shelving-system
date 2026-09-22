#!/usr/bin/env bash
# B-1 Isaac 도구 실행 — 통합 실행기와 같은 환경으로 임의의 Isaac 스크립트를 돌린다.
#   ISAAC_ENTRY=isaac_sim/tools/capture_spines.py ./isaac_sim/tools/run.sh --views 10
#   ./scripts/run_isaac_sim.sh --gui        # 화면 있음
#   ./scripts/run_isaac_sim.sh --headless   # 화면 없음 (기본)
#   ./scripts/run_isaac_sim.sh --gui --book-variants mixed
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

# 터미널에 시스템 ROS(Python 3.12)가 source 돼 있으면 Isaac(3.11)이 그 rclpy 를 먼저 import 하다 죽는다
# (2026-09-17 시연 준비 중 실제 발생). 시스템 ROS 경로를 걷어내고 Isaac 내장 jazzy 만 쓴다
strip_ros() { printf '%s' "$1" | tr ':' '\n' | grep -v -e '^/opt/ros/' -e '/ros2_ws/install' -e '^$' | paste -sd: -; }
export PYTHONPATH="$(strip_ros "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_ros "${LD_LIBRARY_PATH:-}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
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

ENTRY="${ISAAC_ENTRY:?ISAAC_ENTRY 로 실행할 파일을 지정할 것 (예: isaac_sim/tools/capture_spines.py)}"
# Isaac 설치 폴더로 옮겨가서 실행하므로, 상대경로는 **저장소 기준**으로 바꿔 둔다
[[ "$ENTRY" = /* ]] || ENTRY="$REPO_ROOT/$ENTRY"
[ -f "$ENTRY" ] || { echo "실행할 파일이 없다: $ENTRY"; exit 1; }

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$ENTRY" "${ARGS[@]}"
