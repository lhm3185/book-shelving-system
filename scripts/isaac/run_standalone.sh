#!/usr/bin/env bash
# Ridgeback-Franka Isaac Sim standalone runner.
#
# All scene files are loaded from this repository.  The only machine-specific
# path is the Isaac Sim installation itself.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
RUNNER="$REPO_ROOT/isaac_sim/run_simulation.py"
SCENE_CONFIG="$REPO_ROOT/isaac_sim/config/scenes.yaml"
ROBOT_USD="$REPO_ROOT/isaac_sim/assets/robots/franka_nav.usda"
WORLD_USD="$REPO_ROOT/isaac_sim/assets/worlds/library/library_environment.usd"
TRAY_USD="$REPO_ROOT/isaac_sim/assets/props/tray/tray_set.usd"

for required in \
    "$ISAAC_SIM_PATH/python.sh" \
    "$RUNNER" \
    "$SCENE_CONFIG" \
    "$ROBOT_USD" \
    "$WORLD_USD" \
    "$TRAY_USD"
do
    if [ ! -f "$required" ]; then
        echo "Required Ridgeback-Franka runtime file is missing: $required" >&2
        exit 1
    fi
done

ISAAC_ROS_ROOT="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/jazzy"
ISAAC_ROS_PYTHON="$ISAAC_ROS_ROOT/rclpy"
ISAAC_ROS_LIB="$ISAAC_ROS_ROOT/lib"

for required_directory in \
    "$ISAAC_ROS_PYTHON" \
    "$ISAAC_ROS_LIB"
do
    if [ ! -d "$required_directory" ]; then
        echo "Isaac Sim ROS 2 Jazzy path was not found: $required_directory" >&2
        exit 1
    fi
done

# Isaac Sim 5.1은 Python 3.11을 사용한다.
# 시스템 ROS 2의 Python 3.12 경로를 제거하고,
# Isaac에 포함된 Python 3.11용 ROS 패키지를 사용한다.
strip_ros() {
    printf '%s' "$1" \
        | tr ':' '\n' \
        | awk 'NF && $0 !~ "^/opt/ros/" && $0 !~ "/ros2_ws/install"' \
        | paste -sd:
}

CLEAN_ISAAC_PYTHONPATH="$(
    strip_ros "${PYTHONPATH:-}"
)"
CLEAN_ISAAC_LD_LIBRARY_PATH="$(
    strip_ros "${LD_LIBRARY_PATH:-}"
)"

export PYTHONPATH="$ISAAC_ROS_PYTHON"

if [[ -n "$CLEAN_ISAAC_PYTHONPATH" ]]; then
    export PYTHONPATH="$PYTHONPATH:$CLEAN_ISAAC_PYTHONPATH"
fi

export LD_LIBRARY_PATH="$ISAAC_ROS_LIB"

if [[ -n "$CLEAN_ISAAC_LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH="$CLEAN_ISAAC_LD_LIBRARY_PATH:$LD_LIBRARY_PATH"
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_VERSION ROS_PYTHON_VERSION ROS_AUTOMATIC_DISCOVERY_RANGE
unset FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=130
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# run_simulation.py opens a GUI by default.  Keep --gui as a compatibility
# alias and pass --headless through when explicitly requested.
ARGS=()
for argument in "$@"; do
    case "$argument" in
        --gui) ;;
        *) ARGS+=("$argument") ;;
    esac
done

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$RUNNER" --config "$SCENE_CONFIG" "${ARGS[@]}"
