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
WORLD_USD="$REPO_ROOT/isaac_sim/assets/worlds/library_environment.usda"

for required in \
    "$ISAAC_SIM_PATH/python.sh" \
    "$RUNNER" \
    "$SCENE_CONFIG" \
    "$ROBOT_USD" \
    "$WORLD_USD"
do
    if [ ! -f "$required" ]; then
        echo "Required Ridgeback-Franka runtime file is missing: $required" >&2
        exit 1
    fi
done

ISAAC_ROS_LIB="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/jazzy/lib"
if [ ! -d "$ISAAC_ROS_LIB" ]; then
    echo "Isaac Sim ROS 2 Jazzy bridge was not found: $ISAAC_ROS_LIB" >&2
    exit 1
fi

# Isaac Sim 5.1 embeds Python 3.11.  Remove ROS Jazzy's system Python 3.12
# paths before starting it, then expose only Isaac's bundled ROS bridge libs.
strip_ros() {
    printf '%s' "$1" \
        | tr ':' '\n' \
        | awk 'NF && $0 !~ "^/opt/ros/" && $0 !~ "/ros2_ws/install"' \
        | paste -sd:
}

export PYTHONPATH="$(strip_ros "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_ros "${LD_LIBRARY_PATH:-}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_VERSION ROS_PYTHON_VERSION ROS_AUTOMATIC_DISCOVERY_RANGE
unset FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=130
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_ROS_LIB"

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
