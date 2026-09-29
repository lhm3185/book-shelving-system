#!/usr/bin/env bash
# One-command PC B onboard runtime.
# Starts navigation, perception, and manipulation with all ROS environment
# settings loaded from config/ros_network.env.

set -Eeo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/../.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"
NETWORK_ENV="$REPO_ROOT/config/ros_network.env"
PERCEPTION_SITE_PACKAGES="$REPO_ROOT/.venv/lib/python3.12/site-packages"
DDS_PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/book-shelving-system-$(id -u)"
LAUNCHER_STATE="$RUNTIME_DIR/pc_b_launcher.pid"
ROS_LAUNCH_STATE="$RUNTIME_DIR/pc_b_ros.pgid"

ROS_LAUNCH_PID=""
CLEANUP_STARTED=0


process_group_is_running() {
    local process_group_id="$1"

    [[ -n "$process_group_id" ]] \
        && kill -0 -- "-$process_group_id" 2>/dev/null
}


signal_process_group() {
    local signal_name="$1"
    local process_group_id="$2"

    if process_group_is_running "$process_group_id"; then
        kill "-$signal_name" -- "-$process_group_id" 2>/dev/null || true
    fi
}


write_process_state() {
    local state_file="$1"
    local process_id="$2"
    local process_start_time
    local temporary_file

    process_start_time="$(awk '{print $22}' "/proc/$process_id/stat")"
    temporary_file="$state_file.$$"

    mkdir -p "$RUNTIME_DIR"
    chmod 700 "$RUNTIME_DIR"
    printf '%s %s\n' "$process_id" "$process_start_time" \
        > "$temporary_file"
    mv -f "$temporary_file" "$state_file"
}


remove_process_state() {
    local state_file="$1"
    local expected_process_id="$2"
    local recorded_process_id=""
    local ignored_start_time=""

    if [[ -r "$state_file" ]]; then
        read -r recorded_process_id ignored_start_time < "$state_file" || true
    fi

    if [[ "$recorded_process_id" == "$expected_process_id" ]]; then
        rm -f -- "$state_file"
    fi
}


wait_for_ros_launch() {
    local retry_count="$1"
    local index

    for ((index = 0; index < retry_count; index++)); do
        if ! process_group_is_running "$ROS_LAUNCH_PID"; then
            return 0
        fi

        sleep 0.5
    done

    return 1
}


cleanup() {
    local exit_code=$?

    if ((CLEANUP_STARTED)); then
        return
    fi

    CLEANUP_STARTED=1
    trap - EXIT
    trap '' INT TERM

    printf '\nPC B 구성요소를 종료합니다.\n'
    signal_process_group INT "$ROS_LAUNCH_PID"

    if ! wait_for_ros_launch 20; then
        signal_process_group TERM "$ROS_LAUNCH_PID"
    fi

    if ! wait_for_ros_launch 10; then
        signal_process_group KILL "$ROS_LAUNCH_PID"
        wait_for_ros_launch 4 || true
    fi

    if [[ -n "$ROS_LAUNCH_PID" ]]; then
        wait "$ROS_LAUNCH_PID" 2>/dev/null || true
    fi

    remove_process_state "$ROS_LAUNCH_STATE" "$ROS_LAUNCH_PID"
    remove_process_state "$LAUNCHER_STATE" "$$"
    rmdir "$RUNTIME_DIR" 2>/dev/null || true

    exit "$exit_code"
}


if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy를 찾을 수 없습니다." >&2
    exit 1
fi

if [[ ! -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
    echo "ROS workspace가 빌드되지 않았습니다." >&2
    exit 1
fi

if [[ ! -f "$NETWORK_ENV" ]]; then
    echo "공통 ROS 네트워크 설정이 없습니다: $NETWORK_ENV" >&2
    exit 1
fi

if [[ ! -f "$DDS_PROFILE" ]]; then
    echo "Fast DDS 유선 인터페이스 설정이 없습니다: $DDS_PROFILE" >&2
    exit 1
fi

if [[ ! -d "$PERCEPTION_SITE_PACKAGES/ultralytics" ]]; then
    echo "PC B perception Python 환경이 없습니다." >&2
    echo "확인 경로: $PERCEPTION_SITE_PACKAGES" >&2
    exit 1
fi

# ROS generated setup files are not compatible with `set -u`.
source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

set -a
source "$NETWORK_ENV"
set +a

export PYTHONPATH="$PERCEPTION_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"

export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
unset ROS_STATIC_PEERS

trap cleanup EXIT
trap 'exit 130' INT TERM

write_process_state "$LAUNCHER_STATE" "$$"

printf 'PC B를 시작합니다. ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"

setsid ros2 launch \
    shelving_system \
    pc_b.launch.py \
    use_sim_time:=true &
ROS_LAUNCH_PID=$!
write_process_state "$ROS_LAUNCH_STATE" "$ROS_LAUNCH_PID"

wait "$ROS_LAUNCH_PID"
