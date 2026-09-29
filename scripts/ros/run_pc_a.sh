#!/usr/bin/env bash
# One-command PC A runtime.
#
# Start order managed by this script:
#   1. Apply the shared ROS network environment.
#   2. Start Isaac Sim with the production manipulation backend.
#   3. Wait until PC B navigation/perception/manipulation are ready.
#   4. Start simulation_bridge, task_manager, and return_machine.
#   5. Wait until the complete system is READY for an operator command.

set -Eeo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/../.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"
NETWORK_ENV="$REPO_ROOT/config/ros_network.env"
ISAAC_RUNNER="$REPO_ROOT/scripts/isaac/run_standalone.sh"
DDS_PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-240}"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/book-shelving-system-$(id -u)"
LAUNCHER_STATE="$RUNTIME_DIR/pc_a_launcher.pid"
ISAAC_STATE="$RUNTIME_DIR/pc_a_isaac.pgid"
ROS_LAUNCH_STATE="$RUNTIME_DIR/pc_a_ros.pgid"

ISAAC_PID=""
ROS_LAUNCH_PID=""
CLEANUP_STARTED=0


message() {
    printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$1"
}


error() {
    printf '\n[ERROR] %s\n' "$1" >&2
}

load_graphical_session_environment() {
    local gnome_shell_pid=""
    local environment_entry=""

    gnome_shell_pid="$(
        pgrep -o -u "$(id -u)" -x gnome-shell || true
    )"

    if [[ -z "$gnome_shell_pid" ]]; then
        error "PC A에서 실행 중인 GNOME 그래픽 세션을 찾을 수 없습니다."
        return 1
    fi

    while IFS= read -r -d '' environment_entry; do
        case "$environment_entry" in
            DISPLAY=*|XAUTHORITY=*|WAYLAND_DISPLAY=*|XDG_RUNTIME_DIR=*|DBUS_SESSION_BUS_ADDRESS=*)
                export "$environment_entry"
                ;;
        esac
    done < "/proc/$gnome_shell_pid/environ"

    if [[ -z "${DISPLAY:-}" || -z "${XAUTHORITY:-}" ]]; then
        error "그래픽 세션의 DISPLAY 또는 XAUTHORITY를 가져오지 못했습니다."
        return 1
    fi
}

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


wait_for_managed_processes() {
    local retry_count="$1"
    local index

    for ((index = 0; index < retry_count; index++)); do
        if ! process_group_is_running "$ROS_LAUNCH_PID" \
            && ! process_group_is_running "$ISAAC_PID"
        then
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

    message "PC A 구성요소를 종료합니다."

    signal_process_group INT "$ROS_LAUNCH_PID"
    signal_process_group INT "$ISAAC_PID"

    if ! wait_for_managed_processes 20; then
        signal_process_group TERM "$ROS_LAUNCH_PID"
        signal_process_group TERM "$ISAAC_PID"
    fi

    if ! wait_for_managed_processes 10; then
        signal_process_group KILL "$ROS_LAUNCH_PID"
        signal_process_group KILL "$ISAAC_PID"
        wait_for_managed_processes 4 || true
    fi

    if [[ -n "$ROS_LAUNCH_PID" ]]; then
        wait "$ROS_LAUNCH_PID" 2>/dev/null || true
    fi

    if [[ -n "$ISAAC_PID" ]]; then
        wait "$ISAAC_PID" 2>/dev/null || true
    fi

    remove_process_state "$ROS_LAUNCH_STATE" "$ROS_LAUNCH_PID"
    remove_process_state "$ISAAC_STATE" "$ISAAC_PID"
    remove_process_state "$LAUNCHER_STATE" "$$"
    rmdir "$RUNTIME_DIR" 2>/dev/null || true

    exit "$exit_code"
}


managed_processes_are_alive() {
    if [[ -n "$ISAAC_PID" ]] \
        && ! kill -0 "$ISAAC_PID" 2>/dev/null
    then
        error "Isaac Sim이 예기치 않게 종료됐습니다."
        return 1
    fi

    if [[ -n "$ROS_LAUNCH_PID" ]] \
        && ! kill -0 "$ROS_LAUNCH_PID" 2>/dev/null
    then
        error "PC A ROS launch가 예기치 않게 종료됐습니다."
        return 1
    fi
}


wait_until() {
    local description="$1"
    local timeout_seconds="$2"

    shift 2

    local deadline=0
    if ((timeout_seconds > 0)); then
        deadline=$((SECONDS + timeout_seconds))
    fi

    message "$description"

    while ! "$@"; do
        managed_processes_are_alive

        if ((deadline > 0 && SECONDS >= deadline)); then
            error "$description 시간 초과"
            return 1
        fi

        sleep 1
    done
}


topic_has_message() {
    local topic_name="$1"
    local message_type="$2"

    timeout 3 ros2 topic echo \
        "$topic_name" \
        "$message_type" \
        --once \
        >/dev/null 2>&1
}


action_exists() {
    local action_name="$1"

    ros2 action list 2>/dev/null | grep -Fxq "$action_name"
}


lifecycle_is_active() {
    local node_name="$1"

    ros2 lifecycle get "$node_name" 2>/dev/null \
        | grep -q '^active'
}


pc_b_is_ready() {
    action_exists "/navigate_to_target" \
        && action_exists "/detect_grasp_point" \
        && action_exists "/detect_target_slot" \
        && action_exists "/place_book" \
        && lifecycle_is_active /bt_navigator \
        && topic_has_message /scan sensor_msgs/msg/LaserScan
}

system_is_ready() {
    timeout 3 ros2 topic echo \
        /system/status \
        shelving_interfaces/msg/SystemStatus \
        --once \
        --qos-reliability reliable \
        --qos-durability transient_local \
        2>/dev/null \
        | grep -Eq '^[[:space:]]*state: 3[[:space:]]*$'
}


if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    error "ROS 2 Jazzy를 찾을 수 없습니다."
    exit 1
fi

if [[ ! -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
    error "ROS workspace가 빌드되지 않았습니다."
    exit 1
fi

if [[ ! -f "$NETWORK_ENV" ]]; then
    error "공통 ROS 네트워크 설정이 없습니다: $NETWORK_ENV"
    exit 1
fi

if [[ ! -f "$DDS_PROFILE" ]]; then
    error "Fast DDS 유선 인터페이스 설정이 없습니다: $DDS_PROFILE"
    exit 1
fi

if [[ ! -x "$ISAAC_RUNNER" ]]; then
    error "Isaac 실행 스크립트가 없거나 실행할 수 없습니다: $ISAAC_RUNNER"
    exit 1
fi

# ROS generated setup files are not compatible with `set -u`, so this script
# intentionally uses only errexit and pipefail.
source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

set -a
source "$NETWORK_ENV"
set +a

# Use the direct 10.10.0.x Ethernet link only. This keeps the published topic
# rates unchanged while preventing large sensor streams from being duplicated
# over both Ethernet and Wi-Fi.
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
unset ROS_STATIC_PEERS

trap cleanup EXIT
trap 'exit 130' INT TERM

write_process_state "$LAUNCHER_STATE" "$$"

message "PC A를 시작합니다. ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

if pgrep -f -- "$REPO_ROOT/isaac_sim/run_simulation.py" >/dev/null; then
    error "이미 실행 중인 이 저장소의 Isaac Sim이 있습니다."
    exit 1
fi

if [[ -z "${DISPLAY:-}" || -z "${XAUTHORITY:-}" ]]; then
    load_graphical_session_environment
fi

message "Isaac Sim GUI를 DISPLAY=$DISPLAY 에 표시합니다."

message "Isaac Sim을 시작합니다."
setsid "$ISAAC_RUNNER" --enable-manipulation &
ISAAC_PID=$!
write_process_state "$ISAAC_STATE" "$ISAAC_PID"

wait_until \
    "Isaac Sim /clock을 기다립니다." \
    "$STARTUP_TIMEOUT_SEC" \
    topic_has_message \
    /clock \
    rosgraph_msgs/msg/Clock

wait_until \
    "Isaac Sim /odom을 기다립니다." \
    "$STARTUP_TIMEOUT_SEC" \
    topic_has_message \
    /odom \
    nav_msgs/msg/Odometry

wait_until \
    "PC B를 기다립니다. PC B에서 ./scripts/ros/run_pc_b.sh 를 실행하십시오." \
    0 \
    pc_b_is_ready

message "PC B 준비 완료. PC A ROS 노드를 시작합니다."
setsid ros2 launch \
    shelving_system \
    pc_a.launch.py \
    use_sim_time:=true &
ROS_LAUNCH_PID=$!
write_process_state "$ROS_LAUNCH_STATE" "$ROS_LAUNCH_PID"

wait_until \
    "전체 시스템 READY를 기다립니다." \
    "$STARTUP_TIMEOUT_SEC" \
    system_is_ready

message "전체 시스템 준비 완료. 관제 명령을 기다립니다."
message "수동 작업 시작: ros2 service call /system/start_cycle std_srvs/srv/Trigger '{}'"

wait "$ROS_LAUNCH_PID"
