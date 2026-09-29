#!/usr/bin/env bash

set -Eeuo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.."
    pwd
)"

CURRENT_USER_ID="$(id -u)"
CURRENT_PROCESS_GROUP="$(
    ps -o pgid= -p "$$" | tr -d ' '
)"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/book-shelving-system-${CURRENT_USER_ID}"

declare -A PROCESS_GROUPS=()
declare -A PROCESS_IDS=()

PATTERNS=(
    "$REPO_ROOT/scripts/run.sh"
    "$REPO_ROOT/scripts/ros/run_pc_a.sh"
    "$REPO_ROOT/scripts/ros/run_pc_b.sh"
    "ros2 launch shelving_system all.launch.py"
    "ros2 launch shelving_system pc_a.launch.py"
    "ros2 launch shelving_system pc_b.launch.py"
    "$REPO_ROOT/isaac_sim/run_simulation.py"
    "$REPO_ROOT/ros2_ws/install/"
    "/opt/ros/jazzy/lib/nav2_"
    "pointcloud_to_laserscan_node"
    "vision_manager"
    "manipulation_node"
    "simulation_bridge_node"
    "task_manager_node"
    "return_machine_node"
    "ros2cli.daemon.daemonize"
)


process_id_is_running() {
    local process_id="$1"

    kill -0 "$process_id" 2>/dev/null
}


process_group_is_running() {
    local process_group="$1"

    kill -0 -- "-$process_group" 2>/dev/null
}


register_process_id() {
    local process_id="$1"
    local process_group

    [[ "$process_id" =~ ^[0-9]+$ ]] || return
    process_id_is_running "$process_id" || return

    process_group="$(
        ps -o pgid= -p "$process_id" 2>/dev/null \
            | tr -d ' '
    )"

    if [[ "$process_group" =~ ^[0-9]+$ ]] \
        && [[ "$process_group" != "$CURRENT_PROCESS_GROUP" ]]
    then
        PROCESS_GROUPS["$process_group"]=1
        return
    fi

    if [[ "$process_id" != "$$" ]]; then
        PROCESS_IDS["$process_id"]=1
    fi
}


register_state_file() {
    local state_file="$1"
    local process_id
    local expected_start_time
    local actual_start_time

    read -r process_id expected_start_time < "$state_file" || return

    [[ "$process_id" =~ ^[0-9]+$ ]] || return
    [[ "$expected_start_time" =~ ^[0-9]+$ ]] || return
    [[ -r "/proc/$process_id/stat" ]] || return

    actual_start_time="$(awk '{print $22}' "/proc/$process_id/stat")"
    [[ "$actual_start_time" == "$expected_start_time" ]] || return

    register_process_id "$process_id"
}


collect_registered_processes() {
    local state_file

    [[ -d "$RUNTIME_DIR" ]] || return

    shopt -s nullglob
    for state_file in "$RUNTIME_DIR"/*.pid "$RUNTIME_DIR"/*.pgid; do
        register_state_file "$state_file"
    done
    shopt -u nullglob
}


collect_matching_processes() {
    local pattern
    local process_id

    for pattern in "${PATTERNS[@]}"; do
        while IFS= read -r process_id; do
            [[ -n "$process_id" ]] || continue
            register_process_id "$process_id"
        done < <(
            pgrep \
                -u "$CURRENT_USER_ID" \
                -f -- "$pattern" \
                2>/dev/null || true
        )
    done
}


send_signal() {
    local signal_name="$1"
    local process_group
    local process_id

    for process_group in "${!PROCESS_GROUPS[@]}"; do
        if process_group_is_running "$process_group"; then
            printf \
                '[stop] PGID %s에 %s 신호를 보냅니다.\n' \
                "$process_group" \
                "$signal_name"

            kill "-$signal_name" \
                -- "-$process_group" \
                2>/dev/null || true
        fi
    done

    for process_id in "${!PROCESS_IDS[@]}"; do
        if process_id_is_running "$process_id"; then
            printf \
                '[stop] PID %s에 %s 신호를 보냅니다.\n' \
                "$process_id" \
                "$signal_name"

            kill "-$signal_name" \
                "$process_id" \
                2>/dev/null || true
        fi
    done
}


managed_processes_are_running() {
    local process_group
    local process_id

    for process_group in "${!PROCESS_GROUPS[@]}"; do
        if process_group_is_running "$process_group"; then
            return 0
        fi
    done

    for process_id in "${!PROCESS_IDS[@]}"; do
        if process_id_is_running "$process_id"; then
            return 0
        fi
    done

    return 1
}


wait_for_exit() {
    local retry_count="$1"
    local index

    for ((index = 0; index < retry_count; index++)); do
        if ! managed_processes_are_running; then
            return 0
        fi

        sleep 0.5
    done

    return 1
}


clear_runtime_state() {
    local state_file

    [[ -d "$RUNTIME_DIR" ]] || return

    shopt -s nullglob
    for state_file in "$RUNTIME_DIR"/*.pid "$RUNTIME_DIR"/*.pgid; do
        rm -f -- "$state_file"
    done
    shopt -u nullglob

    rmdir "$RUNTIME_DIR" 2>/dev/null || true
}


collect_registered_processes
collect_matching_processes

if ((${#PROCESS_GROUPS[@]} == 0 && ${#PROCESS_IDS[@]} == 0)); then
    clear_runtime_state
    printf '[stop] 종료할 프로젝트 프로세스가 없습니다.\n'
    exit 0
fi

printf \
    '[stop] 발견한 프로세스 그룹: %s\n' \
    "${!PROCESS_GROUPS[*]:-(없음)}"

send_signal INT

if ! wait_for_exit 20; then
    send_signal TERM
fi

if ! wait_for_exit 10; then
    send_signal KILL
    wait_for_exit 4 || true
fi

clear_runtime_state

if managed_processes_are_running; then
    printf '[stop] 일부 프로젝트 프로세스를 종료하지 못했습니다.\n' >&2
    exit 1
fi

printf '[stop] 프로젝트 프로세스 정리를 완료했습니다.\n'
