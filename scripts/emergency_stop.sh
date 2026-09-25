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

declare -A PROCESS_GROUPS=()

PATTERNS=(
    "ros2 launch shelving_system all.launch.py"
    "$REPO_ROOT/isaac_sim/run_simulation.py"
    "$REPO_ROOT/ros2_ws/install/"
    "/opt/ros/jazzy/lib/nav2_"
    "pointcloud_to_laserscan_node"
)

collect_process_groups() {
    local pattern
    local pid
    local process_group

    for pattern in "${PATTERNS[@]}"; do
        while IFS= read -r pid; do
            [[ -n "$pid" ]] || continue

            process_group="$(
                ps -o pgid= -p "$pid" 2>/dev/null \
                    | tr -d ' '
            )"

            [[ "$process_group" =~ ^[0-9]+$ ]] || continue

            # 현재 stop 스크립트와 터미널은 종료하지 않는다.
            if [[ "$process_group" == "$CURRENT_PROCESS_GROUP" ]]; then
                continue
            fi

            PROCESS_GROUPS["$process_group"]=1
        done < <(
            pgrep \
                -u "$CURRENT_USER_ID" \
                -f -- "$pattern" \
                2>/dev/null || true
        )
    done
}

process_group_is_running() {
    local process_group="$1"

    kill -0 -- "-$process_group" 2>/dev/null
}

send_signal() {
    local signal_name="$1"
    local process_group

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
}

wait_for_exit() {
    local retry_count="$1"
    local index
    local process_group
    local running

    for ((index = 0; index < retry_count; index++)); do
        running=0

        for process_group in "${!PROCESS_GROUPS[@]}"; do
            if process_group_is_running "$process_group"; then
                running=1
                break
            fi
        done

        if ((running == 0)); then
            return 0
        fi

        sleep 0.5
    done

    return 1
}

collect_process_groups

if ((${#PROCESS_GROUPS[@]} == 0)); then
    printf '[stop] 종료할 프로젝트 프로세스가 없습니다.\n'
    exit 0
fi

printf \
    '[stop] 발견한 프로세스 그룹: %s\n' \
    "${!PROCESS_GROUPS[*]}"

send_signal INT

if ! wait_for_exit 20; then
    send_signal TERM
fi

if ! wait_for_exit 10; then
    send_signal KILL
    wait_for_exit 4 || true
fi

printf '[stop] 프로젝트 프로세스 정리를 완료했습니다.\n'