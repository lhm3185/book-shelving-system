#!/usr/bin/env bash
# Book Shelving System 통합 실행 진입점.
#
# 현재 지원 모드:
#   navigation-test
#
# 향후 지원 예정:
#   full

set -Eeo pipefail


REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"
ISAAC_RUNNER="$REPO_ROOT/scripts/isaac/run_standalone.sh"

MODE="${1:-navigation-test}"

STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-180}"
ROS_READY_TIMEOUT_SEC="${ROS_READY_TIMEOUT_SEC:-120}"

ISAAC_PID=""
ROS_LAUNCH_PID=""


print_message() {
    printf '\n[%s] %s\n' \
        "$(date '+%H:%M:%S')" \
        "$1"
}


print_error() {
    printf '\n[ERROR] %s\n' "$1" >&2
}


CLEANUP_STARTED=0


process_group_is_running() {
    local process_group_id="$1"

    [[ -n "$process_group_id" ]] \
        && kill -0 -- "-$process_group_id" 2>/dev/null
}


signal_process_group() {
    local signal_name="$1"
    local process_group_id="$2"
    local process_name="$3"

    if ! process_group_is_running "$process_group_id"; then
        return
    fi

    print_message \
        "${process_name} 프로세스 그룹에 ${signal_name} 신호를 보냅니다."

    kill "-${signal_name}" \
        -- "-$process_group_id" \
        2>/dev/null || true
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

    # EXIT 재진입을 막고 추가 Ctrl+C는 종료 작업 중 무시한다.
    trap - EXIT
    trap '' INT TERM

    print_message "전체 구성요소 종료 시작"

    # ROS와 Isaac에 동시에 종료 신호를 보낸다.
    signal_process_group \
        INT "$ROS_LAUNCH_PID" "ROS launch"

    signal_process_group \
        INT "$ISAAC_PID" "Isaac Sim"

    if ! wait_for_managed_processes 20; then
        signal_process_group \
            TERM "$ROS_LAUNCH_PID" "ROS launch"

        signal_process_group \
            TERM "$ISAAC_PID" "Isaac Sim"
    fi

    if ! wait_for_managed_processes 10; then
        signal_process_group \
            KILL "$ROS_LAUNCH_PID" "ROS launch"

        signal_process_group \
            KILL "$ISAAC_PID" "Isaac Sim"

        wait_for_managed_processes 4 || true
    fi

    if [[ -n "$ROS_LAUNCH_PID" ]]; then
        wait "$ROS_LAUNCH_PID" 2>/dev/null || true
    fi

    if [[ -n "$ISAAC_PID" ]]; then
        wait "$ISAAC_PID" 2>/dev/null || true
    fi

    print_message "전체 실행을 종료했습니다."

    exit "$exit_code"
}

check_managed_processes() {
    if [[ -n "$ISAAC_PID" ]] \
        && ! kill -0 "$ISAAC_PID" 2>/dev/null
    then
        print_error "Isaac Sim 프로세스가 종료됐습니다."
        return 1
    fi

    if [[ -n "$ROS_LAUNCH_PID" ]] \
        && ! kill -0 "$ROS_LAUNCH_PID" 2>/dev/null
    then
        print_error "ROS launch 프로세스가 종료됐습니다."
        return 1
    fi

    return 0
}


wait_until() {
    local description="$1"
    local timeout_seconds="$2"

    shift 2

    local deadline=$((SECONDS + timeout_seconds))

    print_message "${description} 대기 중"

    while ! "$@"; do
        check_managed_processes

        if ((SECONDS >= deadline)); then
            print_error \
                "${description} 준비 시간이 초과됐습니다."
            return 1
        fi

        sleep 1
    done

    print_message "${description} 준비 완료"
}


topic_exists() {
    local topic_name="$1"

    ros2 topic list 2>/dev/null \
        | grep -Fxq "$topic_name"
}


clock_is_publishing() {
    timeout 3 \
        ros2 topic echo \
        /clock \
        rosgraph_msgs/msg/Clock \
        --once \
        >/dev/null 2>&1
}

topic_is_publishing() {
    local topic_name="$1"
    local message_type="$2"

    timeout 3 \
        ros2 topic echo \
        "$topic_name" \
        "$message_type" \
        --once \
        >/dev/null 2>&1
}

scenario_is_ready() {
    timeout 3 \
        ros2 topic echo \
        /scenario/state \
        shelving_interfaces/msg/ScenarioState \
        --once \
        --qos-reliability reliable \
        --qos-durability transient_local \
        2>/dev/null \
        | grep -Eq \
            '^[[:space:]]*state: 3[[:space:]]*$'
}


action_exists() {
    local action_name="$1"

    ros2 action list 2>/dev/null \
        | grep -Fxq "$action_name"
}


service_exists() {
    local service_name="$1"

    ros2 service list 2>/dev/null \
        | grep -Fxq "$service_name"
}


lifecycle_is_active() {
    local node_name="$1"

    ros2 lifecycle get "$node_name" 2>/dev/null \
        | grep -q '^active'
}


case "$MODE" in
    navigation-test)
        ;;

    full)
        print_error \
            "full 모드는 아직 구현되지 않았습니다."

        print_error \
            "현재는 navigation-test만 사용할 수 있습니다."

        exit 2
        ;;

    *)
        print_error "알 수 없는 실행 모드: $MODE"

        printf '%s\n' \
            "사용 가능한 모드:" \
            "  navigation-test" \
            "  full  (아직 미구현)" \
            >&2

        exit 2
        ;;
esac


if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then
    print_error "ROS2 Jazzy를 찾을 수 없습니다."
    exit 1
fi


if [[ ! -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
    print_error \
        "ROS workspace가 빌드되지 않았습니다."

    print_error \
        "먼저 ros2_ws에서 colcon build를 실행하십시오."

    exit 1
fi


if [[ ! -x "$ISAAC_RUNNER" ]]; then
    print_error \
        "Isaac 실행 파일이 없거나 실행 권한이 없습니다:"

    print_error "$ISAAC_RUNNER"

    exit 1
fi


# ROS CLI와 ROS launch가 사용할 환경이다.
source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"


trap cleanup EXIT
trap 'exit 130' INT TERM


print_message "실행 모드: $MODE"
print_message "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"


# Isaac Sim은 별도 Python 환경으로 실행된다.
# scripts/isaac/run.sh가 ROS Python 경로를 정리한다.
# allow only one Isaac-Sim Process

if pgrep -f -- "$REPO_ROOT/isaac_sim/run_simulation.py" >/dev/null; then
    print_error "이미 실행 중인 Isaac Sim이 있습니다."
    pgrep -af -- "$REPO_ROOT/isaac_sim/run_simulation.py"
    exit 1
fi
print_message "Isaac Sim 실행"

setsid "$ISAAC_RUNNER" &
ISAAC_PID=$!


wait_until \
    "Isaac Sim /clock" \
    "$STARTUP_TIMEOUT_SEC" \
    clock_is_publishing


wait_until \
    "Ridgeback odometry /odom" \
    "$STARTUP_TIMEOUT_SEC" \
    topic_is_publishing \
    "/odom" \
    "nav_msgs/msg/Odometry"

wait_until \
    "Ridgeback LiDAR /lidar/points_raw" \
    "$STARTUP_TIMEOUT_SEC" \
    topic_is_publishing \
    "/lidar/points_raw" \
    "sensor_msgs/msg/PointCloud2"

print_message "ROS 통합 launch 실행"

setsid ros2 launch \
    shelving_system \
    all.launch.py \
    mode:="$MODE" \
    use_sim_time:=true &

ROS_LAUNCH_PID=$!


wait_until \
    "Nav2 /navigate_to_pose action" \
    "$ROS_READY_TIMEOUT_SEC" \
    action_exists \
    "/navigate_to_pose"


wait_until \
    "Nav2 /navigate_through_poses action" \
    "$ROS_READY_TIMEOUT_SEC" \
    action_exists \
    "/navigate_through_poses"


wait_until \
    "Shelving /navigate_to_target action" \
    "$ROS_READY_TIMEOUT_SEC" \
    action_exists \
    "/navigate_to_target"


wait_until \
    "Mock /place_book action" \
    "$ROS_READY_TIMEOUT_SEC" \
    action_exists \
    "/place_book"

wait_until \
    "Tray /load_tray action" \
    "$ROS_READY_TIMEOUT_SEC" \
    action_exists \
    "/load_tray"

wait_until \
    "Nav2 BT navigator lifecycle" \
    "$ROS_READY_TIMEOUT_SEC" \
    lifecycle_is_active \
    "/bt_navigator"

wait_until \
    "Nav2 controller lifecycle" \
    "$ROS_READY_TIMEOUT_SEC" \
    lifecycle_is_active \
    "/controller_server"

wait_until \
    "LaserScan /scan data" \
    "$ROS_READY_TIMEOUT_SEC" \
    topic_is_publishing \
    "/scan" \
    "sensor_msgs/msg/LaserScan"

wait_until \
    "Nav2 local costmap data" \
    "$ROS_READY_TIMEOUT_SEC" \
    topic_is_publishing \
    "/local_costmap/costmap" \
    "nav_msgs/msg/OccupancyGrid"


wait_until \
    "Return machine publish service" \
    "$ROS_READY_TIMEOUT_SEC" \
    service_exists \
    "/return_machine/publish_job"

wait_until \
    "Isaac scenario READY" \
    "$ROS_READY_TIMEOUT_SEC" \
    scenario_is_ready


print_message "모든 구성요소가 준비됐습니다."
print_message "반납기 테스트 작업을 발행합니다."


ros2 service call \
    /return_machine/publish_job \
    std_srvs/srv/Trigger \
    "{}"


print_message "테스트 작업을 발행했습니다."
print_message "종료하려면 Ctrl+C를 누르십시오."


# ROS launch가 실행되는 동안 스크립트를 유지한다.
wait "$ROS_LAUNCH_PID"