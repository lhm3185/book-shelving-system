#!/usr/bin/env bash
# PC A ROS runtime:
# simulation bridge + task manager + return machine

set -euo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/../.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy를 찾을 수 없습니다." >&2
    exit 1
fi

if [[ ! -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
    echo "ROS workspace가 빌드되지 않았습니다." >&2
    echo "먼저 ros2_ws에서 colcon build를 실행하십시오." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 기존 사용자 Fast DDS 화이트리스트가 특정 NIC나 localhost만
# 허용하여 PC 간 통신을 막는 상황을 피한다.
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS

printf 'PC A ROS 노드를 시작합니다.\n'
printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf 'RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"

exec ros2 launch \
    shelving_system \
    pc_a.launch.py \
    use_sim_time:=true
