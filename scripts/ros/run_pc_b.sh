#!/usr/bin/env bash
# PC B onboard runtime:
# navigation + perception + manipulation

set -eo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/../.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"
PERCEPTION_SITE_PACKAGES="$REPO_ROOT/.venv/lib/python3.12/site-packages"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy를 찾을 수 없습니다." >&2
    exit 1
fi

if [[ ! -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
    echo "ROS workspace가 빌드되지 않았습니다." >&2
    echo "먼저 ros2_ws에서 colcon build를 실행하십시오." >&2
    exit 1
fi

if [[ ! -d "$PERCEPTION_SITE_PACKAGES/ultralytics" ]]; then
    echo "PC B perception Python 환경이 없습니다." >&2
    echo "확인 경로: $PERCEPTION_SITE_PACKAGES" >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

# ROS 노드는 시스템 Python 3.12로 실행되지만 YOLO/PyTorch는
# 프로젝트 .venv에 있으므로 site-packages만 추가한다.
export PYTHONPATH="$PERCEPTION_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS

printf 'PC B 온보드 노드를 시작합니다.\n'
printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf 'RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"

exec ros2 launch \
    shelving_system \
    pc_b.launch.py \
    use_sim_time:=true
