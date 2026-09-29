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

unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS

printf 'PC B를 시작합니다. ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"

exec ros2 launch \
    shelving_system \
    pc_b.launch.py \
    use_sim_time:=true
