#!/usr/bin/env bash
# One-command PC C operator dashboard runtime.

set -Eeo pipefail

REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/../.."
    pwd
)"

ROS_WORKSPACE="$REPO_ROOT/ros2_ws"
NETWORK_ENV="$REPO_ROOT/config/ros_network.env"
DDS_PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

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

if ! python3 -c 'import fastapi, uvicorn, yaml' \
    >/dev/null 2>&1
then
    echo "PC C 웹 Python 패키지가 설치되지 않았습니다." >&2
    echo "다음 명령을 실행하십시오:" >&2
    echo "sudo apt install -y python3-fastapi python3-uvicorn python3-yaml" >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"

set -a
source "$NETWORK_ENV"
set +a

export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
unset ROS_STATIC_PEERS
unset ROS_LOCALHOST_ONLY

WEB_EXECUTABLE="$ROS_WORKSPACE/install/shelving_web/lib/shelving_web/web_gateway_node"

if pgrep \
    -u "$(id -u)" \
    -f -- "$WEB_EXECUTABLE" \
    >/dev/null
then
    echo "PC C 웹 게이트웨이가 이미 실행 중입니다." >&2
    echo "브라우저 주소: http://localhost:8080" >&2
    exit 1
fi

printf 'PC C 관제 웹을 시작합니다.\n'
printf '이 PC에서 접속: http://localhost:8080\n'
printf 'ROS 유선망에서 접속: http://10.10.0.3:8080\n'

exec ros2 launch \
    shelving_web \
    pc_c.launch.py \
    host:=0.0.0.0 \
    port:=8080