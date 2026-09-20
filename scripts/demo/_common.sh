# 공통 설정. 각 스크립트가 source 한다.
#
# 왜 프로파일인가: 레벨 USD 와 꽂을 좌표는 **한 쌍**이다. 따로 넘기면 반드시 섞인다
# (2026-09-20: Franka 좌표를 M0609 레벨에 써서 두 선반 단 사이 허공을 가리켰다).
# 이름 하나만 고르게 한다.
set -u
DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$DEMO_DIR/../.." && pwd)"

PROFILE="${PROFILE:-pick}"
PROFILE_FILE="$DEMO_DIR/profiles/$PROFILE.env"
[ -f "$PROFILE_FILE" ] || {
    echo "프로파일이 없다: $PROFILE"
    echo "  쓸 수 있는 것: $(ls "$DEMO_DIR/profiles" | sed 's/\.env//' | tr '\n' ' ')"
    exit 1
}
# shellcheck disable=SC1090
. "$PROFILE_FILE"

# --- PC 마다 다른 값 (git 에 올리지 않는다) -----------------------------------
# scripts/demo/demo_local.env 에 두거나 환경변수로 넘긴다.
[ -f "$DEMO_DIR/demo_local.env" ] && . "$DEMO_DIR/demo_local.env"

SIM_HOST="${SIM_HOST:-10.10.0.2}"          # Isaac 을 띄울 PC. 그 PC 에서 직접 돌리면 local
SIM_USER="${SIM_USER:-rokey}"
SIM_REPO="${SIM_REPO:-~/b1_arm}"            # GPU PC 의 저장소 경로
SIM_LEVEL_DIR="${SIM_LEVEL_DIR:-~/Desktop/Collected_ing_library_env_v5-firstFinal}"
# 책 USD 는 **저장소 안에 있다** (simulation/assets/book_dataset/usd_v2/).
# 예전에는 개인 홈 경로만 봐서, 새로 클론한 PC 에서는 저장소에 있는 파일을 못 찾았다 (2026-09-20).
# 원격(GPU PC)에서 돌 때는 그 PC 의 저장소 경로를 쓰므로 $SIM_REPO 기준으로 만든다.
SIM_BOOK_GLOB="${SIM_BOOK_GLOB:-$SIM_REPO/simulation/assets/book_dataset/usd_v2/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book0[1-6].usdc}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

CAMERA_PRIM="${CAMERA_PRIM:-/World/Nova_Carter_ROS/m0609/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color}"

say() { printf '%s\n' "$*" >&2; }

ros_env() {
    set +u
    . /opt/ros/jazzy/setup.bash
    . "$REPO_ROOT/ros2_ws/install/setup.bash"
    set -u
}
