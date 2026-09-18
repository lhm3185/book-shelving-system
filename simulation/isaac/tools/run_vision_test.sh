#!/usr/bin/env bash
# 이 PC: 비전 팀원 vision_manager 를 Isaac 손목 카메라에 붙인다.
# 먼저 GPU PC 에서:  ROS_DOMAIN_ID=<같은 값> ~/arm/isaac/run_place_book_server.sh --camera   ("준비 완료" 확인)
set -e
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 GPU PC 와 같게 export 할 것}"
TARGET_FRAME="${TARGET_FRAME:-wrist_camera_optical_frame}"   # arm_base_link 로 두면 vision_manager 의 TF 대기 문제로 좌표가 안 나온다 (문서 3절)
source /opt/ros/jazzy/setup.bash
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${WS:-$REPO_ROOT/ros2_ws}/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
LOG=/tmp/vision_test; mkdir -p $LOG
# 팀 프레임 별칭 + 광학 프레임 (Isaac 이 카메라 TF 를 이미 광학 규약으로 내므로 항등)
ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link \
    --ros-args -p use_sim_time:=true > $LOG/tf_arm.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id wrist_camera --child-frame-id wrist_camera_optical_frame \
    --ros-args -p use_sim_time:=true > $LOG/tf_optical.log 2>&1 &
ros2 run shelving_perception vision_manager --ros-args -p use_sim_time:=true \
    -p model_path:="${MODEL_PATH:-${VISION_MODEL:-$HOME/ws_cobot_pjt/arm/models/book_best.pt}}" -p target_frame:="$TARGET_FRAME" \
    2>&1 | tee $LOG/vision.log
