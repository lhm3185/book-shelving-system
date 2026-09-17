#!/usr/bin/env bash
# 연동 시연 — 이 PC 쪽 (정적 TF 2개 + 비전 vision_manager + 로봇팔 manipulation_node).
# 먼저 GPU PC 에서 Isaac 실행기를 띄우고 "준비 완료" 를 확인한다 (INTEGRATION_DEMO.md).
#   ROS_DOMAIN_ID=130 ./run_demo_pc.sh          Ctrl+C 로 전부 종료
set -u  # ROS setup.bash 는 미정의 변수를 써서 source 앞뒤로 +u/-u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 GPU PC 와 같은 값으로 export 할 것}"
WS="${WS:-$HOME/ws_cobot_pjt/book-shelving-system/ros2_ws}"
MODEL_PATH="${MODEL_PATH:-$HOME/ws_cobot_pjt/arm/models/book_best.pt}"
CAMERA_PRIM_FRAME="${CAMERA_PRIM_FRAME:-Camera_OmniVision_OV9782_Color}"
LOG="${LOG:-/tmp/b1_demo}"; mkdir -p "$LOG"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
set +u; source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"; set -u
[ -f "$MODEL_PATH" ] || { echo "모델 없음: $MODEL_PATH"; exit 1; }

pids=()
cleanup() { echo; echo "종료 중..."; kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; echo "종료"; }
trap cleanup INT TERM EXIT

# 팀 프레임 별칭: 로봇팔 기준 좌표
ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link \
    --ros-args -p use_sim_time:=true > "$LOG/tf_arm.log" 2>&1 & pids+=($!)
# 카메라 prim TF(Isaac, 광학 규약) → 이미지 frame sim_camera (항등)
ros2 run tf2_ros static_transform_publisher --frame-id "$CAMERA_PRIM_FRAME" --child-frame-id sim_camera \
    --ros-args -p use_sim_time:=true > "$LOG/tf_cam.log" 2>&1 & pids+=($!)
# 비전 임계값: origin/vision fa11b0b 에서 코드 기본값이 0.5 → 0.75 로 바뀌었지만 perception.yaml 은 0.5 로 남아 있어
# --params-file 로 실행하면 0.5 가 적용된다 (2026-09-17 확인). 변경 의도대로 0.75 를 명시한다. 바꾸려면 VISION_CONF=0.6 등
ros2 run shelving_perception vision_manager --ros-args \
    --params-file "$WS/src/shelving_perception/config/perception.yaml" -p model_path:="$MODEL_PATH" \
    -p confidence_threshold:="${VISION_CONF:-0.75}" \
    > "$LOG/vision.log" 2>&1 & pids+=($!)
ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file "$WS/src/shelving_manipulation/config/manipulation.yaml" -p executor:=sim \
    > "$LOG/manipulation.log" 2>&1 & pids+=($!)

echo "실행됨 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID). 로그: $LOG"
echo "  비전      tail -f $LOG/vision.log"
echo "  로봇팔    tail -f $LOG/manipulation.log"
wait
