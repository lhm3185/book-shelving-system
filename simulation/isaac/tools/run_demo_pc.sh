#!/usr/bin/env bash
# 연동 시연 — 이 PC 쪽 (정적 TF 2개 + 비전 vision_manager + 로봇팔 manipulation_node).
# 먼저 GPU PC 에서 Isaac 실행기를 띄우고 "준비 완료" 를 확인한다 (INTEGRATION_DEMO.md).
#   ROS_DOMAIN_ID=130 ./run_demo_pc.sh          Ctrl+C 로 전부 종료
set -u  # ROS setup.bash 는 미정의 변수를 써서 source 앞뒤로 +u/-u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 GPU PC 와 같은 값으로 export 할 것}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${WS:-$REPO_ROOT/ros2_ws}"
MODEL_PATH="${MODEL_PATH:-${VISION_MODEL:-$HOME/ws_cobot_pjt/arm/models/book_tray_best.pt}}"
# 책장 모델: 빈 칸 판정(새 방식)에 필요하다. perception.yaml 의 경로는 비전 담당 PC 기준이라
# 이 PC 에는 없다 — 없으면 노드가 뜨자마자 죽어 "토픽이 안 나온다" 로만 보인다 (2026-09-18)
SHELF_MODEL="${SHELF_MODEL:-$HOME/ws_cobot_pjt/arm/models/shelf_best.pt}"
CAMERA_PRIM_FRAME="${CAMERA_PRIM_FRAME:-Camera_OmniVision_OV9782_Color}"
LOG="${LOG:-/tmp/b1_demo}"; mkdir -p "$LOG"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
set +u; source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"; set -u
# 비전 패키지가 모델을 동봉하게 되었다 (origin/vision 33650a0, LFS). 개인 경로가 없으면
# 동봉본으로 넘어간다 — 예전에는 여기서 그냥 죽어서 "토픽이 안 나온다" 로만 보였다 (2026-09-18).
BUNDLED="$WS/install/shelving_perception/share/shelving_perception/resource"
[ -f "$MODEL_PATH" ]  || { [ -f "$BUNDLED/book_tray_best.pt" ] && MODEL_PATH="$BUNDLED/book_tray_best.pt" \
    && echo "책 모델: 개인 경로에 없어 패키지 동봉본을 쓴다"; }
[ -f "$SHELF_MODEL" ] || { [ -f "$BUNDLED/best.pt" ] && SHELF_MODEL="$BUNDLED/best.pt" \
    && echo "책장 모델: 개인 경로에 없어 패키지 동봉본을 쓴다"; }
[ -f "$MODEL_PATH" ] || { echo "책 모델 없음: $MODEL_PATH (동봉본도 없다 — colcon build 후 git lfs pull)"; exit 1; }
[ -f "$SHELF_MODEL" ] || { echo "책장 모델 없음: $SHELF_MODEL (동봉본도 없다 — colcon build 후 git lfs pull)"; exit 1; }

pids=()
cleanup() { echo; echo "종료 중..."; kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; echo "종료"; }
trap cleanup INT TERM EXIT

# 팀 프레임 별칭: 로봇팔 기준 좌표.
# **부모 프레임은 로봇마다 다르다.** Isaac 은 팔 베이스 prim 의 **이름**을 TF frame 으로 낸다
#   franka → panda_link0 / m0609 → base_link
# 예전에는 panda_link0 이 박혀 있어서 M0609 에서는 이 별칭이 트리에 안 붙었고,
# `arm_base_link` 로 TF 조회가 실패했다 (2026-09-20. 비전 쪽이 그래서 base_link 를 직접 쓰게 바꿨다).
ARM_BASE_FRAME="${ARM_BASE_FRAME:-$(ARM_ROBOT="${ARM_ROBOT:-m0609}" python3 -c \
    "import sys; sys.path.insert(0, '$REPO_ROOT/simulation/isaac/config'); \
     from robot_profiles import profile; print(profile().base_link.split('/')[-1])")}"
echo "팔 기준 프레임: $ARM_BASE_FRAME → arm_base_link (ARM_ROBOT=${ARM_ROBOT:-m0609})"
ros2 run tf2_ros static_transform_publisher --frame-id "$ARM_BASE_FRAME" --child-frame-id arm_base_link \
    --ros-args -p use_sim_time:=true > "$LOG/tf_arm.log" 2>&1 & pids+=($!)
# 카메라 prim TF(Isaac, 광학 규약) → 이미지 frame sim_camera (항등)
ros2 run tf2_ros static_transform_publisher --frame-id "$CAMERA_PRIM_FRAME" --child-frame-id sim_camera \
    --ros-args -p use_sim_time:=true > "$LOG/tf_cam.log" 2>&1 & pids+=($!)
# 비전 임계값: origin/vision fa11b0b 에서 코드 기본값이 0.5 → 0.75 로 바뀌었지만 perception.yaml 은 0.5 로 남아 있어
# --params-file 로 실행하면 0.5 가 적용된다 (2026-09-17 확인). 변경 의도대로 0.75 를 명시한다. 바꾸려면 VISION_CONF=0.6 등
ros2 run shelving_perception vision_manager --ros-args \
    --params-file "$WS/src/shelving_perception/config/perception.yaml" -p model_path:="$MODEL_PATH" \
    -p shelf_model_path:="$SHELF_MODEL" \
    -p confidence_threshold:="${VISION_CONF:-0.75}" \
    ${VISION_EXTRA:-} \
    > "$LOG/vision.log" 2>&1 & pids+=($!)
ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file "$WS/src/shelving_manipulation/config/manipulation.yaml" -p executor:=sim \
    > "$LOG/manipulation.log" 2>&1 & pids+=($!)

# 비전 CV 창은 origin/vision 에서 기본값이 false 로 바뀌었다. 시연에서 화면을 띄우려면
#   VISION_EXTRA="-p show_debug_window:=true" ROS_DOMAIN_ID=130 ./run_demo_pc.sh
echo "실행됨 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID). 로그: $LOG"
echo "  책 모델   $MODEL_PATH"
echo "  책장 모델 $SHELF_MODEL"
echo "  비전      tail -f $LOG/vision.log"
echo "  로봇팔    tail -f $LOG/manipulation.log"
wait
