#!/usr/bin/env bash
# 비전 담당이 Isaac 없이 인식을 시험할 수 있도록, 손목 카메라 토픽을 bag 으로 녹화한다 (GPU PC 에서 실행).
# 먼저 Isaac 을 띄워 "준비 완료" 를 확인할 것:
#   ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --headless --book-variants mixed --sensor-policy always --camera-hz 5
# 그다음:
#   ROS_DOMAIN_ID=130 ~/arm/tools/record_vision_bag.sh 40 ~/vision_bag/tray_full
set -u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 export 할 것}"
DUR="${1:-40}"
OUT="${2:-$HOME/vision_bag/tray_full}"
CAM_FRAME="${CAM_FRAME:-Camera_OmniVision_OV9782_Color}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "$(dirname "$OUT")"
rm -rf "$OUT"

# 먼저 녹화를 띄운다. 정적 TF 는 한 번만 발행되므로 녹화가 돌기 시작한 뒤에 띄워야 bag 에 들어간다
ros2 bag record -o "$OUT" --compression-mode file --compression-format zstd \
    /rgb /depth /camera_info /clock /tf /tf_static &
BAG_PID=$!
sleep 4

ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link \
    --ros-args -p use_sim_time:=true > /tmp/bag_tf_arm.log 2>&1 &
TF1=$!
# Isaac 카메라 prim TF 는 이미 광학 규약이라 이미지 frame(sim_camera) 과는 항등 변환이다
ros2 run tf2_ros static_transform_publisher --frame-id "$CAM_FRAME" --child-frame-id sim_camera \
    --ros-args -p use_sim_time:=true > /tmp/bag_tf_cam.log 2>&1 &
TF2=$!

cleanup() { kill "$TF1" "$TF2" 2>/dev/null; kill -INT "$BAG_PID" 2>/dev/null; wait "$BAG_PID" 2>/dev/null; }
trap cleanup EXIT INT TERM

echo "녹화 중 ${DUR}초 → $OUT"
sleep "$DUR"
cleanup
trap - EXIT
sleep 2
ros2 bag info "$OUT"
