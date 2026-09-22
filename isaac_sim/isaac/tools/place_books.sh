#!/usr/bin/env bash
# 연동 시연 — PlaceBook 목표를 차례로 보낸다 (1차 고정 칸 4곳). run_demo_pc.sh 가 떠 있어야 한다.
#   ROS_DOMAIN_ID=130 ./place_books.sh 2        # 2권
set -u  # ROS setup.bash 는 미정의 변수를 써서 source 앞뒤로 +u/-u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 export 할 것}"
N="${1:-4}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${WS:-$REPO_ROOT/ros2_ws}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
XS=(-0.3497 -0.4297 -0.5097 -0.2697)
for i in $(seq 0 $((N - 1))); do
  x=${XS[$i]}; job="demo_$(date +%H%M%S)_$i"
  echo "== $job : 트레이 다음 칸 → 서가 x=$x"
  ros2 action send_goal --feedback /place_book shelving_interfaces/action/PlaceBook \
    "{job_id: $job, book_id: book_$i, target_slot: {header: {frame_id: arm_base_link}, pose: {position: {x: $x, y: 0.5495, z: 0.3399}, orientation: {z: 0.7071068, w: 0.7071068}}, confidence: 1.0}}" \
    | grep -E "phase:|success|error_code|message|status:" | uniq
done
