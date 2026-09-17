#!/usr/bin/env bash
# AMR 라이다 → 2D 스캔맵 파이프라인 시험 (이 PC). GPU PC 에서 run_demo_gpu.sh --amr-test-overrides 가 떠 있어야 한다.
#   ROS_DOMAIN_ID=130 ./lidar_scanmap_test.sh 30      # 30초 동안 지도를 만들고 저장
# 3D 32채널 포인트클라우드 → 라이다 높이 기준 바닥 위~1 m 만 남긴 LaserScan → slam_toolbox → map 저장
DUR="${1:-30}"
OUT="${OUT:-/tmp/b1_demo/scanmap}"; mkdir -p "$OUT"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
source /opt/ros/jazzy/setup.bash
cleanup() {
  pkill -f "pointcloud_to_laserscan_node --ros-args -r cloud_in:=/point_cloud" 2>/dev/null
  pkill -f "async_slam_toolbox_node --ros-args -p use_sim_time:=true" 2>/dev/null
}
trap cleanup EXIT
# 라이다는 바닥에서 0.233 m → Lidar 프레임 z −0.233 이 바닥. 바닥 +5 cm ~ 1 m 만 장애물로 쓴다
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/point_cloud -r scan:=/scan -p use_sim_time:=true \
  -p min_height:=-0.18 -p max_height:=0.77 -p range_min:=1.0 -p range_max:=20.0 \
  -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.00436 -p scan_time:=0.072 -p use_inf:=true \
  > "$OUT/p2l.log" 2>&1 &
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true -p odom_frame:=odom -p map_frame:=map -p base_frame:=base_link \
  -p scan_topic:=/scan -p mode:=mapping -p resolution:=0.05 -p max_laser_range:=20.0 \
  -p minimum_travel_distance:=0.0 -p minimum_travel_heading:=0.0 -p map_update_interval:=1.0 \
  > "$OUT/slam.log" 2>&1 &
# Jazzy slam_toolbox 는 lifecycle 노드 — configure·activate 를 해야 /map 이 나온다 (안 하면 unconfigured 로 조용히 대기)
sleep 5
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
echo "지도 작성 ${DUR}s ..."; sleep "$DUR"
ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p use_sim_time:=true -p save_map_timeout:=5.0 > "$OUT/saver.log" 2>&1
ls -la "$OUT"/map.* 2>/dev/null || { echo "지도 저장 실패 — $OUT/*.log 확인"; tail -5 "$OUT/slam.log"; }
