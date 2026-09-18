#!/usr/bin/env bash
# Isaac 스크립트를 place_book_server 와 같은 환경으로 띄운다 (GPU PC). 기본은 학습용 촬영 스크립트.
#   ISAAC_ENTRY=~/arm/isaac/capture_spines.py ~/arm/isaac/run_capture.sh --views 60 --out ~/spine_ds
# ROS_DOMAIN_ID 는 호출한 쉘 값을 따른다. 없으면 팀 기본 130.
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
# 터미널에 시스템 ROS(/opt/ros/jazzy, Python 3.12)가 source 돼 있으면 Isaac(Python 3.11)이 그 rclpy 를 먼저 import 하다
# 죽는다 (2026-09-17 시연 준비 중 실제 발생). 시스템 ROS 경로를 걷어내고 Isaac 내장 jazzy 만 쓴다
strip_ros() { printf '%s' "$1" | tr ':' '
' | grep -v -e '^/opt/ros/' -e '/ros2_ws/install' -e '^$' | paste -sd: -; }
export PYTHONPATH="$(strip_ros "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_ros "${LD_LIBRARY_PATH:-}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
# Isaac 내장 jazzy 라이브러리 (시스템 ROS 는 Python 3.12 라 Isaac(3.11)에서 못 쓴다)
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
cd "$HOME/isaacsim"
exec ./python.sh "${ISAAC_ENTRY:-$HOME/arm/isaac/capture_spines.py}" "$@"
