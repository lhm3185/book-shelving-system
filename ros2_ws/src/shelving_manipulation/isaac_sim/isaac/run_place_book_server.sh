#!/usr/bin/env bash
# Isaac 작업 실행기를 ROS2 환경과 함께 띄운다 (GPU PC).
# ROS_DOMAIN_ID 는 호출한 쉘 값을 따른다. 없으면 팀 기본 130.
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
# Isaac 내장 jazzy 라이브러리 (시스템 ROS 는 Python 3.12 라 Isaac(3.11)에서 못 쓴다)
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
cd "$HOME/isaacsim"
exec ./python.sh "$HOME/arm/isaac/place_book_server.py" "$@"
