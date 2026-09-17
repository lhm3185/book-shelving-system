#!/usr/bin/env bash
# 연동 시연 — GPU PC 쪽: 레벨 v5(카메라·라이다 탑재 로봇, 9/17 저녁 레벨 디자인 수정본) + 로봇팔 작업 실행기 + 카메라 TF·/clock 보충.
#   ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh            # GPU PC 모니터에 Isaac 창
#   ROS_DOMAIN_ID=130 ~/arm/isaac/run_demo_gpu.sh --headless  # 창 없이
# 원격(ssh)에서 창을 GPU PC 모니터에 띄울 때 DISPLAY·XAUTHORITY 를 잡아준다.
set -u  # ROS setup.bash 는 미정의 변수를 써서 source 앞뒤로 +u/-u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 이 PC 와 같은 값으로 export 할 것}"
LEVEL="${LEVEL:-$HOME/Desktop/ing_library_env_v5.usd}"
CAMERA_PRIM="${CAMERA_PRIM:-/World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color}"
GUI=(--gui)
if [ "${1:-}" = "--headless" ]; then GUI=(); shift; fi
if [ ${#GUI[@]} -gt 0 ] && [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:1
  export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
fi
exec "$HOME/arm/isaac/run_place_book_server.sh" --usd "$LEVEL" --camera-prim "$CAMERA_PRIM" "${GUI[@]}" "$@"
