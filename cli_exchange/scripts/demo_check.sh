#!/bin/bash
# **눈으로 확인하는 판** — rqt 창을 띄우고, Enter 를 칠 때까지 기다리고, 끝나도 안 끈다.
#
#   bash cli_exchange/scripts/demo_check.sh [실행ID] ["추가 스위치"]
#
# 기본은 비전이 찾은 빈칸으로 꽂는 시연 경로(`SIM_DETECT_SLOT=1`)다.
#
# 뜨는 창 (토픽 이름은 `shelving_perception/config/perception.yaml` 그대로):
#   /perception/debug_image        비전이 **판단을 그려서** 내놓는 것  ← 확인은 여기부터
#   /perception/depth_debug_image  깊이 시각화
#   rqt_console                    로그 (410·409 메시지가 여기 뜬다)
#
# 원본 카메라도 보려면:  RQT="debug,depth,rgb,raw_depth,console" bash …/demo_check.sh
#
# 흐름:
#   ① 시뮬이 뜬다 (실패하면 자동 재시도)
#   ② 노드·비전이 붙고 rqt 창이 뜬다
#   ③ **Enter 를 칠 때까지 기다린다** — 창 배치하고 토픽 고를 시간
#   ④ 풀사이클 한 번
#   ⑤ **끄지 않는다.** 결과를 눈으로 보고, 끝나면 찍어 준 kill 명령으로 끈다
set -u
ID=${1:-CHECK$(date +%H%M)}
EXTRA=${2:-SIM_DETECT_SLOT=1}
R=~/b1_arm

cd "$(dirname "$0")/../.." 2>/dev/null || true
echo "==============================================================="
echo "  확인용 판  ID=$ID"
echo "  스위치     $EXTRA"
echo "  rqt        ${RQT:-debug,depth,console}"
echo "  DISPLAY    ${DISPLAY:-:1}   (GPU PC 는 :1 이다. 창이 안 뜨면 여기를 먼저 본다)"
echo "==============================================================="
command -v rqt >/dev/null 2>&1 || {
  echo "**rqt 가 없다.** 먼저 설치한다:"
  echo "    sudo apt update && sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-common-plugins"
  echo "  (설치 없이도 판은 돌지만 창이 안 뜬다)"
}
exec env SIM_RQT="${RQT:-debug,depth,console}" SIM_PAUSE=1 SIM_KEEP=1 \
     bash "$R/night/run_vision.sh" "$ID" "$EXTRA"
