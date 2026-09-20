#!/usr/bin/env bash
# ③ 명령을 보내기 전에 **연결이 됐는지** 본다. 여기서 막히면 goal 을 보내도 소용없다.
#
#   ./scripts/demo/status.sh
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
ros_env
say "### 도메인 $ROS_DOMAIN_ID"
if ros2 action list 2>/dev/null | grep -q place_book; then
    say "액션 서버 /place_book  OK"
else
    say "액션 서버 /place_book  **없음**"
    say "  1) Isaac 터미널에 '준비 완료' 가 떴는가"
    say "  2) nodes_up.sh 가 떠 있는가"
    say "  3) 세 터미널 모두 ROS_DOMAIN_ID=$ROS_DOMAIN_ID 인가 (export 는 터미널·ssh 를 넘지 않는다)"
    exit 1
fi
say "--- 시뮬 상태 (트레이 책 목록) ---"
timeout 10 ros2 topic echo /manipulation/sim/state --once 2>/dev/null | head -14 \
    || say "  /manipulation/sim/state 를 못 받았다 — Isaac 쪽을 확인할 것"
