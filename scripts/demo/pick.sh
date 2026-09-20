#!/usr/bin/env bash
# ④ 책 한 권 명령. 좌표는 **프로파일에서** 온다 (레벨과 좌표를 따로 넘기지 않는다).
#
#   ./scripts/demo/pick.sh                          # 프로파일 기본 좌표
#   BOOK_ID=book_1 ./scripts/demo/pick.sh           # 다른 책
#   PROFILE=shelf ./scripts/demo/pick.sh            # 서가 삽입 좌표로
#   GOAL_X=-0.3897 ./scripts/demo/pick.sh           # 칸만 바꿔서
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
ros_env
JOB_ID="${JOB_ID:-$(date +%H%M%S)}"
BOOK_ID="${BOOK_ID:-book_0}"
say "### 프로파일 $PROFILE / job $JOB_ID / $BOOK_ID → ($GOAL_X, $GOAL_Y, $GOAL_Z)"
# frame_id 는 **반드시 arm_base_link**. 다른 프레임은 410 으로 거절된다
# (결합체에서 base_link 는 AMR 몸체라 팔 기준과 0.655 m 어긋난다)
exec ros2 action send_goal /place_book shelving_interfaces/action/PlaceBook \
"{job_id: '$JOB_ID', book_id: '$BOOK_ID', target_slot: {header: {frame_id: 'arm_base_link'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {z: 0.7071068, w: 0.7071068}}, confidence: 1.0}}" \
${FEEDBACK:+--feedback}
