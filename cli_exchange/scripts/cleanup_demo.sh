#!/bin/bash
# 데모·시뮬 프로세스 정리 (pkill 금지 대체). 패턴을 이 파일 안에만 두어 호출한 셸이 잡히지 않게 한다.
# 주의: 이 패턴의 단어를 다른 셸 명령줄에 직접 쓰면 그 셸도 잡힌다.
PAT='run_simulation\.py|manipulation_node|vision_manager|static_transform_publisher|rqt_image_view|scripts/demo/patrol_and_pick\.sh'
list() { pgrep -u "$(id -u)" -f "$PAT" | grep -vx "$$" | grep -vx "$PPID"; }
for sig in INT TERM KILL; do
  pids=$(list); [ -z "$pids" ] && break
  kill -$sig $pids 2>/dev/null; sleep 6
done
left=$(list); [ -z "$left" ] && echo "잔여 없음" || ps -o pid,cmd -p $left | cut -c1-120
