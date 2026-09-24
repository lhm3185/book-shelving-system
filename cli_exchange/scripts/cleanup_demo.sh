#!/bin/bash
# 데모·시뮬 프로세스 정리 (pkill 금지 대체). 패턴을 이 파일 안에만 두어 호출한 셸이 잡히지 않게 한다.
# 주의: 이 패턴의 단어를 다른 셸 명령줄에 직접 쓰면 그 셸도 잡힌다.
PAT='run_simulation\.py|manipulation_node|vision_manager|static_transform_publisher|rqt_image_view|scripts/demo/patrol_and_pick\.sh'
# **조상 전부를 제외한다.** 예전에는 자기($$)와 부모($PPID)만 뺐는데, 배치 셸처럼
# **할아버지**가 잡히면 재시도 경로가 자기를 띄운 셸을 죽인다 (2026-09-24: 그 탓에
# SIM_BOOT_RETRY 가 한 번도 못 돌았다 — 필요한 순간에만 안 도는 구조였다).
ancestors() {
  local pid=$$
  while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
    echo "$pid"
    pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
  done
}
list() {
  local skip; skip=$(ancestors)
  pgrep -u "$(id -u)" -f "$PAT" | grep -vxF "$skip"
}
for sig in INT TERM KILL; do
  pids=$(list); [ -z "$pids" ] && break
  kill -$sig $pids 2>/dev/null; sleep 6
done
left=$(list); [ -z "$left" ] && echo "잔여 없음" || ps -o pid,cmd -p $left | cut -c1-120
