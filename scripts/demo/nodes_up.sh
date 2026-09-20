#!/usr/bin/env bash
# ② 이 PC 의 ROS 노드 (비전 + 로봇팔). Isaac 이 '준비 완료' 를 낸 뒤에 띄운다.
#
#   ./scripts/demo/nodes_up.sh                 # 그냥 실행
#   SHOW_VISION=1 ./scripts/demo/nodes_up.sh   # 비전 CV 창까지
#
# **Isaac 을 다시 띄웠으면 이것도 반드시 다시 띄운다** (2026-09-18 확인된 규칙).
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
say "### 도메인 $ROS_DOMAIN_ID 로 노드를 띄운다 (Ctrl+C 로 전부 종료)"
[ -n "${SHOW_VISION:-}" ] && export VISION_EXTRA="-p show_debug_window:=true"
exec "$REPO_ROOT/simulation/isaac/tools/run_demo_pc.sh"
