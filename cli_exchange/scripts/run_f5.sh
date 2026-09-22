#!/bin/bash
# F-5 민감도: demo_best 에서 SIM_GOAL_Y 만 바꿔 한 판씩
cd ~/b1_arm
for g in "$@"; do
  bash night/run_vision.sh v_gy$g "SIM_PICK_Y=-3.049 SIM_GOAL_Y=$g SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing SIM_TRACE_CARRY=1 SIM_TRACE_PHASE=1 SIM_TRACE_BASE=1" > night/runs/v_gy$g.out 2>&1
  echo "$g $(date +%H:%M)"
done
