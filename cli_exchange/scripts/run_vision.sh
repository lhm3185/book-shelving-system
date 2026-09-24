#!/bin/bash
# 비전 포함 저녁 경로 1회 (night/BASELINE_0922.md 6단계 그대로).
#   bash night/run_vision.sh <실행ID> "A=1 B=2"     (둘째 인자 = 출발점 조합 위에 얹을 스위치)
# 로그: night/runs/<ID>/{sim,vis,man,tf1,tf2,cyc}.log, env.txt
# 기본은 --gui (DISPLAY=:1). SIM_HEADLESS=1 이면 --headless.
ID=${1:?실행ID}; shift
R=~/b1_arm
D=$R/night/runs/$ID; mkdir -p "$D"
cd $R

# **앞 판이 끝난 뒤 충분히 띄우고 시작한다.** 2026-09-24 에 Isaac 이 기동 직후
# 죽는 일이 5회 있었고 전부 **연속 실행 중**이었다 (앞 판 종료 직후 바로 다음 판).
# 20초 이상 띄우면 재현되지 않는다. 10회 연속 기동 시험은 3~4초 간격에서 실패 0이었다.
# 원인은 아직 모른다 — 알 때까지는 간격으로 피한다.
BOOT_GAP=${SIM_BOOT_GAP:-20}
STAMP=$R/night/runs/.last_end
if [ -f "$STAMP" ]; then
  _wait=$(( BOOT_GAP - ( $(date +%s) - $(cat "$STAMP") ) ))
  if [ "$_wait" -gt 0 ]; then
    echo "앞 판 종료 후 ${_wait}초 더 기다린다 (SIM_BOOT_GAP=$BOOT_GAP)"
    sleep "$_wait"
  fi
fi

bash night/cleanup_demo.sh > "$D/cleanup.txt" 2>&1

export DISPLAY=${DISPLAY:-:1}
export ISAAC_SIM_PATH=${ISAAC_SIM_PATH:-$HOME/isaacsim}
export SIM_USD=${SIM_USD:-$HOME/Desktop/ing_library_env_v5.usd}
export SIM_TRAY_SETTLE_S=2.0 SIM_TRAY_DELIVERY_S=6.0
for kv in ${1:-}; do case $kv in NO_COMBO=*) export "$kv";; esac; done
if [ "${NO_COMBO:-0}" = 0 ]; then    # NO_COMBO=1 → 기본 스위치 (0-2 / F-1 회귀)
  export SIM_JOINT_SEGS=approach,carry_rotate,return SIM_GRIP_ROT90=1
  export SIM_GRASP_KINEMATIC=1 SIM_HAND_DRIFT_M=0.25 SIM_SPEED_SCALE=0.5 SIM_MAX_STEP=0.12
  # **콜리전 근사 교정** (2026-09-24 도윤님 승인, M1~M5 다섯 판 확인).
  # 레벨 파일은 안 고치고 런타임에 바로잡는다. 되돌리려면 이 세 줄을 지우거나
  # 둘째 인자로 빈 값을 덮어쓰면 된다 (예: "SIM_BOOK_COLL=").
  #   BOOK_COLL   둥근 표지의 볼록 껍질이 끌개라 꽂힌 책이 매번 다르게 2~3° 눕는다.
  #               상자로 두면 0.35~0.46° 로 모인다 (일곱 판 흔들림 0.3 mm)
  #   SHELF_COLL  서가 콜라이더가 convexDecomposition 이라 얇은 판에 복셀 한 겹
  #               (18 mm)이 붙는다. 서가는 정적이라 삼각망이 합법이다
  #   ROW_MEASURE 판 높이를 레벨에서 잰다 — `0.355 × 1.4` 의 출처가 코드에 없다
  export SIM_BOOK_COLL=boundingCube SIM_SHELF_COLL=none SIM_SHELF_ROW_MEASURE=1
fi
for kv in ${1:-}; do export "$kv"; done
env | grep -E '^SIM_' | sort > "$D/env.txt"

MODE=--gui; [ "${SIM_HEADLESS:-0}" != 0 ] && MODE=--headless

# **기동에 실패하면 다시 띄운다** (`SIM_BOOT_RETRY`, 기본 2회).
# Isaac 이 `SimulationApp(...)` 생성자 안 `libgpu.foundation.plugin` 에서 죽는 일이
# 2026-09-22~24 에 7회 있었다 (breakpad 스택, 우리 코드 밖이다). 원인은 못 고치지만
# **7회 모두 재시도 한 번에 떴다.** 발표 당일 한 번에 떠야 하므로 감싼다.
#   - 조용히 재시도하지 않는다. "한 번에 떴다" 와 "세 번 만에 떴다" 는 다른 사실이다
#   - 매 시도의 실패 지점을 night/boot_log.csv 에 쌓는다. 발표 뒤에 원인을 볼 자료다
BOOT_CSV=$R/night/boot_log.csv
[ -f "$BOOT_CSV" ] || echo "시각,실행ID,시도,성공,준비까지초,GPU_MiB,마지막줄" > "$BOOT_CSV"
_booted=0
for _try in $(seq 1 $(( ${SIM_BOOT_RETRY:-2} + 1 ))); do
  [ "$_try" -gt 1 ] && {
    echo "**기동 실패 — 다시 띄운다 ($_try/$(( ${SIM_BOOT_RETRY:-2} + 1 )))**"
    mv "$D/sim.log" "$D/sim.boot$(( _try - 1 )).log" 2>/dev/null
    bash night/cleanup_demo.sh > /dev/null 2>&1; sleep 10
  }
  _t0=$(date +%s)
  _gpu=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
  ./scripts/run_isaac_sim.sh $MODE --books 5 \
    --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color \
    --amr-test-overrides --drive-speed 0.6 ${SIM_EXTRA_ARGS:-} > "$D/sim.log" 2>&1 &
  echo $! > "$D/sim.pid"
  # 시뮬 준비(명령 대기) — 시간이 아니라 상태로 기다린다.
  # 죽었으면 180번을 다 세지 않는다 — 프로세스가 사라지면 바로 나온다.
  for i in $(seq 1 180); do
    grep -q '준비 완료' "$D/sim.log" && { _booted=1; break; }
    kill -0 "$(cat "$D/sim.pid")" 2>/dev/null || break
    sleep 2
  done
  echo "$(date -Is),$ID,$_try,$_booted,$(( $(date +%s) - _t0 )),${_gpu:-},\"$(tail -1 "$D/sim.log" | tr -d '\r' | cut -c1-70 | tr ',' ';')\"" >> "$BOOT_CSV"
  [ "$_booted" = 1 ] && break
done
[ "$_booted" = 1 ] || { echo "시뮬 준비 실패 ($(( ${SIM_BOOT_RETRY:-2} + 1 ))회 모두)"; tail -30 "$D/sim.log"; bash night/cleanup_demo.sh; exit 2; }
[ "$_try" -gt 1 ] && echo "**주의: $_try 번째 시도에 떴다** — night/boot_log.csv 참고" 

(
  unset PYTHONPATH LD_LIBRARY_PATH
  export ROS_DOMAIN_ID=130 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=$R/config/fastdds_local.xml
  . /opt/ros/jazzy/setup.bash; . ros2_ws/install/setup.bash
  ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link --ros-args -p use_sim_time:=true > "$D/tf1.log" 2>&1 &
  ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color --child-frame-id sim_camera --ros-args -p use_sim_time:=true > "$D/tf2.log" 2>&1 &
  P=ros2_ws/src/shelving_perception/resource
  ros2 run shelving_perception vision_manager --ros-args --params-file ros2_ws/src/shelving_perception/config/perception.yaml \
    -p model_path:=$P/book_tray_best.pt -p shelf_model_path:=$P/best.pt -p confidence_threshold:=0.75 ${VIS_EXTRA:-} > "$D/vis.log" 2>&1 &
  ros2 run shelving_manipulation manipulation_node --ros-args --params-file ros2_ws/src/shelving_manipulation/config/manipulation.yaml -p executor:=sim ${MAN_EXTRA:-} > "$D/man.log" 2>&1 &
  sleep 10
  timeout 300 ros2 topic pub -r 1 /perception/detect_request std_msgs/Bool "{data: true}" > /dev/null 2>&1 &
  sleep 5
  timeout ${CYC_TIMEOUT:-1500} python3 simulation/isaac/tools/full_cycle.py --speed 0.6 ${CYC_ARGS:-} > "$D/cyc.log" 2>&1
  echo "cycle rc=$?" >> "$D/cyc.log"
)
sleep 5
kill -INT $(cat "$D/sim.pid") 2>/dev/null; sleep 8
bash night/cleanup_demo.sh >> "$D/cleanup.txt" 2>&1
date +%s > "$R/night/runs/.last_end"      # 다음 판이 이만큼 띄우고 시작한다
grep -hE 'code=|error_code|RESULT|결과|성공|실패|rc=' "$D/cyc.log" | tail -8 | cut -c1-400
grep -hE '### (스위치|\[403추적\]|\[스윙\]|\[경로검사\])' "$D/sim.log" | cut -c1-300 | tail -20
