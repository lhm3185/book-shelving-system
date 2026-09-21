#!/usr/bin/env bash
# 한 바퀴 돌고 → 돌아온 자리에서 비전으로 책을 찾아 → 집어서 꽂는다.
#
#   ./scripts/demo/patrol_and_pick.sh                 # 전부 (Isaac 포함, 약 5분)
#   ./scripts/demo/patrol_and_pick.sh --keep-sim      # 떠 있는 Isaac 을 그대로 쓴다 (빠름)
#   ./scripts/demo/patrol_and_pick.sh --no-patrol     # 주행 없이 파지만
#   ./scripts/demo/patrol_and_pick.sh --speed 0.8
#
# **이 스크립트는 GPU PC(10.10.0.1)에서 돌린다.** Isaac·비전·로봇팔이 한 PC 에 있으면
# 도메인만 맞으면 되고, 따로 띄울 터미널이 줄어든다.
#
# 끝나면 Ctrl+C 로 전부 내린다 (Isaac 포함).
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LEVEL="${SIM_LEVEL:-$HOME/Desktop/assets/level/ing_library_env_v4.usd}"
CAM=/World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-129}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# **화이트리스트를 쓰지 않는다.** 이 스크립트는 한 PC 안에서 Isaac·비전·로봇팔을
# 다 돌리는데, 화이트리스트는 내장 전송(공유메모리·로컬호스트)을 꺼서
# **같은 PC 의 노드끼리 서로를 못 찾게 만든다** (2026-09-21 실측: .1 안에서
# /manipulation/sim/command 가 안 보였다).
# 다른 PC 에서 붙어야 하면 FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml 로 준다.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE-$REPO/config/fastdds_local.xml}"
LOG="${LOG:-/tmp/b1_demo}"; mkdir -p "$LOG"

KEEP_SIM=0; DO_PATROL=1; SPEED=0.5; GOAL_X=""
while [ $# -gt 0 ]; do
    case "$1" in
        --keep-sim)  KEEP_SIM=1 ;;
        --no-patrol) DO_PATROL=0 ;;
        --speed)     SPEED="$2"; shift ;;
        --goal-x)    GOAL_X="$2"; shift ;;
        *) echo "모르는 인자: $1"; exit 2 ;;
    esac
    shift
done

pids=()
cleanup() {
    echo; echo "정리 중..."
    # **우리가 띄운 PID 만** 죽인다. pkill 패턴은 자기 명령줄까지 잡아 셸을 죽인 적이 있다
    for p in "${pids[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    sleep 2
    for p in "${pids[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    echo "정리 끝"
}
trap cleanup INT TERM EXIT

set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/ros2_ws/install/setup.bash"; set -u

# ------------------------------------------------------------------ 1. Isaac
if [ "$KEEP_SIM" -eq 0 ]; then
    [ -f "$LEVEL" ] || { echo "**레벨이 없다: $LEVEL**"; exit 1; }
    OLD=$(pgrep -f 'isaac/run_simulation' || true)
    [ -n "$OLD" ] && { echo "남은 Isaac 종료: $OLD"; kill $OLD; sleep 5; }
    echo "[1/4] Isaac 시작 (약 4분) — 레벨 $(basename "$LEVEL")"
    ( cd "$REPO" && SIM_USD="$LEVEL" ./scripts/run_isaac_sim.sh --gui \
        --camera-prim "$CAM" --amr-test-overrides --drive-speed "$SPEED" \
        > "$LOG/isaac.log" 2>&1 ) &
    pids+=($!)
else
    echo "[1/4] 떠 있는 Isaac 을 그대로 쓴다"
fi

# 준비 확인. **떠 있는 Isaac 을 쓸 때는 로그 파일이 우리 것이 아니다** —
# 그때는 ROS 로 확인한다 (시뮬 실행기가 명령 토픽을 구독하고 있으면 준비된 것이다)
echo -n "      준비 대기"
ready=0
for _ in $(seq 1 60); do
    if [ "$KEEP_SIM" -eq 0 ] && grep -aq "준비 완료 (step" "$LOG/isaac.log" 2>/dev/null; then
        ready=1; break
    fi
    if timeout 5 ros2 topic info /manipulation/sim/command --no-daemon 2>/dev/null \
            | grep -q "Subscription count: [1-9]"; then
        ready=1; break
    fi
    echo -n "."; sleep 10
done
echo
[ "$ready" -eq 1 ] || {
    echo "**Isaac 이 준비되지 않았다.** ${LOG}/isaac.log (또는 떠 있는 시뮬의 로그)를 볼 것"
    exit 1; }
echo "      준비 완료"

# ------------------------------------------------------------------ 2. 비전·로봇팔 노드
echo "[2/4] 비전·로봇팔 노드"
ARM_BASE=$(ARM_ROBOT=franka python3 -c \
    "import sys; sys.path.insert(0, '$REPO/simulation/isaac/config'); \
     from robot_profiles import profile; print(profile().base_link.split('/')[-1])")
[ -n "$ARM_BASE" ] || { echo "팔 기준 프레임을 못 구했다"; exit 1; }
echo "      팔 기준 프레임: $ARM_BASE → arm_base_link"
ros2 run tf2_ros static_transform_publisher --frame-id "$ARM_BASE" --child-frame-id arm_base_link \
    --ros-args -p use_sim_time:=true > "$LOG/tf_arm.log" 2>&1 & pids+=($!)
ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color \
    --child-frame-id sim_camera --ros-args -p use_sim_time:=true > "$LOG/tf_cam.log" 2>&1 & pids+=($!)

RES="$REPO/ros2_ws/install/shelving_perception/share/shelving_perception/resource"
ros2 run shelving_perception vision_manager --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_perception/config/perception.yaml" \
    -p model_path:="${MODEL_PATH:-$RES/book_tray_best.pt}" \
    -p shelf_model_path:="${SHELF_MODEL:-$RES/best.pt}" \
    -p confidence_threshold:="${VISION_CONF:-0.75}" \
    > "$LOG/vision.log" 2>&1 & pids+=($!)
ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_manipulation/config/manipulation.yaml" -p executor:=sim \
    > "$LOG/manipulation.log" 2>&1 & pids+=($!)
sleep 8

# 검출 요청을 계속 보낸다 — 비전은 요청이 있을 때만 검출한다
ros2 topic pub -r 1 /perception/detect_request std_msgs/Bool "{data: true}" \
    > "$LOG/trigger.log" 2>&1 & pids+=($!)
# 검출 화면 (실패해도 시연은 계속한다)
ros2 run rqt_image_view rqt_image_view /perception/debug_image \
    > "$LOG/rqt.log" 2>&1 & pids+=($!)
echo "      비전 창: /perception/debug_image (노란 ROI · 초록 상자 · 빨간 점)"
sleep 5

# ------------------------------------------------------------------ 3. 순회
if [ "$DO_PATROL" -eq 1 ]; then
    echo "[3/4] 도서관 한 바퀴 (약 $(python3 -c "print(int(32.04/$SPEED))")초)"
    python3 "$REPO/simulation/isaac/tools/patrol.py" --speed "$SPEED" || {
        echo "**주행 실패** — 여기서 멈춘다"; exit 1; }
else
    echo "[3/4] 주행 건너뜀"
fi

# ------------------------------------------------------------------ 4. 파지·반납
echo "[4/4] 비전 좌표로 파지·반납"
ARGS=()
[ -n "$GOAL_X" ] && ARGS+=(--goal-x "$GOAL_X")
# **":-" 를 붙이지 않는다.** 빈 배열에 붙이면 빈 문자열 하나로 펼쳐져
# pick_from_vision.py 가 "unrecognized arguments: " 로 죽는다 (bash 4.4+ 는 그냥 안전하다)
python3 "$REPO/simulation/isaac/tools/pick_from_vision.py" "${ARGS[@]}"
RC=$?

echo
[ "$RC" -eq 0 ] && echo "**끝. 한 바퀴 돌고 파지·반납까지 성공했다**" \
               || echo "**파지 실패 (코드 $RC)** — $LOG/manipulation.log 를 볼 것"
echo "로그: $LOG"
echo "창을 닫으려면 Ctrl+C"
wait
