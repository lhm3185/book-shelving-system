#!/usr/bin/env bash
# 반납 한 사이클 (통합) — 시작점 → 주행 → 서가 두 단 스캔·빈칸 검출 → 책 검출·파지 → **비전 빈칸에** 삽입 → 시작점 복귀.
#
#   scripts/demo/full_cycle.sh                # Isaac(GUI) + 노드 전부 새로 띄우고 반납 작업 1건을 흘린다
#   SIM_LEVEL=<usd> scripts/demo/full_cycle.sh
#
# 흐름은 task_manager(FSM) 가 끌고 간다:
#   RECEIVE_TRAY → NAV_TO_RETURN(시뮬에선 이미 만족) → NAV_TO_SHELF(patrol, 검증된 경로)
#   → DETECT_TARGET_SLOT(manipulation_node 가 시뮬에 scan_shelf 를 보내 두 단을 훑고 비전 빈칸을 모은다)
#   → PLACE_BOOK(책은 관측 자세에서 비전으로 다시 잡고, 빈칸에 꽂는다) → RETURN_HOME(goto 출발 자리)
# 트레이는 **무인반납기에서 미끄러져 데크에 실린다** (레벨 트레이·책 그대로). 그러려면 로봇이 반납기 앞에
#   출발 자세는 **레벨에 고정된 그대로** 쓴다 (회전 없음, SIM_ROBOT_YAW 비움): 루트 (4.986,-5.607) yaw 90°,
#   팔 베이스 (4.986,-5.307) = 1차 시연 출발 자세. 데크 자리(팔 기준 칸 좌표 (-0.4748, 0.0788) 을 yaw 90 으로
#   돌린 것) = (4.907, -5.782) → 이송 목표(SIM_TRAY_TO), 반납기 트레이(y -5.66)에서
#   -x 로 미끄러져 데크에 앉는다 (오늘 실측 오차 2 mm). 도착해서는 시뮬 주행기가 제자리에서 yaw 0° 로 맞춘다
#   (nav_manager work_yaw_deg) — 1차 시연이 파지에 성공한 자리·각도 (2.535, -3.019, 0°).
#   녹화: logs/rec 에 REC_EVERY 스텝(기본 30 = 2 fps)마다 뷰포트 PNG — 스캔 모션을 눈으로 확인한다.
#   작업 위치에서는 manipulation_node 가 스캔 전에 베이스를 yaw 90° 로 돌려 서가를 팔 +Y 에 둔다.
# 물림축 90°(SIM_GRIP_ROT90=1): .4 팀 검증 조합. 꺼진 채로는 손가락이 책 옆면을 물어(폭 13.6 mm, 책 35 mm)
#   운반 중 기울고 upright/spine 검사에 실패했다 (2026-09-23 18:24 실행). .4 CLI 도 같은 결론.
# 홈(책 관측) 자세는 손목(관절 7)만 +90° 돌린다(SIM_HOME_J7_DEG): 트레이가 로봇 기준 90° 틀어져 실리는 레벨이라
#   카메라·물림축을 트레이에 맞춘다 (사용자 제안, 2026-09-23). 반대로 보이면 -90.
# 홈 자세를 10 cm 올린다(SIM_HOME_SHIFT): 그리퍼가 트레이 책 윗부분을 가렸다 (2026-09-23 스냅샷).
# 키네마틱 파지(SIM_GRASP_KINEMATIC=1): 마찰 파지는 판마다 미끄러짐이 달라 운반 중 3.1 cm 어긋남(406)이 났다 (2026-09-23 19:29).
#   .4 팀 검증 조합도 키네마틱 파지다. 마찰 파지 검증은 별도 과제.
# 속도 0.5 배(SIM_SPEED_SCALE): 기본 1.0 으로 돌리면 **빠져 나오는 구간(RETREATING)에서
#   관절 각속도가 URDF 한계의 100% 까지 올라가 403 이 난다** (2026-09-24 실측, 8스텝 초과).
#   책은 이미 제대로 꽂혔고 검사 일곱 개가 전부 통과한 뒤인데 작업만 실패로 보고된다.
#   0.5 는 우리 16판이 쓴 값이고 `PRESET_DEMO.speed_scale` 과 같다 — 문턱을 낮춘 게
#   아니라 **천천히 움직이는 것**이다. 403 검사(80%)는 그대로 둔다.
# 차체고정(SIM_FIX_BASE)은 끈다: 매 스텝 아티큘레이션 자세를 다시 쓰는 방식이라, 작업 위치에서 루트를 옮긴 뒤엔
#   파지 접근이 0.4 rad 남긴 채 멈췄다 (2026-09-23). 차체 질량 300 kg 은 그대로라 팔 반작용은 버틴다.
# 끝나도 Isaac·노드는 내리지 않는다 (화면으로 확인하려고).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# 2026-09-23 18:08 사용자가 고친 레벨 — 카트 시작 자세가 레벨에 고정돼 있다 (루트 (4.986,-5.607) yaw 90°, 팔 베이스 (4.986,-5.307))
LEVEL="${SIM_LEVEL:-/home/rokey/env_v5/Collected_ing_library_env_v5/ing_library_env_v5.usd}"
CAM=/World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color
# 책 원본: 레벨이 평탄화돼 참조가 없으므로 저장소의 책 USD 를 직접 준다 (book.usd 는 1바이트 깨진 파일)
BOOKS="${SIM_BOOKS:-/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_softcover_01_cover14.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_hardcover_01_cover62.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_softcover_01_cover59.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_hardcover_01_cover87.usdc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-129}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE-$REPO/config/fastdds_local.xml}"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
LOG="${LOG:-$REPO/logs}"; mkdir -p "$LOG"
SPEED="${SPEED:-0.5}"; JOB_DELAY="${JOB_DELAY:-25.0}"   # 소수점 필수 — 25 는 INTEGER 로 읽혀 노드가 죽는다
case "$(hostname)" in IsaacSim15|BryanKUBT) ;; *) echo "!!! 시연 PC 가 아님: $(hostname) — 중단"; exit 99;; esac
spawn() { setsid nohup "$@" < /dev/null & }
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/ros2_ws/install/setup.bash"; set -u

[ -f "$LEVEL" ] || { echo "**레벨이 없다**: $LEVEL"; exit 1; }
OLD=$(pgrep -f 'isaac/run_simulation' || true)
[ -n "$OLD" ] && { echo "남은 Isaac 종료: $OLD"; kill $OLD; sleep 5; }
pkill -f "install/shelving_(manipulation|perception|navigation|system)/" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
pkill -f "detect_request" 2>/dev/null || true
sleep 1
echo "[1/4] Isaac 시작 (약 3~4분) — 레벨 $(basename "$LEVEL")  $(date +%T)"
spawn env SIM_USD="$LEVEL" SIM_FIX_BASE="${SIM_FIX_BASE:-0}" SIM_GRIP_ROT90="${SIM_GRIP_ROT90:-1}" SIM_GRASP_KINEMATIC="${SIM_GRASP_KINEMATIC:-1}" SIM_HOME_J7_DEG="${SIM_HOME_J7_DEG:-90}" SIM_HOME_SHIFT="${SIM_HOME_SHIFT:-0,0,0.10}" SIM_ROBOT_YAW="${SIM_ROBOT_YAW:-}" SIM_TRAY_TO="${SIM_TRAY_TO:-4.907,-5.782,0.3365}" \
    SIM_SPEED_SCALE="${SIM_SPEED_SCALE:-0.5}" SIM_MAX_STEP="${SIM_MAX_STEP:-0.12}" \
    "$REPO/scripts/run_isaac_sim.sh" --gui --camera-prim "$CAM" --amr-test-overrides \
    --drive-speed "$SPEED" --record-dir "$LOG/rec" --record-every "${REC_EVERY:-30}" > "$LOG/isaac.log" 2>&1
echo -n "      준비 대기"
ready=0
for _ in $(seq 1 60); do
    grep -aq "준비 완료 (step" "$LOG/isaac.log" 2>/dev/null && { ready=1; break; }
    grep -aq "Traceback\|RuntimeError" "$LOG/isaac.log" 2>/dev/null && break
    echo -n "."; sleep 10
done
echo
[ "$ready" -eq 1 ] || { echo "**Isaac 이 준비되지 않았다.** $LOG/isaac.log 끝:"; grep -a "py stderr" "$LOG/isaac.log" | tail -8 | cut -c60-240; exit 1; }
echo "      준비 완료 $(date +%T)"
grep -aE "출발 자리|트레이 배치|책 [0-9]+권|레벨 책" "$LOG/isaac.log" | cut -c60-200 | tail -4

echo "[2/4] TF·비전·로봇팔·주행·FSM 노드  $(date +%T)"
ARM_BASE=$(ARM_ROBOT=franka python3 -c \
    "import sys; sys.path.insert(0, '$REPO/simulation/isaac/config'); \
     from robot_profiles import profile; print(profile().base_link.split('/')[-1])")
spawn ros2 run tf2_ros static_transform_publisher --frame-id "$ARM_BASE" \
    --child-frame-id arm_base_link --ros-args -p use_sim_time:=true > "$LOG/tf_arm.log" 2>&1
spawn ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color \
    --child-frame-id sim_camera --ros-args -p use_sim_time:=true > "$LOG/tf_cam.log" 2>&1
RES="$REPO/ros2_ws/install/shelving_perception/share/shelving_perception/resource"
spawn ros2 run shelving_perception vision_manager --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_perception/config/perception.yaml" \
    -p model_path:="${MODEL_PATH:-$RES/book_tray_best.pt}" \
    -p shelf_model_path:="${SHELF_MODEL:-$RES/best.pt}" \
    -p confidence_threshold:="${VISION_CONF:-0.75}" > "$LOG/vision.log" 2>&1
spawn ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_manipulation/config/manipulation.yaml" -p executor:=sim \
    -p slot_x_snap_to_gap:=false \
    > "$LOG/manipulation.log" 2>&1
spawn ros2 run shelving_navigation nav_manager --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_navigation/config/navigation.yaml" \
    -p waypoints_file:="$REPO/cli_exchange/config/waypoints_measured.yaml" > "$LOG/nav_manager.log" 2>&1
spawn ros2 run shelving_system task_manager_node --ros-args -p use_sim_time:=false -p book_profiles_path:="$REPO/cli_exchange/config/book_profiles_measured.yaml" > "$LOG/task_manager.log" 2>&1
sleep 10
# 검출 화면 창 — 책·빈칸 검출은 요청이 올 때만 그려지므로(스캔 정지점·책 관측 때) 창은 늘 떠 있어야 놓치지 않는다.
# 재시작 스크립트가 창을 닫고 다시 안 띄운 채 두 판을 돌렸다 (2026-09-23 19:09 지적).
spawn ros2 run rqt_image_view rqt_image_view /perception/debug_image > "$LOG/rqt_debug.log" 2>&1
spawn ros2 run rqt_image_view rqt_image_view /perception/depth_debug_image > "$LOG/rqt_depth.log" 2>&1
echo "[3/4] 반납 작업 1건 투입 (${JOB_DELAY}s 뒤)  $(date +%T)"
spawn ros2 run shelving_system return_machine_node --ros-args -p auto_publish:=true \
    -p publish_delay_sec:="$JOB_DELAY" > "$LOG/return_machine.log" 2>&1

echo "[4/4] 진행 상태 — 검출 좌표·단계 전이를 실시간으로 (같은 내용이 $LOG/detections.log 에 남는다)  $(date +%T)"
python3 - "$LOG" <<'PY'
# 화면에 **검출 좌표와 단계 전이**를 실시간으로 띄우고 logs/detections.log 에도 남긴다 (2026-09-23 요청).
import os, re, sys, time
LOG = sys.argv[1]
FILES = {
    "FSM":  (os.path.join(LOG, "task_manager.log"), re.compile(r"(->|Navigat.*(succeeded|failed)|Empty slot detected|Book-placement|COMPLETED|FAILED|Failed|error|Error)")),
    "비전": (os.path.join(LOG, "vision.log"),       re.compile(r"(Empty shelf target|Empty center pixel|Book picked|ROI 밖 책:|ROI 밖 책 \d)")),
    "조작": (os.path.join(LOG, "manipulation.log"), re.compile(r"(빈칸 선택|빈칸 관측 버림|빈 슬롯|파지|PlaceBook|베이스 회전|책 좌표|안전 범위)")),
    "시뮬": (os.path.join(LOG, "isaac.log"),        re.compile(r"(\[스윕\]|스윕 시작|관측 자세|서가 앞면|:scan (FAILED|SUCCEEDED)|작업 job_[0-9a-f]+ (FAILED|SUCCEEDED)|\[JUDGE\]|배치 확인|주행\] (도착|회전 끝)|실측 \[)")),
}
strip = re.compile(r"^\[[A-Z]+\] \[[0-9.]+\] \[[a-z_]+\]: |^.*\[py stderr\]: ### ")
seen = {k: 0 for k in FILES}
out = open(os.path.join(LOG, "detections.log"), "a")
t0 = time.monotonic(); done = False
while time.monotonic() - t0 < 1500 and not done:
    for tag, (path, pat) in FILES.items():
        try:
            lines = open(path, errors="ignore").read().splitlines()
        except FileNotFoundError:
            continue
        for l in lines[seen[tag]:]:
            if pat.search(l) and "XMLProfile" not in l and "feedback" not in l:
                m = strip.sub("", l)[:220]
                line = f"{time.strftime('%H:%M:%S')} [{tag}] {m}"
                print("  ", line, flush=True); out.write(line + "\n"); out.flush()
                if tag == "FSM" and ("COMPLETED -> IDLE" in l or "-> FAILED" in l or "FAILED ->" in l):
                    done = True
        seen[tag] = len(lines)
    time.sleep(1)
if not done:
    print("  (25분 경과 — 감시 종료, 노드는 계속 떠 있다)")
PY
echo "끝 $(date +%T) — 로그: $LOG (Isaac·노드는 그대로 떠 있다)"
