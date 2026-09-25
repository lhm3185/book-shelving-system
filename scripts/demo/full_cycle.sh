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
# **레벨이 그 자리에 없으면 이 PC 의 자리를 본다.** 아침에 도윤님이 환경변수 없이
# 이 스크립트 한 줄로 띄울 수 있어야 한다 — 검증된 값이 실행 셸에만 살아 있으면
# 안 된다는 것을 2026-09-24 밤에 여섯 번 겪었다. 원래 경로가 있으면 그대로 쓴다.
[ -f "$LEVEL" ] || for _c in "$HOME/levels/Final_Level_Library/Final_Level_Library.usdc"; do
    [ -f "$_c" ] && { LEVEL="$_c"; echo "레벨 대체: $LEVEL"; break; }
done
CAM=/World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color
# 책 원본: 레벨이 평탄화돼 참조가 없으므로 저장소의 책 USD 를 직접 준다 (book.usd 는 1바이트 깨진 파일)
BOOKS="${SIM_BOOKS:-/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_softcover_01_cover14.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_hardcover_01_cover62.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_softcover_01_cover59.usdc,/home/rokey/b1_work/simulation/assets/book_dataset/usd_v2/decorative_book_set_01_2k__book_hardcover_01_cover87.usdc}"
# 책도 같은 이유로 — 원본이 없으면 **이 저장소의 usd_v2** 에서 같은 네 권을 쓴다
if ! [ -f "${BOOKS%%,*}" ]; then
    _bd="$REPO/simulation/assets/book_dataset/usd_v2"
    _b=""
    for _n in decorative_book_set_01_2k__book_softcover_01_cover14 \
              decorative_book_set_01_2k__book_hardcover_01_cover62 \
              decorative_book_set_01_2k__book_softcover_01_cover59 \
              decorative_book_set_01_2k__book_hardcover_01_cover87; do
        [ -f "$_bd/$_n.usdc" ] && _b="${_b:+$_b,}$_bd/$_n.usdc"
    done
    [ -n "$_b" ] && { BOOKS="$_b"; echo "책 대체: 저장소 usd_v2 4권"; }
fi
# 트레이 출발/도착 — 이 레벨의 실측값. 출발은 레벨의 트레이 자리, 도착은 데크 위
# 같은 z. 예전 기본값(4.907,-5.782)은 다른 레벨 것이라 이 레벨에서 트레이가 카트를 민다.
export SIM_TRAY_FROM="${SIM_TRAY_FROM:-5.817607391996635,-5.659500598907469,0.333765}"
export SIM_TRAY_TO="${SIM_TRAY_TO:-4.993110179901123,-5.659414291381836,0.333765}"
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
# **이전 판을 확실히 내린 뒤 띄운다.** 예전에는 Isaac 만 kill 하고 5 초 기다린 게 전부라
# ① Isaac 이 안 죽으면 두 판이 겹쳐 돌았고 ② rqt 창은 아예 안 껐다 — 판마다 두 개씩
# 쌓여 어느 창이 이번 판 것인지 알 수 없었다 (2026-09-24 지적, 그때 8 개 떠 있었다).
OLD=$(pgrep -f 'isaac/run_simulation' || true)
if [ -n "$OLD" ]; then
    echo "남은 Isaac 종료: $OLD"; kill $OLD 2>/dev/null || true
    for _ in $(seq 20); do kill -0 $OLD 2>/dev/null || break; sleep 1; done
    STILL=""; for pid in $OLD; do kill -0 "$pid" 2>/dev/null && STILL="$STILL $pid"; done
    [ -n "$STILL" ] && { echo "      안 죽어 강제 종료:$STILL"; kill -9 $STILL 2>/dev/null || true; sleep 2; }
fi
# **셸은 거른다.** `pgrep -f rqt_image_view` 는 그 문자열을 명령줄에 담은 셸까지
# 잡는다 — 21:09 판에서 "5 개" 를 껐는데 그 중 하나가 창을 세어 보던 우리 셸이었다
# (exit 144). 실제 창은 python 이므로 comm 이 셸인 PID 를 빼고, 우리 자신도 뺀다.
RQT=""
for pid in $(pgrep -f rqt_image_view || true); do
    [ "$pid" = "$$" ] && continue
    case "$(cat /proc/$pid/comm 2>/dev/null)" in bash|sh|dash|zsh|"") continue;; esac
    RQT="$RQT $pid"
done
if [ -n "$RQT" ]; then
    echo "남은 rqt 창 종료:$(echo $RQT | wc -w) 개"
    kill $RQT 2>/dev/null || true; sleep 2
    for pid in $RQT; do kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null || true; done
fi
pkill -f "install/shelving_(manipulation|perception|navigation|system)/" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
pkill -f "detect_request" 2>/dev/null || true
sleep 1
# **화면 녹화를 켤 때는 REC_EVERY=0 으로 이걸 끈다.** 뷰포트 PNG 를 2 fps 로 쓰는 것이
# 시뮬을 느리게 하고, 삽입 작업이 제한의 87~88 % 를 이미 쓰고 있어 그 차이가 타임아웃이
# 된다 (2026-09-25 07:36). 판 하나가 16 GB 이기도 하다 — 화면 녹화가 있으면 중복이다.
if [ "${REC_EVERY:-30}" = "0" ]; then
    REC_ARGS=""; echo "      (뷰포트 녹화 끔 — REC_EVERY=0)"
else
    REC_ARGS="--record-dir $LOG/rec --record-every ${REC_EVERY:-30}"
    # **가까이서 보고 싶을 때** 녹화 카메라를 옮긴다 (REC_EYE / REC_LOOK, "x y z").
    # 기본 뷰는 방 전체라 로봇이 화면의 8% 밖에 안 돼 팔이 책을 스치는지 안 보인다
    # (2026-09-25 네 권 예행 실측). 서가 책은 콜리전이 없어 **눈이 유일한 검사**다.
    [ -n "${REC_EYE:-}" ]  && REC_ARGS="$REC_ARGS --record-eye $REC_EYE"
    [ -n "${REC_LOOK:-}" ] && REC_ARGS="$REC_ARGS --record-look $REC_LOOK"
fi
echo "[1/4] Isaac 시작 (약 3~4분) — 레벨 $(basename "$LEVEL")  $(date +%T)"
spawn env SIM_USD="$LEVEL" SIM_FIX_BASE="${SIM_FIX_BASE:-0}" SIM_GRIP_ROT90="${SIM_GRIP_ROT90:-1}" SIM_GRASP_KINEMATIC="${SIM_GRASP_KINEMATIC:-1}" SIM_HOME_J7_DEG="${SIM_HOME_J7_DEG:-90}" SIM_HOME_SHIFT="${SIM_HOME_SHIFT:-0,0,0.10}" SIM_ROBOT_YAW="${SIM_ROBOT_YAW:-}" SIM_TRAY_TO="${SIM_TRAY_TO:-4.907,-5.782,0.3365}" \
    SIM_SPEED_SCALE="${SIM_SPEED_SCALE:-0.5}" SIM_MAX_STEP="${SIM_MAX_STEP:-0.12}" \
    SIM_CARRY_MODE="${SIM_CARRY_MODE:-swing}" \
    "$REPO/scripts/run_isaac_sim.sh" --gui --camera-prim "$CAM" --amr-test-overrides \
    --drive-speed "$SPEED" $REC_ARGS > "$LOG/isaac.log" 2>&1
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
    ${MAN_EXTRA:-} > "$LOG/manipulation.log" 2>&1
spawn ros2 run shelving_navigation nav_manager --ros-args \
    --params-file "$REPO/ros2_ws/src/shelving_navigation/config/navigation.yaml" \
    -p waypoints_file:="$REPO/cli_exchange/config/waypoints_measured.yaml" > "$LOG/nav_manager.log" 2>&1
# shelf_map 도 실측 사본으로 (서가 B 관측 자세). shelf_01 은 원본과 접두어가 같아 두 권 흐름은 안 바뀐다.
spawn ros2 run shelving_system task_manager_node --ros-args -p use_sim_time:=false -p book_profiles_path:="$REPO/cli_exchange/config/book_profiles_measured.yaml" -p shelf_map_path:="${SIM_SHELF_MAP:-$REPO/cli_exchange/config/shelf_map_measured.yaml}" > "$LOG/task_manager.log" 2>&1
sleep 10
# 검출 화면 창 — 책·빈칸 검출은 요청이 올 때만 그려지므로(스캔 정지점·책 관측 때) 창은 늘 떠 있어야 놓치지 않는다.
# 재시작 스크립트가 창을 닫고 다시 안 띄운 채 두 판을 돌렸다 (2026-09-23 19:09 지적).
spawn ros2 run rqt_image_view rqt_image_view /perception/debug_image > "$LOG/rqt_debug.log" 2>&1
spawn ros2 run rqt_image_view rqt_image_view /perception/depth_debug_image > "$LOG/rqt_depth.log" 2>&1
echo "[3/4] 반납 작업 1건 투입 (${JOB_DELAY}s 뒤)  $(date +%T)"
# **두 권.** 기본 `book_ids` 가 `['book_001']` 한 건이라 한 권 꽂고 복귀했다.
# FSM 은 이미 반복할 줄 안다 (`_current_task_index += 1` → SELECT_BOOK 부터 다시).
# 분류코드는 **둘 다 0~4 로 시작**해야 한다 — shelf_map 에서 그 범위만 shelf_01 이고,
# 5~9 를 주면 shelf_02 로 가는데 그 observation_pose 는 아직 자리표시자(3.0, +1.0)다.
spawn ros2 run shelving_system return_machine_node --ros-args -p auto_publish:=true \
    -p publish_delay_sec:="$JOB_DELAY" \
    -p book_ids:="${SIM_BOOK_IDS:-['book_001','book_002']}" \
    -p rfid_tags:="${SIM_RFID_TAGS:-['rfid_001','rfid_002']}" \
    -p classification_codes:="${SIM_CLASS_CODES:-['005.7','006.3']}" \
    > "$LOG/return_machine.log" 2>&1

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
