# ★ 삽입 성공 기준선 (v06_picky, 2026-09-22 23:5x) — 앞으로 모든 비교의 기준
    bash night/run_vision.sh <ID> "SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1"
    (= 출발점 조합 SIM_JOINT_SEGS=approach,carry_rotate,return SIM_GRIP_ROT90=1 SIM_GRASP_KINEMATIC=1
       SIM_HAND_DRIFT_M=0.25 SIM_SPEED_SCALE=0.5 SIM_MAX_STEP=0.12 + 위 두 개)
- 결과: placement_verified True, checks 5개 true, fallen_books [], 나머지 4권 트레이·바른자세 ±1 mm
- 남은 것: 403 (carry_rotate 관절6 |Δq| 0.0435 rad/스텝 = 속도한계 2.61 rad/s ÷ 60 Hz 정확히 100% — **위치 한계가 아니라 속도 포화**, q +1.20~1.29 는 위치 한계 −0.087~3.0 안)
         트레이 16.7 cm 밀림 (중심 y −3.023 → −3.190, 같은 구간) — **기하 문제** (손·책이 트레이를 관통)
- 두 문제는 성질이 다르다: 트레이 밀림 → 스윙 동선(2-3), 403 → 재시간 배분(3-2 SIM_RETIME). 스윙으로 관절6 이동량이 줄면 둘 다 풀릴 수 있지만 **가정하지 말고 결과로 확인**.
- SIM_SPEED_SCALE=0.5 인데도 100% → 배율이 관절 속도까지 안 내려간다 (3-1 이 확인할 것)

---
# 저녁 기준선 — 403 @ RETREATING 이 나온 실행 그대로 (랩탑 세션 제공, 2026-09-22 23:3x)

**이 경로(비전 포함)로 한 번 재현해야 밤새 비교가 성립한다.** drive_job.py(비전 없이 book_index)는 좋은 도구지만
기준선 재현에는 쓰지 않는다 — r01/r02 의 401 "IK 실패 [2.478,-3.253,0.56]" 은 저녁 파지점과 다른 자리였다.

## 출발점 조합 (전 단계 공통 환경)
    SIM_JOINT_SEGS=approach,carry_rotate,return SIM_GRIP_ROT90=1 SIM_GRASP_KINEMATIC=1 \
    SIM_HAND_DRIFT_M=0.25 SIM_SPEED_SCALE=0.5

## 1) 정리
    bash night/cleanup_demo.sh          # (GPU PC 의 /tmp/cl3.sh 대신)

## 2) Isaac 기동 (--start-home 을 주지 않는다 = 기본 move. keep 이면 approach 출발점이 실제 팔과 어긋난다)
    env DISPLAY=:1 SIM_USD=$HOME/Desktop/ing_library_env_v5.usd \
      SIM_TRAY_SETTLE_S=2.0 SIM_TRAY_DELIVERY_S=6.0 \
      SIM_JOINT_SEGS=approach,carry_rotate,return SIM_GRIP_ROT90=1 \
      SIM_GRASP_KINEMATIC=1 SIM_HAND_DRIFT_M=0.25 SIM_SPEED_SCALE=0.5 SIM_MAX_STEP=0.12 \
      ./scripts/run_isaac_sim.sh --gui --books 5 \
        --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color \
        --amr-test-overrides --drive-speed 0.6

## 3) ROS 환경
    export ROS_DOMAIN_ID=130 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/b1_arm/config/fastdds_local.xml
    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash

## 4) TF 2개
    ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link --ros-args -p use_sim_time:=true
    ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color --child-frame-id sim_camera --ros-args -p use_sim_time:=true

## 5) 비전·로봇팔 노드
    R=ros2_ws/src/shelving_perception/resource
    ros2 run shelving_perception vision_manager --ros-args --params-file ros2_ws/src/shelving_perception/config/perception.yaml \
      -p model_path:=$R/book_tray_best.pt -p shelf_model_path:=$R/best.pt -p confidence_threshold:=0.75
    ros2 run shelving_manipulation manipulation_node --ros-args --params-file ros2_ws/src/shelving_manipulation/config/manipulation.yaml -p executor:=sim
    sleep 10
    ros2 topic pub -r 1 /perception/detect_request std_msgs/Bool "{data: true}"
    sleep 5

## 6) 사이클
    python3 simulation/isaac/tools/full_cycle.py --speed 0.6
- 주행 목표 PICK_SPOT = (2.535, SIM_PICK_Y 기본 -3.324), 도착 후 베이스 yaw 0°
- pick_from_vision.py 가 서브프로세스로: 책 선택은 비전(/perception/books 최신값), 삽입 목표 기본 (-0.3497, 0.5495, 0.3399) 팔 기준

## 저녁에 찍힌 값 (비교 기준)
    비전 좌표(윗면 중심) frame=arm_base_link (-0.3633, +0.0088, +0.1951)
    꽂을 곳                                   (-0.3497, +0.5495, +0.3399)
    → 계획 OK, 최대 인접변화 0.082 rad
    → code=403 phase=RETREATING "관절 각속도 80% 초과 22스텝 (최대 100%)"

## 재현이 안 되면 확인 순서
- A. --start-home 이 기본(move)인가
- B. 베이스가 (2.535, -3.324), yaw 0 에 섰는가. 팔 베이스 ≈ 루트 +0.30 → (2.835, -3.324)
- C. SIM_PICK_Y 가 -3.324 인가 (옛값 -3.019 면 서가에 너무 붙는다)
- D. sim.log 시작부의 `스위치: ...` 줄이 위 조합과 같은가 (켰다고 믿지 말고 확인)
- 계획 실패면 approach 목표를 **팔 기준**으로 바꿔 저녁 (-0.3633, +0.0088, +0.195) 과 비교
- 참고: 이 401 은 저녁 401("32배로 나눠도 한 걸음 1.558 rad" = 해는 있는데 불연속)과 종류가 다르다 — "IK 실패" = 해 없음

## v29.md (올라옴, c7cfa6a)
    git fetch origin && git show origin/temp/DELETE-AFTER-night-transfer-0922:docs/doyoon-kim/web_claude/v29.md
