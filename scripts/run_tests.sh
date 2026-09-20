#!/usr/bin/env bash
# 테스트 전부 돌린다. **Isaac 없이** 돌아가는 것만 모아 두었다 (GPU PC 없이도 된다).
#
#   ./scripts/run_tests.sh          # 전부
#   ./scripts/run_tests.sh sim      # 시뮬 쪽만 (ROS 없이)
#
# 왜 있나: 테스트는 있는데 돌리는 진입점이 비어 있었다 (2026-09-20 기준 1바이트 빈 파일).
# 그 사이 `arm_base_link` 별칭이 M0609 에서 끊긴 것을 **시연 전날 밤에야** 알았다.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHAT="${1:-all}"
fail=0

run() {   # run <이름> <작업디렉터리> <경로>
    printf '\n=== %s ===\n' "$1"
    ( cd "$2" && python3 -m pytest "$3" -q ) || fail=1
}

# --- 시뮬 쪽: 순수 파이썬. ROS 도 Isaac 도 필요 없다 -------------------------
if [ "$WHAT" = "all" ] || [ "$WHAT" = "sim" ]; then
    run "시뮬 (경로 계획·프레임 계약)" "$REPO_ROOT" "simulation/isaac/tests"
fi

# --- 로봇팔 노드: ROS 가 필요하다 ---------------------------------------------
# **패키지 디렉터리 안에서** 돌린다. 상위에서 돌리면 ament 린터가 다른 패키지(비전)까지
# 훑어서 남의 코드 스타일로 빨갛게 된다 (2026-09-20 실제로 그랬다).
if [ "$WHAT" = "all" ] || [ "$WHAT" = "ros" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ] && [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
        set +u
        . /opt/ros/jazzy/setup.bash
        . "$REPO_ROOT/ros2_ws/install/setup.bash"
        set -u
        run "로봇팔 노드" "$REPO_ROOT/ros2_ws/src/shelving_manipulation" "test"
    else
        printf '\n=== 로봇팔 노드 — 건너뜀 ===\n'
        echo "  ROS 또는 빌드 결과가 없다. 먼저: cd ros2_ws && colcon build --symlink-install"
    fi
fi

printf '\n'
if [ "$fail" -eq 0 ]; then
    echo "전부 통과"
else
    echo "**실패 있음** — 위 출력을 볼 것"
fi
exit "$fail"
