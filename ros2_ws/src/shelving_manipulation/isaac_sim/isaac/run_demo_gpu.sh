#!/usr/bin/env bash
# (호환용) 예전 시연 실행 경로. 통합 실행기로 넘긴다 — 공식 명령은 scripts/run_isaac_sim.sh 다.
#   ROS_DOMAIN_ID=130 ./scripts/run_isaac_sim.sh --gui --camera-prim <카메라> --book-variants mixed
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
CAMERA_PRIM="${CAMERA_PRIM:-/World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color}"
export SIM_USD="${SIM_USD:-${LEVEL:-$HOME/Desktop/ing_library_env_v5.usd}}"
echo "### 통합 실행기로 실행한다: $REPO_ROOT/scripts/run_isaac_sim.sh (USD=$SIM_USD)" >&2
MODE=(--gui)
if [ "${1:-}" = "--headless" ]; then MODE=(--headless); shift; fi
exec "$REPO_ROOT/scripts/run_isaac_sim.sh" "${MODE[@]}" --camera-prim "$CAMERA_PRIM" "$@"
