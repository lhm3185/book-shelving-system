#!/usr/bin/env bash
# 여러 종류 책 시험 한 판 — Isaac(GPU PC) 재시작 + 로봇팔 노드 재시작 + N권 꽂기.
# 노드를 새로 띄우는 이유: 트레이 칸 배정이 노드 안에 남아 있어, 그대로 쓰면 이전 판의 다음 칸부터 집는다.
#   ROS_DOMAIN_ID=130 ./mixed_book_run.sh 4              # 섞인 6종, 4권
#   ROS_DOMAIN_ID=130 BOOKS=same ./mixed_book_run.sh 4   # 같은 종류 6권 (비교용)
set -u
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 export 할 것}"
N="${1:-4}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${WS:-$REPO_ROOT/ros2_ws}"
GPU="${GPU:-rokey@10.10.0.2}"
KEY="${KEY:-$HOME/.ssh/id_ed25519_rocycle}"
LEVEL="${LEVEL:-\$HOME/Desktop/ing_library_env_v5.usd}"
LOG="${LOG:-/tmp/b1_mixed}"; mkdir -p "$LOG"
SSH="ssh -i $KEY -o BatchMode=yes -o ConnectTimeout=8"
VARIANTS=(--book-variants mixed)
[ "${BOOKS:-mixed}" = "same" ] && VARIANTS=()

echo "== GPU PC: Isaac 재시작 (${BOOKS:-mixed})"
$SSH "$GPU" "pgrep -f 'place_book_serve[r].py' | xargs -r kill -9; sleep 2; rm -f /tmp/mixed_run.log; \
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; \
    SIM_USD=$LEVEL nohup ~/book-shelving-system/scripts/run_isaac_sim.sh --headless ${VARIANTS[*]} > /tmp/mixed_run.log 2>&1 & echo 시작"

echo "== 이 PC: 로봇팔 노드 재시작"
# 판을 여러 번 돌리면 ros2 데몬이 죽은 노드의 액션 서버 정보를 들고 있어
# 새 목표가 M411 로 거절된다 (2026-09-18 실측). 데몬을 먼저 내린다
PAT="manipulation""_node"
pgrep -f "$PAT" | xargs -r kill -9
set +u; source /opt/ros/jazzy/setup.bash; set -u
ros2 daemon stop > /dev/null 2>&1 || true
sleep 1
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
nohup ros2 run shelving_manipulation manipulation_node --ros-args \
    --params-file "$WS/src/shelving_manipulation/config/manipulation.yaml" -p executor:=sim \
    > "$LOG/node.log" 2>&1 &
NODE_PID=$!
trap 'kill -9 $NODE_PID 2>/dev/null' EXIT

echo "== Isaac 준비 대기"
for _ in $(seq 1 12); do
    python3 -c "import time; time.sleep(10)"
    if [ "$($SSH "$GPU" 'grep -ac "준비 완료" /tmp/mixed_run.log')" = "1" ]; then READY=1; break; fi
done
[ "${READY:-0}" = "1" ] || { echo "Isaac 준비 안 됨 — GPU PC /tmp/mixed_run.log 확인"; exit 1; }
$SSH "$GPU" 'grep -a "book_[0-9]: 두께" /tmp/mixed_run.log | sed "s/.*### //"'

echo "== $N 권 꽂기"
"$(dirname "$0")/place_books.sh" "$N" | grep -E "^==|success:|error_code|message"

echo "== Isaac 쪽 기록"
$SSH "$GPU" 'grep -aE "작업 demo|\[파지\]|\[놓음\]" /tmp/mixed_run.log | sed "s/.*### //" | cut -c1-160'
