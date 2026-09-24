#!/bin/bash
# **자체 검증판** — 한 판을 돌리면서 **여러 각도를 동시에** 찍는다.
#
#   bash cli_exchange/scripts/verify_demo.sh [실행ID] ["추가 스위치"]
#
# 도윤님께 보여드리기 전에 우리끼리 먼저 보는 판이다. 순서:
#   ① 돌린다 (화면 녹화가 아니라 **시뮬 안 카메라**다 — 남의 바탕화면을 찍지 않는다)
#   ② 로그를 판정 도구로 본다
#   ③ 프레임을 **눈으로** 본다 (각도마다 따로 저장된다)
#
# 왜 여러 각도인가: 한 각도로는 못 보는 것이 있다. 옆은 꽂히는 깊이, 위는
# 비뚤어짐이 보인다. 2026-09-24 에 "꽂혔다" 던 판에서 책이 85.8° 누워 있었는데
# **숫자로만** 잡혔다.
#
# 프레임: night/runs/<ID>/frames/<시점>/f*.png
set -u
ID=${1:-VER$(date +%H%M)}
EXTRA=${2:-SIM_DETECT_SLOT=1}
VIEWS=${VIEWS:-front,side,top,shelf}
EVERY=${REC_EVERY:-15}          # 60 Hz 기준 15 = 초당 4장
R=~/b1_arm
D=$R/night/runs/$ID

echo "==============================================================="
echo "  자체 검증판  ID=$ID"
echo "  시점        $VIEWS   (auto,front,side,top,shelf,wide 또는 all)"
echo "  간격        $EVERY 스텝마다 1장"
echo "  스캔        $(grep -oP 'scan_command:\s*\K\S+' ros2_ws/src/shelving_manipulation/config/manipulation.yaml 2>/dev/null || echo '?')"
echo "  프레임      $D/frames/<시점>/"
echo "==============================================================="

export SIM_EXTRA_ARGS="--record-dir $D/frames --record-every $EVERY --record-views $VIEWS"
# 키오스크 자리를 넘겨 주면 `wide` 가 서가와 출발 자리를 한 화면에 넣는다
export SIM_RECORD_HOME_XY=${SIM_RECORD_HOME_XY:-5.0,-5.6}

bash "$R/night/run_vision.sh" "$ID" "$EXTRA"
rc=$?

echo
echo "=================== ② 로그 판정 ==============================="
python3 simulation/isaac/tools/judge_run.py "$D" || true
echo
echo "=================== 스캔이 두 층을 훑었나 ======================"
grep -hE '^\[스윕\] 판|스윕 시작' "$D/sim.log" 2>/dev/null | cut -c1-160 \
  || echo "  **[스윕] 줄이 없다** — scan_sweep 이 아니라 자세 표로 돌았을 수 있다"
echo
echo "=================== ③ 프레임 ================================="
for v in ${VIEWS//,/ }; do
  n=$(ls "$D/frames/$v"/f*.png 2>/dev/null | wc -l)
  printf "  %-8s %4d 장   %s\n" "$v" "$n" "$D/frames/$v"
done
echo
echo "  눈으로 볼 때: 각 폴더의 **처음·중간·끝** 과 스캔 구간을 본다."
echo "  스캔 구간은 sim.log 의 '스윕 시작' 시각 언저리다."
exit $rc
