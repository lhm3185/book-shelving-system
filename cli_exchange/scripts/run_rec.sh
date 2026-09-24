#!/bin/bash
# 녹화판: run_vision.sh 에 시뮬 카메라 녹화 인자를 붙여 돌린다.
#   bash night/run_rec.sh <실행ID> side|rear "A=1 B=2"
# 프레임: night/runs/<ID>/frames/f*.png (--record-every 30, REC_EVERY 로 조절)
#   bash night/run_rec.sh <ID> wide "..."   ← 발표용: 홈~트레이~서가 전 구간이 든다
ID=${1:?실행ID}; VIEW=${2:?side|rear}; shift 2
D=$HOME/b1_arm/night/runs/$ID
case $VIEW in
  side) CAM="--record-eye 4.0 -2.95 1.3 --record-look 2.6 -2.85 0.55" ;;
  rear) CAM="--record-eye 2.55 -4.1 1.55 --record-look 2.5 -2.65 0.5" ;;
  # **전 구간이 한 화면에 들어야 한다** — 키오스크(5.0, -5.6)와 서가(2.5, -2.5)가
  # 4.4 m 떨어져 있어 side/rear 로는 한쪽이 잘린다. 둘의 중점을 보고 옆으로 물러난다.
  wide) CAM="--record-eye 7.2 -7.0 3.4 --record-look 3.7 -4.1 0.7" ;;
  *) echo "시점은 side|rear"; exit 2 ;;
esac
export SIM_EXTRA_ARGS="--record-dir $D/frames --record-every ${REC_EVERY:-30} $CAM"
bash $HOME/b1_arm/night/run_vision.sh "$ID" "$@"
