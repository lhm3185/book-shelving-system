#!/bin/bash
# 녹화판: run_vision.sh 에 시뮬 카메라 녹화 인자를 붙여 돌린다.
#   bash night/run_rec.sh <실행ID> side|rear "A=1 B=2"
# 프레임: night/runs/<ID>/frames/f*.png (--record-every 30)
ID=${1:?실행ID}; VIEW=${2:?side|rear}; shift 2
D=$HOME/b1_arm/night/runs/$ID
case $VIEW in
  side) CAM="--record-eye 4.0 -2.95 1.3 --record-look 2.6 -2.85 0.55" ;;
  rear) CAM="--record-eye 2.55 -4.1 1.55 --record-look 2.5 -2.65 0.5" ;;
  *) echo "시점은 side|rear"; exit 2 ;;
esac
export SIM_EXTRA_ARGS="--record-dir $D/frames --record-every ${REC_EVERY:-30} $CAM"
bash $HOME/b1_arm/night/run_vision.sh "$ID" "$@"
