#!/usr/bin/env bash
# 녹화 PNG 연속물 → 카메라별 mp4.  사용: scripts/demo/make_clips.sh [판폴더] [fps]
#
# 왜: 한 판 한 대에 4,500~17,000장(4~17 GB)이 쌓인다. 확인은 영상으로 하고 원본은 지운다.
# fps 는 **찍은 주기에 맞춰** 준다 — 물리 60 Hz 기준 REC_EVERY=4 면 15, 8 이면 7.5, 15 면 4 다.
# 그 값으로 뽑으면 실시간이고, 더 높이면 빨리 감기가 된다.
set -euo pipefail
DIR="${1:-$(ls -td "$HOME/b1_arm/logs/rec"/*/ 2>/dev/null | head -1)}"
FPS="${2:-15}"
[ -d "$DIR" ] || { echo "판 폴더가 없다: $DIR"; exit 1; }
OUT="$DIR/clips"; mkdir -p "$OUT"
echo "판 폴더 $DIR · $FPS fps"
shopt -s nullglob
made=0
for sub in "$DIR"/*/; do
    name=$(basename "$sub")
    [ "$name" = "clips" ] && continue
    n=$(ls "$sub"/*.png 2>/dev/null | wc -l)
    [ "$n" -gt 0 ] || continue
    first=$(ls "$sub"/*.png | head -1)
    echo "  [$name] $n장 → $OUT/$name.mp4"
    ffmpeg -nostdin -loglevel error -y -framerate "$FPS" \
        -pattern_type glob -i "$sub/*.png" \
        -c:v libx264 -pix_fmt yuv420p -crf 18 "$OUT/$name.mp4"
    made=$((made+1))
done
# 폴더 바로 아래에 png 가 있으면(카메라 하나짜리 옛 배치) 그것도 뽑는다
n=$(ls "$DIR"/*.png 2>/dev/null | wc -l)
if [ "$n" -gt 0 ]; then
    echo "  [auto] $n장 → $OUT/auto.mp4"
    ffmpeg -nostdin -loglevel error -y -framerate "$FPS" \
        -pattern_type glob -i "$DIR/*.png" \
        -c:v libx264 -pix_fmt yuv420p -crf 18 "$OUT/auto.mp4"
    made=$((made+1))
fi
[ "$made" -gt 0 ] || { echo "뽑을 png 가 없다"; exit 1; }
echo
ls -lh "$OUT"/*.mp4 | awk '{print "  ", $9, $5}'
echo
echo "원본 PNG: $(du -sh "$DIR" | cut -f1) — 영상 확인 뒤 지울 것"
