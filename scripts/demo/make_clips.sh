#!/usr/bin/env bash
# 녹화 프레임(PNG) → mp4. 시점마다 한 편.
#
#   bash scripts/demo/make_clips.sh <rec폴더> [나갈폴더] [fps] [crf]
#
# 프레임은 시뮬 스텝마다 찍히므로 **실시간이 아니다**. 45분 판이 20 fps 에서 약 3~4분이 된다.
# 저장소에 넣는 편은 **20 MB 아래 하나만** (mp4 는 LFS 대상이 아니라 저장소를 영구히 키운다).
set -euo pipefail
REC="${1:?rec 폴더를 달라 — 예: logs/rec/0929_025840}"
OUT="${2:-$REC/clips}"
FPS="${3:-20}"
CRF="${4:-32}"
mkdir -p "$OUT"
for d in "$REC" "$REC"/*/; do
    [ -d "$d" ] || continue
    name=$(basename "$d"); [ "$d" = "$REC" ] && name=auto
    [ "$name" = "clips" ] && continue
    n=$(find "$d" -maxdepth 1 -name 'f*.png' | wc -l)
    [ "$n" -gt 10 ] || { echo "건너뜀 $name (프레임 $n장)"; continue; }
    ffmpeg -nostdin -loglevel error -y -framerate "$FPS" \
        -pattern_type glob -i "$d/f*.png" \
        -vf "scale=1280:720:flags=lanczos" -c:v libx264 -preset slow -crf "$CRF" \
        -pix_fmt yuv420p -movflags +faststart "$OUT/$name.mp4"
    printf "%-8s %5d장 → %s  %.1f MB\n" "$name" "$n" "$OUT/$name.mp4" \
        "$(stat -c%s "$OUT/$name.mp4" | awk '{print $1/1048576}')"
done
