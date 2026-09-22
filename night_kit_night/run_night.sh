#!/usr/bin/env bash
# 야간 무인 실행기 — tmux 안에서 돌린다.
#
#   tmux new -s night
#   cd ~/ws_cobot_pjt/book-shelving-system && ./night/run_night.sh
#   (Ctrl+b d 로 빠져나와도 계속 돈다. 아침에 tmux attach -t night)
#
# 이중 안전장치:
#   ① Stop 훅(.claude/hooks/keep_going.sh) — 한 세션 안에서 "보고하고 멈추기"를 막는다
#   ② 이 바깥 루프 — 그래도 세션이 끝나면(문맥 초과·오류·훅 한도) 새 세션을 띄워 이어서 한다
#      상태는 대화가 아니라 night/TODO.md · night/LEDGER.md 파일에 있으므로 새 세션이 이어받는다
set -u
cd "$(dirname "$0")/.."
N=night
DEADLINE="${NIGHT_DEADLINE:-07:30}"
export NIGHT_DEADLINE="$DEADLINE"
export BASH_DEFAULT_TIMEOUT_MS=600000      # Bash 도구 기본 10분 (Isaac 한 판이 들어가게)
export BASH_MAX_TIMEOUT_MS=1200000         # 상한 20분

date +%H:%M > "$N/ACTIVE"                  # 훅이 이 파일이 있을 때만 막는다
rm -f "$N/DONE" "$N"/.blocks_*
trap 'rm -f "$N/ACTIVE"; echo "[run_night] 종료 $(date +%T)" | tee -a "$N/run.log"' EXIT

# 노트북이면 잠들지 않게 (systemd 가 있을 때)
INHIBIT=""; command -v systemd-inhibit >/dev/null && INHIBIT="systemd-inhibit --what=idle:sleep --why=night-run"

past_deadline() {
    local now end start; now=$(date +%H%M); end=$(echo "$DEADLINE" | tr -d :); start=$(tr -d : < "$N/ACTIVE" | head -c4)
    [ "$now" -ge "$end" ] && [ "$now" -lt "$start" ]
}

i=0
while [ ! -f "$N/DONE" ] && ! past_deadline; do
    i=$((i+1))
    echo "[run_night] 세션 #$i 시작 $(date +%T)" | tee -a "$N/run.log"
    $INHIBIT claude -p "$(cat "$N/PROMPT.md")" \
        --permission-mode acceptEdits \
        --output-format text \
        >> "$N/session_$i.log" 2>&1
    echo "[run_night] 세션 #$i 끝 $(date +%T) (종료코드 $?)" | tee -a "$N/run.log"
    sleep 20                                # 연속 실패 시 폭주 방지
done
