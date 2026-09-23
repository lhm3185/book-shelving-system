#!/usr/bin/env bash
# Stop 훅 — 야간 작업 중에는 Claude 가 "보고하고 멈추는" 것을 막는다.
#
# 동작: 종료 코드 2 + stderr 메시지 → Claude Code 가 멈추지 않고 그 메시지를 받아 계속한다
#       (공식 문서: Stop 훅 exit 2 = "Prevents Claude from stopping, continues the conversation").
#
# 켜지는 조건 — 셋 다일 때만 막는다 (낮에 사람이 쓸 때 절대 안 막히게):
#   1) night/ACTIVE 파일이 있다       (run_night.sh 가 만들고, 끝나면 지운다)
#   2) night/DONE 파일이 없다          (모든 항목이 끝나면 Claude 가 만든다)
#   3) 마감 시각 전이다                (NIGHT_DEADLINE, 기본 07:30)
# 무한 반복 방지: 같은 세션에서 MAX_BLOCKS 번 넘게 막으면 놓아준다 (바깥 루프가 새로 띄운다).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
N="$ROOT/night"
INPUT="$(cat)"

[ -f "$N/ACTIVE" ] || exit 0
[ -f "$N/DONE" ] && exit 0

DEADLINE="${NIGHT_DEADLINE:-07:30}"
now=$(date +%H%M); end=$(echo "$DEADLINE" | tr -d :)
# 자정을 넘기는 밤: 시작 시각(ACTIVE 에 기록)보다 작고 마감보다 크면 마감 지남
start=$(cat "$N/ACTIVE" 2>/dev/null | tr -d : | head -c4)
if [ -n "$start" ] && [ "$now" -ge "$end" ] && [ "$now" -lt "$start" ]; then
    echo "마감 $DEADLINE 지남 — 멈춰도 된다" >> "$N/hook.log"; exit 0
fi

SID=$(printf '%s' "$INPUT" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("session_id","x"))' 2>/dev/null || echo x)
CNT_FILE="$N/.blocks_$SID"
cnt=$(( $(cat "$CNT_FILE" 2>/dev/null || echo 0) + 1 )); echo "$cnt" > "$CNT_FILE"
MAX_BLOCKS="${NIGHT_MAX_BLOCKS:-300}"
if [ "$cnt" -gt "$MAX_BLOCKS" ]; then
    echo "$(date +%T) 세션 $SID 차단 $cnt 회 초과 — 놓아줌 (바깥 루프가 재시작)" >> "$N/hook.log"; exit 0
fi

NEXT=$(grep -m1 -E '^\s*- \[ \]' "$N/TODO.md" 2>/dev/null | sed 's/^\s*- \[ \] //')
echo "$(date +%T) 차단 #$cnt 다음: ${NEXT:-없음}" >> "$N/hook.log"

cat >&2 <<EOF
[야간 작업 — 멈추지 말 것] 지금은 무인 야간 작업 중이다. 사람은 없고 아무도 대답하지 않는다.
보고·요약·"계속하겠습니다"·승인 요청으로 턴을 끝내지 마라. 그 말 대신 바로 다음 도구 호출을 하라.
진행 상황은 말로 하지 말고 night/LEDGER.md 에 한 줄 적어라.
다음 항목: ${NEXT:-"TODO.md 의 미완료 항목이 없다 — 마무리 절차를 하고 night/DONE 을 만들어라"}
막혔으면: LEDGER 에 '막힘: 이유' 를 적고 그 항목을 [~] 로 바꾼 뒤 다음 항목으로 가라.
전부 끝났으면: 마무리 절차(회귀·보고서) 후 'touch night/DONE' 을 실행하라. 그때만 멈춰도 된다.
EOF
exit 2
