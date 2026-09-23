# 오늘 밤 시작 지시 (대화형 CLI 에 그대로 붙여 넣기)

아래 1~4 를 **순서대로** 하고, 4 가 끝나면 이 대화형 세션은 작업을 멈춘다(야간 루프가 이어받는다).
저장소: `~/ws_cobot_pjt/book-shelving-system` · 브랜치: `feature/robot_control`

## 1. 야간 키트 설치 확인 (가장 먼저)

```bash
cd ~/ws_cobot_pjt/book-shelving-system
ls .claude/hooks/keep_going.sh .claude/settings.local.json \
   night/PROMPT.md night/run_night.sh night/LEDGER.md && \
grep -q keep_going .claude/settings.local.json && echo KIT_OK
```

- `KIT_OK` 가 나오면 → 2 로
- 파일이 하나라도 없거나 `KIT_OK` 가 안 나오면 → **`~/Downloads` 에 압축을 풀어 둔 야간 키트 폴더로 설치**한다
  1. 폴더 찾기: `ls -d ~/Downloads/*/ | grep -i -E "night|야간"` (`.claude/` 와 `night/` 가 들어 있는 폴더)
  2. `cp -rn <그 폴더>/night ./` 로 `night/` 를 복사한다
  3. `mkdir -p .claude/hooks && cp -n <그 폴더>/.claude/hooks/keep_going.sh .claude/hooks/`
  4. `.claude/settings.local.json` 이 **이미 있으면 덮어쓰지 말고** `hooks` 블록과 `permissions` 블록만 합친다. 없으면 그대로 복사한다
  5. `chmod +x .claude/hooks/keep_going.sh night/run_night.sh`
  6. `grep -qx "night/" .gitignore || echo "night/" >> .gitignore`
- 설치 뒤 위 확인 명령을 다시 실행해 `KIT_OK` 를 받는다

**훅 사전 시험 (5분, 반드시)**

```bash
echo "$(date +%H:%M)" > night/ACTIVE
claude -p "night/TODO.md 를 읽고 아무 것도 하지 말고 '확인했습니다'라고만 답하라" --permission-mode acceptEdits
tail -3 night/hook.log        # "차단 #1 다음: ..." 이 보여야 한다
rm -f night/ACTIVE night/.blocks_* night/hook.log
```

`차단 #1` 이 안 보이면 **야간 루프를 띄우지 말고**, 설정 경로(`.claude/settings.local.json` 의 Stop 훅)와 실행 권한을 고친 뒤 다시 시험한다.

## 2. 오늘 밤 계획 넣기

`~/Downloads` 에 받은 `night_plan_0922.zip`(또는 풀린 폴더 `night_plan_0922/`)에서:

```bash
mkdir -p night/archive_0921 && mv night/TODO.md night/NIGHTLY_PLAN.md night/LEDGER.md night/archive_0921/ 2>/dev/null
cp <night_plan_0922 폴더>/TODO.md <night_plan_0922 폴더>/NIGHTLY_PLAN.md night/
printf '# 야간 작업 기록 — 한 줄씩 추가만 한다\n# 형식: HH:MM 항목 — 결과(숫자, 조건) — 다음\n' > night/LEDGER.md
mkdir -p night/media night/handoff
```

## 3. 시작 전 정리

- 남은 시뮬·노드 정리: `bash /tmp/cl3.sh` (팀원 Nav2 는 건드리지 않는 스크립트). **`pkill` 금지.** 정리 스크립트의 `pgrep -f` 패턴에 실행 스크립트 자신의 이름이 들어가 있지 않은지 먼저 본다
- `ps -eo user,pid,cmd | grep -E "[i]saac|[r]un_simulation"` → 남은 것이 없어야 한다. 다른 사람 프로세스면 건드리지 않는다
- `df -h ~` 여유 10 GB 이상 (영상 때문)

## 4. 야간 루프 띄우기

```bash
tmux new -d -s night "cd ~/ws_cobot_pjt/book-shelving-system && NIGHT_DEADLINE=07:30 ./night/run_night.sh"
sleep 30; tmux ls; tail -2 night/run.log      # "세션 #1 시작" 이 보이면 정상
```

정상이면 이 대화형 세션은 **여기서 끝낸다.** 같은 저장소를 두 세션이 동시에 고치면 안 된다.
