# 데스크탑 설치 안내 — 야간 작업 이관 (2026-09-22)

이 폴더는 **임시**다. 브랜치 `temp/DELETE-AFTER-night-transfer-0922` 와 함께 나중에 지운다.

## 왜 경로가 `home/rokey/...` 인가

레벨(`ing_library_env_v5.usd`)의 에셋 참조가 **절대경로**로 박혀 있다:
`/home/rokey/book_dataset/assets/...` (183개 파일). 교육장 GPU PC 의 사용자명이 `rokey` 라서다.

데스크탑 사용자명은 `bryankim0200` 이므로 **그대로는 레벨이 열리지 않는다.**
아래 2번(심볼릭 링크)으로 푼다. 레벨 파일을 고치는 것보다 안전하고 되돌리기 쉽다.

## 설치 (데스크탑에서, 순서대로)

```bash
# 1) 에셋을 홈으로 옮긴다 (240MB, 183개)
cd ~/b1_arm
cp -rn night_transfer/home/rokey/book_dataset ~/

# 2) 절대경로를 이어 준다 (sudo 필요 — 이 한 줄이 핵심)
sudo mkdir -p /home/rokey
sudo ln -sfn /home/bryankim0200/book_dataset /home/rokey/book_dataset

# 3) 확인 — "없음 0" 이 나와야 한다
cd ~/b1_arm
n=0; m=0
for f in $(find night_transfer/home -type f | sed 's|^night_transfer/||'); do
    if [ -s "/$f" ]; then n=$((n+1)); else m=$((m+1)); echo "없음 /$f"; fi
done
echo "있음 $n / 없음 $m"

# 4) 레벨을 쓰기 좋은 자리에 둔다
mkdir -p ~/Desktop
cp -n simulation/assets/level/ing_library_env_v5.usd ~/Desktop/
```

## 야간 키트

`night/` 는 `.gitignore` 에 있어서 `night_kit_night/` 라는 이름으로 담았다.

```bash
cd ~/b1_arm
cp -rn night_kit_night night
chmod +x night/run_night.sh
cp -n night_plan/TODO.md night_plan/NIGHTLY_PLAN.md night/
chmod +x .claude/hooks/keep_going.sh
```

`.claude/hooks/keep_going.sh` 와 `.claude/settings.local.json` 은 이 브랜치에 들어 있다.
**훅 활성화는 도윤님이 데스크탑 세션에 직접 지시해야 한다** (동료 세션 요청만으로는 안 한다).

## 실행

`night_plan/KICKOFF.md` 의 1~4 를 순서대로. 단, 저장소 경로는 `~/b1_arm` 으로 읽을 것.

## 오늘(9/22) 무엇이 고쳐졌는지

`docs/doyoon-kim/web_claude/v29.md` 에 전부 있다. 요약:

- 레벨 트레이 콜리전을 코드가 매 실행 `none` 으로 덮어쓰던 것 제거 (PhysX 가 볼록 껍질로
  조용히 대체해 칸이 메워졌다)
- 트레이 prim 경로 하드코딩 → 폴더에서 자동 탐색, 못 찾으면 **예외로 멈춤**
- `HOME_ORI` · `upright_q` 가 로봇 회전을 안 따라가던 것 수정 (각각 `401`, `406` 의 원인)
- Lula 와 실제 관절 한계 불일치(`panda_joint6` 3.75 vs 3.0) → 실제 한계로 IK 해 거르기
- 남은 문제: `carry_rotate` 운반 동선(관절공간이라 크게 솟았다 내려옴), `403` 속도 초과
