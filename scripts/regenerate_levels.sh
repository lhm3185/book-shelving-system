#!/usr/bin/env bash
# 원본 레벨에서 **파생 레벨**을 다시 만든다.
#
#   ./scripts/regenerate_levels.sh --dry-run     # 무엇을 할지 보기만 한다
#   ./scripts/regenerate_levels.sh               # 실제 생성
#
# 왜 필요한가
#     AMR 담당이 준 것은 원본 `ing_library_env_v5.usd` 하나다.
#     시연에 쓰는 `level_yaw0.usd` / `level_shelf01.usd` 는 우리가 그 원본을 가공한 **파생본**이라
#     백업·USB 어디에도 없다. PC 를 새로 세팅할 때마다 여기서 다시 만든다.
#
# 알아 둘 것
#     - 두 도구 다 **Lula 를 안 쓴다.** M0609 기술서(m0609_description.yaml)가 없어도 이 단계는 된다
#     - 원본이 참조하는 바닥·벽(env_wall_floor)이 레벨 폴더에 없으면 **경고만 나고 판정은 통과한다.**
#       이 스크립트가 저장소 사본으로 채운다 (바이트 동일). 건너뛰려면 NO_FILL=1
#     - Nova Carter 본체가 Isaac 클라우드 에셋을 참조한다 → **첫 실행 때 몇 분 멈춘다. 인터넷 필요.**
#       멈춘 게 아니니 기다릴 것
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LV="${LV:-$HOME/Desktop/Collected_ing_library_env_v5-firstFinal}"
SRC="${SRC:-ing_library_env_v5.usd}"

# --shelf / --row-z 는 기본값이 근거 있는 값이다. 바꿀 때는 아래를 읽을 것.
#   shelf_brown__book_shelf_01 : 레벨의 서가 16개 중 **콜라이더가 있는 유일한 서가**다.
#                                다른 서가 앞에서 꽂으면 책이 통과해 바닥으로 떨어진다 (2026-09-20 실측)
#   row-z 1.042                : 그 서가의 선반판 윗면 월드 z. 광선으로 잰 값이며
#                                판은 0.498 / 1.042 / 1.581 / 2.058 에 있다.
#                                M0609 는 팔 베이스가 0.655 라 0.498 단은 받침판보다 낮아 못 쓴다
SHELF="${SHELF:-/World/bookshelves/shelf_brown__book_shelf_01}"
ROW_Z="${ROW_Z:-1.042}"
DRY=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run|-n) DRY=1 ;;
        --lv)    LV="$2"; shift ;;
        --src)   SRC="$2"; shift ;;
        --shelf) SHELF="$2"; shift ;;
        --row-z) ROW_Z="$2"; shift ;;
        *) echo "모르는 인자: $1  (--dry-run | --lv <폴더> | --src <원본> | --shelf <prim> | --row-z <z>)"; exit 2 ;;
    esac
    shift
done

note() { printf '   %s\n' "$*"; }
say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

say "사전 확인"
ISAAC="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
fail=0
chk() { if [ -e "$2" ]; then note "OK    $1"; else note "없음  $1  →  $2"; fail=1; fi; }
chk "Isaac Sim (python.sh)" "$ISAAC/python.sh"
chk "실행기 run_isaac_tool.sh" "$REPO_ROOT/scripts/run_isaac_tool.sh"
chk "원본 레벨 $SRC" "$LV/$SRC"
# Nova_Carter_ROS.usd 는 원본이 참조하는 레이어다. 없으면 로봇이 통째로 안 들어온다
if [ -e "$LV/Nova_Carter_ROS.usd" ]; then
    note "OK    Nova_Carter_ROS.usd (원본이 참조하는 레이어)"
else
    note "주의  Nova_Carter_ROS.usd 가 없다 — 로봇이 안 들어올 수 있다. 원본 폴더를 통째로 복원할 것"
fi
[ "$fail" -eq 0 ] || { echo; echo "**빠진 것이 있다** — docs/doyoon-kim/manipulation/SETUP_NEW_PC.md 참조"; exit 1; }

# 원본 레벨은 **레벨 폴더 기준 상대경로**로 바닥·벽을 payload 한다.
# 백업에서 복원한 폴더에 이 트리가 빠져 있으면 USD 가 'Could not open asset' 경고만 내고
# **판정은 그대로 통과한다** — 바닥 없는 레벨이 조용히 저장된다 (2026-09-21 실제로 겪었다).
# 저장소에 같은 상대경로로 들어 있으므로(바이트 동일) 없으면 채운다.
_REPO_BD="$REPO_ROOT/isaac_sim/assets/book_dataset"
_need_fill=0
for _rel in assets/env_wall_floor dataset/env_wall_floor; do
    [ -d "$LV/book_dataset/$_rel" ] || _need_fill=1
done
if [ "$_need_fill" -eq 1 ]; then
    if [ "${NO_FILL:-0}" = "1" ]; then
        note "주의  레벨 폴더에 env_wall_floor 가 없다 — 바닥·벽 없이 저장된다 (NO_FILL=1 이라 그대로 둔다)"
    else
        note "레벨 폴더에 바닥·벽 에셋이 없다 → 저장소 사본으로 채운다"
        for _rel in assets/env_wall_floor dataset/env_wall_floor; do
            if [ -d "$_REPO_BD/$_rel" ]; then
                if [ "$DRY" -eq 1 ]; then
                    printf '   $ rsync -a %s/ %s/\n' "$_REPO_BD/$_rel" "$LV/book_dataset/$_rel"
                else
                    mkdir -p "$LV/book_dataset/$_rel"
                    rsync -a "$_REPO_BD/$_rel/" "$LV/book_dataset/$_rel/" \
                        && note "  채움 book_dataset/$_rel" \
                        || note "  **복사 실패** book_dataset/$_rel"
                fi
            fi
        done
        note "(건너뛰려면 NO_FILL=1)"
    fi
else
    note "OK    레벨 폴더의 바닥·벽 에셋 (env_wall_floor)"
fi

step() {   # step <번호> <설명> <출력파일> <명령...>
    local n="$1" desc="$2" out="$3"; shift 3
    say "$n  $desc"
    note "→ $out"
    if [ "$DRY" -eq 1 ]; then
        printf '   $ %s\n' "$*"
        return 0
    fi
    ( cd "$REPO_ROOT" && "$@" ) || { echo "   **실패**"; exit 1; }
    # 도구가 조용히 실패하면 다음 단계가 엉뚱한 파일을 쓰게 된다 — 산출물을 직접 확인한다
    [ -f "$out" ] || { echo "   **산출물이 안 생겼다: $out**"; exit 1; }
    note "생성됨 ($(du -h "$out" | cut -f1))"
}

step "1/2" "로봇 yaw 를 0 으로 (경로 계산이 월드 축을 쓴다)" "$LV/level_yaw0.usd" \
    env ARM_ROBOT=m0609 ISAAC_ENTRY=isaac_sim/isaac/tools/set_robot_yaw.py \
        ./scripts/run_isaac_tool.sh --usd "$LV/$SRC" --out "$LV/level_yaw0.usd" --yaw 0

step "2/2" "로봇을 서가 앞으로 (자기검증을 스스로 돌린다)" "$LV/level_shelf01.usd" \
    env ARM_ROBOT=m0609 ISAAC_ENTRY=isaac_sim/isaac/tools/place_robot_at_shelf.py \
        ./scripts/run_isaac_tool.sh --usd "$LV/level_yaw0.usd" --out "$LV/level_shelf01.usd" \
            --shelf "$SHELF" --row-z "$ROW_Z"

say "합격 판정"
note "2/2 출력에서 다음 두 줄을 볼 것:"
note "    ① 왕복 자기일치: ... 오차 0.0 mm OK"
note "    받침판 서가쪽 모서리 ... → 여유 +4.8 cm"
note "둘 중 하나라도 다르면 로봇이 서가 앞에 제대로 안 선 것이다."
note ""
note "다음: ./scripts/setup_check.sh  →  ./scripts/run_tests.sh"
[ "$DRY" -eq 1 ] && note "" && note "(--dry-run 이었다. 실제로 하려면 인자 없이 다시 실행)"
exit 0
