#!/usr/bin/env bash
# ① Isaac 을 띄운다. **어느 PC 에서 실행해도 된다** — GPU PC 에는 ssh 로 들어간다.
#
#   PROFILE=pick  ./scripts/demo/sim_up.sh        # 파지까지 (검증된 구성)
#   PROFILE=shelf ./scripts/demo/sim_up.sh        # 서가 삽입 (미해결)
#   SIM_HOST=local ./scripts/demo/sim_up.sh       # GPU PC 에 직접 앉아서 실행할 때
#   DRY_RUN=1 ./scripts/demo/sim_up.sh            # 명령만 보고 실행은 안 한다
#
# 이 스크립트는 화면을 점유한다. **띄운 터미널을 닫으면 Isaac 이 죽는다.**
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

say "### 프로파일 $PROFILE — $PROFILE_NOTE"
say "### 레벨 $SIM_LEVEL_NAME / 도메인 $ROS_DOMAIN_ID / 대상 ${SIM_HOST}"

# 서가 관련 변수는 프로파일에 따라 있을 수도 없을 수도 있다. **한 줄 문자열**로 모은다 —
# 줄바꿈과 섞으면 값이 빌 때 '역슬래시 + 빈 줄' 이 되어 거기서 명령이 끊긴다
SHELF_ENV=""
[ -n "${SIM_SHELF_PRIM:-}" ] && SHELF_ENV="$SHELF_ENV SIM_SHELF_PRIM=$SIM_SHELF_PRIM"
[ -n "${SIM_SHELF_ROW_Z:-}" ] && SHELF_ENV="$SHELF_ENV SIM_SHELF_ROW_Z=$SIM_SHELF_ROW_Z"

# --start-home snap 은 선택이 아니다: move 는 900스텝에서 매번 실패하고,
# 그러면 팔이 계획과 다른 자세에 남아 **그 뒤 모든 동작이 연쇄로 시간 초과**된다 (2026-09-20).
REMOTE=$(cat <<EOF
set -e
cd $SIM_REPO
BOOKS=\$(ls $SIM_BOOK_GLOB | paste -sd,)
[ -n "\$BOOKS" ] || { echo "책 USD 를 못 찾았다: $SIM_BOOK_GLOB"; exit 1; }
# 일부만 있어도 통과하면 **다른 책 구성으로 장면이 뜬다** — 개수까지 본다
_n=\$(echo \$BOOKS | tr , '\n' | wc -l)
[ "\$_n" -eq 6 ] || { echo "책이 6권이 아니다 (\$_n권). git lfs·클론 상태를 확인할 것"; exit 1; }
echo "### 책 \$_n권"
# 레벨 USD 는 **생성물**이라 없을 수 있다. 여기서 안 잡으면 Isaac 이 4분 뜬 뒤에 실패한다
[ -f $SIM_LEVEL_DIR/$SIM_LEVEL_NAME ] || {
    echo "레벨 USD 가 없다: $SIM_LEVEL_DIR/$SIM_LEVEL_NAME"
    echo "  원본에서 만드는 명령은 docs/doyoon-kim/manipulation/SETUP_NEW_PC.md §5 참조"; exit 1; }
ARM_ROBOT=m0609 \\
ROS_DOMAIN_ID=$ROS_DOMAIN_ID \\
FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE \\
SIM_USD=$SIM_LEVEL_DIR/$SIM_LEVEL_NAME $SHELF_ENV \\
./scripts/run_isaac_sim.sh --start-home snap \\
  --camera-prim $CAMERA_PRIM \\
  --sensor-policy always \\
  --book-variants "\$BOOKS"
EOF
)

if [ -n "${DRY_RUN:-}" ]; then
    say "### DRY_RUN — 아래를 실행할 예정이다"
    printf '%s\n' "$REMOTE"
    exit 0
fi

if [ "$SIM_HOST" = "local" ]; then
    bash -c "$REMOTE"
else
    say "### ssh $SIM_USER@$SIM_HOST (Ctrl+C 로 Isaac 종료)"
    # 스크립트를 base64 로 실어 보낸다. 따옴표를 sed 로 이스케이프하는 방식은 한 번 틀리면
    # **원격에서만** 깨져서, 지금처럼 GPU PC 에 접속 못 하는 상황에서는 검증할 방법이 없다.
    # base64 는 따옴표·개행·한글이 섞여도 안 깨진다 (왕복 일치를 로컬에서 확인함).
    ssh -t "$SIM_USER@$SIM_HOST" \
        "echo $(printf '%s' "$REMOTE" | base64 -w0) | base64 -d | bash -l"
fi
