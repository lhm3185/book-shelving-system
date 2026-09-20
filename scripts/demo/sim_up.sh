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
echo "### 책 \$(echo \$BOOKS | tr , '\n' | wc -l)권"
ARM_ROBOT=m0609 \\
ROS_DOMAIN_ID=$ROS_DOMAIN_ID \\
FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.ros/fastdds_whitelist.xml \\
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
    ssh -t "$SIM_USER@$SIM_HOST" "bash -lc '$(printf '%s' "$REMOTE" | sed "s/'/'\\\\''/g")'"
fi
