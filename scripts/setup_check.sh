#!/usr/bin/env bash
# 새 PC 에서 **무엇이 빠졌는지** 알려준다. 아무것도 설치하지 않고 확인만 한다.
#
#   ./scripts/setup_check.sh
#
# 왜 있나: 저장소만 받아서는 안 돌아간다. Isaac 설치, ROS, 그리고 **git 에 없는 자산**
# (레벨 USD, 책 USD, M0609 기술서)이 따로 있어야 한다. 무엇이 없는지 사람이 하나씩
# 찾아내는 데 시간이 든다 — 한 번에 보여준다.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
miss=0

ok()   { printf '  OK    %s\n' "$*"; }
bad()  { printf '  없음  %s\n' "$*"; miss=$((miss+1)); }
warn() { printf '  주의  %s\n' "$*"; }
head_() { printf '\n== %s ==\n' "$*"; }

head_ "1. 운영체제"
. /etc/os-release 2>/dev/null || true
case "${VERSION_ID:-}" in
    24.04) ok "Ubuntu 24.04 (ROS 2 Jazzy 기준. 권장)" ;;
    22.04) warn "Ubuntu 22.04 — Isaac Sim 은 되지만 ROS 2 Jazzy 는 소스 빌드가 필요하다" ;;
    *)     bad "Ubuntu 22.04 또는 24.04 가 아니다 (${PRETTY_NAME:-알 수 없음})" ;;
esac

head_ "2. ROS 2 / 빌드 도구"
[ -f /opt/ros/jazzy/setup.bash ] && ok "ROS 2 Jazzy" || bad "ROS 2 Jazzy (/opt/ros/jazzy)"
command -v colcon  >/dev/null && ok "colcon"  || bad "colcon (sudo apt install python3-colcon-common-extensions)"
command -v git     >/dev/null && ok "git"     || bad "git"
command -v git-lfs >/dev/null || git lfs version >/dev/null 2>&1 \
    && ok "git-lfs (비전 모델이 LFS 로 들어온다)" || bad "git-lfs (sudo apt install git-lfs && git lfs install)"

head_ "3. Isaac Sim"
ISAAC="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
if [ -x "$ISAAC/python.sh" ]; then
    ok "Isaac Sim ($ISAAC)"
else
    bad "Isaac Sim — $ISAAC/python.sh 가 없다. 설치 후 ISAAC_SIM_PATH 로 지정"
fi
# `A && B | sed || bad` 는 파이프라인 종료코드가 sed 것이라 **드라이버가 깨져도 통과**한다
# (커널 모듈 불일치가 시연 당일 아침에 실제로 생길 수 있는 상태다). if 로 명확히 가른다.
if command -v nvidia-smi >/dev/null; then
    _gpu=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>&1)
    if [ -n "$_gpu" ] && ! printf '%s' "$_gpu" | grep -qi "failed\|error"; then
        printf '%s\n' "$_gpu" | sed 's/^/  OK    GPU /'
    elif printf '%s' "$_gpu" | grep -qi "version mismatch"; then
        # apt 가 드라이버를 올렸는데 커널 모듈이 구버전일 때. 설치 실패가 아니라 **재부팅** 건이다
        bad "GPU 드라이버/라이브러리 버전 불일치 — **재부팅하면 해소된다**"
    else
        bad "nvidia-smi 실패: $_gpu"
    fi
else
    bad "nvidia-smi 없음 — NVIDIA 드라이버 (Isaac 은 RTX 계열이 필요하다)"
fi

head_ "4. 저장소 빌드"
for p in shelving_manipulation shelving_perception shelving_interfaces; do
    [ -d "$REPO_ROOT/ros2_ws/install/$p" ] && ok "빌드됨 $p" \
        || bad "빌드 안 됨 $p (cd ros2_ws && colcon build --symlink-install)"
done
for f in "$REPO_ROOT"/ros2_ws/src/shelving_perception/resource/*.pt; do
    [ -e "$f" ] || { bad "비전 모델 파일이 없다 (git lfs pull)"; break; }
    sz=$(stat -c%s "$f" 2>/dev/null || echo 0)
    [ "$sz" -gt 100000 ] && ok "비전 모델 $((sz/1024/1024))MB $(basename "$f")" \
        || bad "비전 모델이 LFS 포인터다 ($(basename "$f")) — git lfs pull"
done

head_ "5-1. 저장소 안에 있는 에셋 (클론하면 따라온다)"
# 책·트레이·서가는 git 에 들어 있다. 2026-09-20 까지 스크립트가 개인 홈 경로만 봐서
# "저장소에 있는데 없다" 가 됐었다 — 지금은 저장소 사본을 먼저 본다.
_n=$(ls "$REPO_ROOT"/simulation/assets/book_dataset/usd_v2/*book0[1-6].usdc 2>/dev/null | wc -l)
[ "$_n" -eq 6 ] && ok "책 USD 6종" || bad "책 USD ($_n/6) — git 클론이 온전한지 확인"
[ -f "$REPO_ROOT/simulation/assets/book_dataset/assets/tray/tray_v1.usdc" ] \
    && ok "트레이 USD" || bad "트레이 USD"

head_ "5-2. git 에 없는 자산 (USB 나 scp 로 받아야 한다)"
# 용량이 크거나 외부 소유라 저장소에 못 넣은 것들. 경로는 robot_profiles.py 기본값과 같아야 한다.
declare -A ASSETS=(
    ["$HOME/Isaac_Sim_b-1/src_pra/M0609/descriptor/m0609_description.yaml"]="M0609 Lula 기술서 (IK 에 필요)"
    ["$HOME/Isaac_Sim_b-1/src_pra/M0609/doosan-robot2/urdf/m0609.urdf"]="M0609 URDF"
    ["$HOME/Desktop/Collected_ing_library_env_v5-firstFinal"]="레벨 USD 폴더 (약 130MB/개) — AMR 담당 제작본"
)
for p in "${!ASSETS[@]}"; do
    [ -e "$p" ] && ok "${ASSETS[$p]}" || bad "${ASSETS[$p]}  →  $p"
done

# 레벨은 **생성물**이다. 없으면 Isaac 이 4분 뜬 뒤에야 실패한다 — 여기서 미리 본다
_LV="$HOME/Desktop/Collected_ing_library_env_v5-firstFinal"
for _f in level_yaw0.usd level_shelf01.usd; do
    [ -f "$_LV/$_f" ] && ok "레벨 $_f" \
        || bad "레벨 $_f — 원본에서 생성 필요 (SETUP_NEW_PC.md §5)"
done

head_ "6. DDS 네트워크 설정 (가장 많이 걸리는 함정)"
WL="$HOME/.ros/fastdds_whitelist.xml"
if [ -f "$WL" ]; then
    addrs=$(grep -oE '<address>[0-9.]+</address>' "$WL" | grep -oE '[0-9.]+' | tr '\n' ' ')
    ok "화이트리스트 있음: $addrs"
    myip=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' | tr '\n' ' ')
    hit=0
    for a in $addrs; do for m in $myip; do [ "$a" = "$m" ] && hit=1; done; done
    if [ "$hit" -eq 0 ]; then
        warn "이 PC 주소($myip)가 화이트리스트에 없다 — **DDS 통신이 전부 막힌다**"
        warn "  연구실 유선망(10.10.0.x) 기준 파일이다. 다른 망에서는 주소를 추가하거나"
        warn "  FASTRTPS_DEFAULT_PROFILES_FILE 을 비우고 실행할 것"
    fi
else
    warn "화이트리스트 없음 — 한 PC 안에서만 쓸 거면 그래도 된다"
fi

printf '\n'
if [ "$miss" -eq 0 ]; then
    echo "빠진 것 없음. ./scripts/run_tests.sh 로 넘어갈 것"
else
    echo "**빠진 것 $miss 개** — 위 '없음' 항목을 채울 것 (docs/doyoon-kim/manipulation/SETUP_NEW_PC.md)"
fi
# 빠진 게 있으면 0 이 아닌 값으로 끝낸다 — `setup_check && run_tests` 로 이어 쓰는 사람이 있다
[ "$miss" -eq 0 ]
