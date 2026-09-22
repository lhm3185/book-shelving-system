#!/usr/bin/env bash
# 새 PC 에서 **무엇이 빠졌는지** 알려준다. 아무것도 설치하지 않고 확인만 한다.
#
#   ./scripts/setup_check.sh
#
# 왜 있나: 저장소 자산은 Git LFS 내려받기 상태까지 온전해야 하고, Isaac/ROS/GPU도
# 준비돼야 한다. 무엇이 없는지 사람이 하나씩 찾는 대신 한 번에 보여준다.
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
_n=$(ls "$REPO_ROOT"/isaac_sim/assets/book_dataset/usd_v2/*book0[1-6].usdc 2>/dev/null | wc -l)
[ "$_n" -eq 6 ] && ok "책 USD 6종" || bad "책 USD ($_n/6) — git 클론이 온전한지 확인"
[ -f "$REPO_ROOT/isaac_sim/assets/book_dataset/assets/tray/tray_v1.usdc" ] \
    && ok "트레이 USD" || bad "트레이 USD"
[ -f "$REPO_ROOT/isaac_sim/assets/cobot3_ws/isaacpjt/M0609/descriptor/m0609_description.yaml" ] \
    && ok "M0609 Lula 기술서" || bad "M0609 Lula 기술서"
[ -f "$REPO_ROOT/isaac_sim/assets/cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609.urdf" ] \
    && ok "M0609 URDF" || bad "M0609 URDF"

head_ "5-2. 최종 통합 월드 (저장소 기본값)"
# world_loader.py의 기본값과 같은 파일을 본다. 외부 Desktop 경로는 기본 실행에 필요 없다.
_LV="$REPO_ROOT/isaac_sim/assets"
for _f in ing_library_env_v5-test.usd Nova_Carter_ROS.usd \
    cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim/m0609_isaac_sim.usd \
    cobot3_ws/isaacpjt/M0609/onrobot_rg2/urdf/onrobot_rg2/onrobot_rg2.usd; do
    _p="$_LV/$_f"
    _sz=$(stat -c%s "$_p" 2>/dev/null || echo 0)
    [ "$_sz" -gt 1024 ] && ok "시뮬 자산 $_f" \
        || bad "시뮬 자산 $_f — git lfs pull 또는 클론 상태 확인"
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
    # "없음" 이라고만 하면 만들어야 하나 싶어진다. 연구실 밖에서는 **없는 게 정상**이다.
    ok "화이트리스트 없음 — 연구실 밖(집·다른 망)에서는 이게 정상이다. 만들지 말 것"
    note_wl=1
fi
if [ "${note_wl:-0}" = "1" ]; then
    printf '  참고  이 파일은 연구실 유선망(10.10.0.x)만 허용하는 것이라, 그 망에 있을 때만 쓴다.\n'
    printf '        집에서 만들면 ROS 통신이 전부 막힌다.\n'
fi

printf '\n'
if [ "$miss" -eq 0 ]; then
    echo "빠진 것 없음. ./scripts/run_tests.sh 로 넘어갈 것"
else
    echo "**빠진 것 $miss 개** — 위 '없음' 항목을 채울 것 (docs/doyoon-kim/manipulation/SETUP_NEW_PC.md)"
fi
# 빠진 게 있으면 0 이 아닌 값으로 끝낸다 — `setup_check && run_tests` 로 이어 쓰는 사람이 있다
[ "$miss" -eq 0 ]
