"""좌표 계약 프레임을 규칙으로 고정한다 — 2026-09-20 에 실제로 깨진 것들.

무슨 일이 있었나
    Isaac 은 팔 베이스 prim 의 **이름**을 TF frame 으로 낸다.
        franka → panda_link0 / m0609 → base_link
    그런데 정적 TF 가 `panda_link0 → arm_base_link` 로 박혀 있었다. M0609 에는
    `panda_link0` 이라는 frame 이 없으므로 **`arm_base_link` 가 트리에 안 붙었다.**
    비전 쪽은 조회가 실패하니 `target_frame` 을 `base_link` 로 바꿨고, 그러면 계약이 깨진다.

    로봇을 바꾼 것은 3일 전인데 **시연 전날 밤에야** 알았다. 사람이 아니라 테스트가 잡게 한다.
"""

import re
import subprocess
import sys
from pathlib import Path

ISAAC_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ISAAC_ROOT.parents[1]
sys.path.insert(0, str(ISAAC_ROOT / "config"))
from robot_profiles import profile  # noqa: E402

CONTRACT_FRAME = "arm_base_link"

# 정적 TF 별칭을 내보내는 스크립트들 (부모 프레임이 로봇마다 달라야 한다)
TF_SCRIPTS = [
    ISAAC_ROOT / "tools" / "run_demo_pc.sh",
    ISAAC_ROOT / "tools" / "run_vision_test.sh",
    ISAAC_ROOT / "tools" / "record_vision_bag.sh",
]


def _profile_base_frame(robot):
    """robot_profiles 가 말하는 팔 베이스의 **TF frame 이름** (prim 경로의 마지막 조각)"""
    out = subprocess.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, %r); from robot_profiles import profile;"
         " print(profile().base_link.split('/')[-1])" % str(ISAAC_ROOT / "config")],
        env={"ARM_ROBOT": robot, "PATH": "/usr/bin:/bin"},
        capture_output=True, text=True, check=True)
    return out.stdout.strip()


def test_두_로봇의_팔_베이스_프레임이_실제로_다르다():
    """이 전제가 깨지면 아래 테스트들의 의미가 없어진다 (음성 대조)."""
    assert _profile_base_frame("franka") == "panda_link0"
    assert _profile_base_frame("m0609") == "base_link"


def test_통합_월드의_기본_로봇은_m0609(monkeypatch):
    """환경변수가 없어도 기본 v5-test 월드의 Nova Carter를 검사해야 한다."""
    monkeypatch.delenv("ARM_ROBOT", raising=False)
    assert profile().name == "m0609"
    assert profile().root == "/World/Nova_Carter_ROS"


def test_라이다_prim은_로봇_프로파일마다_다르다():
    """Franka 경로를 Nova Carter에 붙이는 머지 회귀를 막는다."""
    assert profile("franka").lidar_prim == "front_laser/Lidar"
    assert profile("m0609").lidar_prim == "chassis_link/sensors/XT_32/PandarXT_32_10hz"


def test_정적TF_스크립트가_부모_프레임을_박아두지_않는다():
    """`--frame-id panda_link0` 처럼 박아 두면 로봇을 바꿨을 때 별칭이 끊긴다."""
    for path in TF_SCRIPTS:
        text = path.read_text(encoding="utf-8")
        hardcoded = re.findall(r"--frame-id\s+(panda_link0|m0609/\S+)\b", text)
        assert not hardcoded, (
            f"{path.name} 이 부모 프레임을 박아 두었다: {hardcoded}. "
            "로봇 프로파일에서 읽거나 ARM_BASE_FRAME 으로 받을 것")


def test_정적TF_스크립트가_계약_이름을_그대로_쓴다():
    """자식은 항상 arm_base_link 다 — 로봇이 바뀌어도 이 이름은 안 바뀐다."""
    for path in TF_SCRIPTS:
        text = path.read_text(encoding="utf-8")
        assert f"--child-frame-id {CONTRACT_FRAME}" in text, (
            f"{path.name} 에 '--child-frame-id {CONTRACT_FRAME}' 가 없다")


def test_로봇팔_노드가_계약_프레임만_받는다():
    """비전이 base_link 로 보내면 거절해야 한다 — 결합체에서 이름 충돌 위험이 있다."""
    cfg = (REPO_ROOT / "ros2_ws" / "src" / "shelving_manipulation"
           / "config" / "manipulation.yaml").read_text(encoding="utf-8")
    assert CONTRACT_FRAME in cfg, "manipulation.yaml 에 계약 프레임이 없다"


def test_시연_스크립트가_계약_프레임으로_목표를_보낸다():
    pick = (REPO_ROOT / "scripts" / "demo" / "pick.sh").read_text(encoding="utf-8")
    assert f"frame_id: '{CONTRACT_FRAME}'" in pick, (
        "pick.sh 가 계약 프레임이 아닌 것으로 목표를 보낸다")
