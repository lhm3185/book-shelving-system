"""도구들이 쓰는 저장소 기준 경로. 개인 절대경로를 기본값으로 두지 않기 위한 것이다.

    USD  : $SIM_USD  > isaac_sim/library_system.usd
    트레이: $SIM_TRAY > isaac_sim/assets/tray.usd
    모델  : $VISION_MODEL
"""
import os
from pathlib import Path

ISAAC_ROOT = Path(__file__).resolve().parents[1]      # isaac_sim/isaac
REPO_ROOT = ISAAC_ROOT.parents[1]                     # 저장소 최상위
CONTROLLERS = ISAAC_ROOT / "controllers"


def default_usd():
    return os.environ.get("SIM_USD") or str(
        REPO_ROOT / "simulation" / "assets" / "level" / "ing_library_env_v5.usd"
    )


def default_tray():
    return os.environ.get("SIM_TRAY") or str(
        REPO_ROOT / "simulation" / "assets" / "book_dataset" / "assets" / "tray" / "tray_v1.usdc"
    )


def default_model(name="book_tray_best.pt"):
    """YOLO 모델 경로. **run_demo_pc.sh 와 같은 순서로 찾는다.**

    예전에는 `<저장소>/models/` 하나만 봤는데 그 디렉터리는 저장소에 없다 —
    그래서 `FileNotFoundError: .../models/book_tray_best.pt` 로 바로 죽었다 (2026-09-21).
    모델은 실제로 **비전 패키지에 동봉**돼 있고(origin/vision 33650a0, LFS),
    개인 작업 경로에도 흔히 있다. 둘 다 본다.

    순서: $VISION_MODEL > 개인 경로 > 패키지 동봉본(install > src) > 저장소 models/
    """
    env = os.environ.get("VISION_MODEL")
    if env:
        return env
    home = Path.home()
    ws = REPO_ROOT / "ros2_ws"
    for c in (home / "ws_cobot_pjt/arm/models" / name,
              ws / "install/shelving_perception/share/shelving_perception/resource" / name,
              ws / "src/shelving_perception/resource" / name,
              REPO_ROOT / "models" / name):
        if c.is_file():
            return str(c)
    # 못 찾아도 **조용히 넘어가지 않는다** — 어디를 봤는지 알려준다
    raise FileNotFoundError(
        f"YOLO 모델 '{name}' 을 못 찾았다. 본 곳:\n  " + "\n  ".join(
            str(c) for c in (home / "ws_cobot_pjt/arm/models" / name,
                             ws / "install/shelving_perception/share/shelving_perception/resource" / name,
                             ws / "src/shelving_perception/resource" / name,
                             REPO_ROOT / "models" / name))
        + "\n  → VISION_MODEL=<경로> 로 직접 주거나, colcon build 후 git lfs pull")


def ros2_ws():
    return os.environ.get("WS") or str(REPO_ROOT / "ros2_ws")
