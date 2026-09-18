"""도구들이 쓰는 저장소 기준 경로. 개인 절대경로를 기본값으로 두지 않기 위한 것이다.

    USD  : $SIM_USD  > simulation/library_system.usd
    트레이: $SIM_TRAY > simulation/assets/tray.usd
    모델  : $VISION_MODEL
"""
import os
from pathlib import Path

ISAAC_ROOT = Path(__file__).resolve().parents[1]      # simulation/isaac
REPO_ROOT = ISAAC_ROOT.parents[1]                     # 저장소 최상위
CONTROLLERS = ISAAC_ROOT / "controllers"


def default_usd():
    return os.environ.get("SIM_USD") or str(REPO_ROOT / "simulation" / "library_system.usd")


def default_tray():
    return os.environ.get("SIM_TRAY") or str(REPO_ROOT / "simulation" / "assets" / "tray.usd")


def default_model():
    return os.environ.get("VISION_MODEL") or str(REPO_ROOT / "models" / "book_tray_best.pt")


def ros2_ws():
    return os.environ.get("WS") or str(REPO_ROOT / "ros2_ws")
