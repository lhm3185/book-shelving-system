"""통합 USD 를 열고 필요한 Prim 이 있는지 검사한다 (Isaac Sim 5.1.0).

경로는 **저장소 기준**으로 계산한다. 개인 PC 절대경로를 기본값으로 쓰지 않는다.
Isaac 설치 위치만 시스템마다 다르므로 `ISAAC_SIM_PATH` 환경변수로 받는다 (실행 스크립트가 쓴다).
"""
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_USD = ( REPO_ROOT / "simulation" / "assets" / "level" / "ing_library_env_v5.usd" )
DEFAULT_TRAY = ( REPO_ROOT / "simulation" / "assets" / "book_dataset" / "assets" / "tray" / "tray_v1.usdc" )

# 로봇 prim 경로는 로봇마다 다르다 → 프로파일에서 가져온다 (config/robot_profiles.py)
sys.path.insert(0, str(Path(__file__).resolve().parent / "config"))
from robot_profiles import profile  # noqa: E402

BOT = profile()

# 통합 USD 에 있어야 하는 것들. 없으면 시작할 때 바로 알린다
# 값이 튜플이면 **그중 하나**만 있으면 된다. 2026-09-25 레벨 정리로 서가는 /World/bookshelves_main,
# 책은 /World/bookshelves_main/books 로 옮겨졌다(옛 레벨은 /World/bookshelves, /World/books).
REQUIRED_PRIMS = {
    "로봇(AMR+로봇팔)": BOT.root,
    "서가": ("/World/bookshelves_main", "/World/bookshelves"),
    "책 원본": ("/World/bookshelves_main/books", "/World/books"),
}
OPTIONAL_PRIMS = {
    "손목 카메라": f"{BOT.root}/{BOT.camera_prim}",
    # 라이다는 AMR 담당 구성이라 로봇마다 다르다. 없으면 건너뛴다
    "라이다": f"{BOT.root}/front_laser/Lidar",
    "무인반납기": "/World/return_machine",
}


def resolve_usd(path=None):
    """USD 경로 결정: 인자 > 환경변수 SIM_USD > 저장소 기본값"""
    p = path or os.environ.get("SIM_USD") or str(DEFAULT_USD)
    return str(Path(os.path.expanduser(p)).resolve())


def is_placeholder(path):
    """git-lfs 포인터거나 비어 있는 파일인지 (아직 통합 USD 가 없는 상태)"""
    try:
        if os.path.getsize(path) < 4096:
            with open(path, "rb") as f:
                head = f.read(200)
            return head.startswith(b"version https://git-lfs") or len(head.strip()) == 0
    except OSError:
        return False
    return False


def open_world(app, usd_path, say=print):
    """USD 를 열고 로딩이 끝날 때까지 기다린다. 열 수 없으면 이유를 분명히 알린다"""
    from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage

    usd = resolve_usd(usd_path)
    if not os.path.exists(usd):
        raise FileNotFoundError(
            f"USD 가 없다: {usd}\n"
            f"  통합 USD 는 {DEFAULT_USD} 이다. 아직 없으면 --usd 로 시험용 레벨을 지정한다.")
    if is_placeholder(usd):
        raise RuntimeError(
            f"USD 가 빈 파일이다 (git-lfs 포인터 또는 빈 내용): {usd}\n"
            f"  통합 USD 가 아직 채워지지 않았다. 채워질 때까지 --usd 로 시험용 레벨을 지정한다.")
    say(f"USD 열기: {usd}")
    open_stage(usd)
    app.update()
    while is_stage_loading():
        app.update()
    return get_current_stage()


def check_prims(stage, say=print, required=None, optional=None):
    """있어야 하는 Prim 을 검사한다. 없으면 RuntimeError, 선택 항목은 알리기만 한다"""
    missing = []
    for name, path in (required or REQUIRED_PRIMS).items():
        cands = path if isinstance(path, (tuple, list)) else (path,)
        if not any(stage.GetPrimAtPath(p).IsValid() for p in cands):
            missing.append(f"{name} ({' 또는 '.join(cands)})")
    for name, path in (optional or OPTIONAL_PRIMS).items():
        if not stage.GetPrimAtPath(path).IsValid():
            say(f"선택 Prim 없음: {name} ({path}) — 해당 기능은 건너뛴다")
    if missing:
        raise RuntimeError("통합 USD 에 필요한 Prim 이 없다: " + ", ".join(missing))
    say(f"Prim 검사 통과 ({len(required or REQUIRED_PRIMS)}개)")
