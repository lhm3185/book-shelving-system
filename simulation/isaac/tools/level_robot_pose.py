#!/usr/bin/env python3
"""레벨에 배치된 로봇의 **자리와 각도를 레벨에서 그대로 읽는다**.

왜: 복귀 좌표가 `waypoints.yaml` 에 **상수로** 적혀 있어서 레벨과 끊겨 있었다.
2026-09-24 실측으로 **4.99 m · 90°** 어긋나 있었고, 그래서 삽입 뒤 복귀하면
처음 자리로 안 돌아왔다.

도윤님 물음: *"레벨 위 배치된 로봇의 각도랑 좌표를 그대로 가져다 쓰면 안 되는 거야?"*
→ **된다. 이 파일이 그것이다.** 레벨이 바뀌면 다시 돌리면 되고, 값을 손으로
옮겨 적지 않는다.

    python3 level_robot_pose.py <레벨.usd>
    python3 level_robot_pose.py <레벨.usd> --waypoints <원본.yaml> --out <사본.yaml>

Isaac 없이 돈다 (`usd-core` 만 쓴다).
"""
import argparse
import math
import sys

import numpy as np

#: 로봇 루트 후보. 앞에서부터 찾아 처음 있는 것을 쓴다 (`robot_profiles.py` 와 같은 값)
ROOT_CANDIDATES = ("/World/ridgeback_franka", "/World/Nova_Carter_ROS")

#: 루트 → 팔 베이스 오프셋 (m), **로봇 몸체 기준**. yaw 로 돌려서 더한다.
#: 방향을 실측 두 쌍으로 가렸다 (2026-09-24):
#:   출발  루트 (4.986, −5.607) yaw +90° → 팔 베이스 (4.9850, −5.3071)
#:   서가  루트 (2.535, −3.019) yaw   0° → 팔 베이스 (2.835,  −3.019)
#: 로컬 **+x** 0.30 이면 둘 다 맞는다. 로컬 +y 로 두면 출발 쪽이 x 로 틀어진다.
#: **처음에 +y 로 적었다가 시험이 잡았다.**
ARM_BASE_OFFSET_LOCAL = (0.30, 0.0)


def yaw_from_matrix(m) -> float:
    """4x4 월드 변환 → z 축 둘레 각(rad). 기울어져 있어도 **수평 성분**만 본다."""
    x_axis = np.array([m[0][0], m[0][1]], float)
    n = float(np.linalg.norm(x_axis))
    if n < 1e-9:
        return 0.0
    return math.atan2(x_axis[1] / n, x_axis[0] / n)


def quat_from_yaw(yaw: float):
    """yaw(rad) → `(x, y, z, w)`. 경유점 yaml 의 순서다."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def arm_base_from_root(x: float, y: float, yaw: float):
    """루트 자리·각도 → 팔 베이스 자리. 오프셋을 yaw 로 돌려서 더한다."""
    ox, oy = ARM_BASE_OFFSET_LOCAL
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * ox - s * oy, y + s * ox + c * oy


def read_pose(usd_path: str, root: str = ""):
    """레벨에서 로봇 루트의 `(x, y, yaw_rad, 쓴 prim 경로)` 를 읽는다."""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise SystemExit(f"레벨을 못 연다: {usd_path}")
    paths = [root] if root else list(ROOT_CANDIDATES)
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            continue
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return float(m[3][0]), float(m[3][1]), yaw_from_matrix(m), p
    raise SystemExit(f"로봇 루트를 못 찾았다. 찾아본 곳: {paths}\n"
                     f"  --root 로 직접 줄 수 있다")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("--root", default="", help="로봇 루트 prim (기본: 후보에서 찾는다)")
    ap.add_argument("--waypoints", default="", help="원본 경유점 yaml — 주면 `home` 을 바꿔 사본을 쓴다")
    ap.add_argument("--out", default="", help="사본 경로 (--waypoints 와 같이 쓴다)")
    a = ap.parse_args()

    x, y, yaw, used = read_pose(a.level, a.root)
    ax, ay = arm_base_from_root(x, y, yaw)
    qx, qy, qz, qw = quat_from_yaw(yaw)
    print(f"레벨   {a.level}")
    print(f"루트   {used}")
    print(f"  자리 ({x:+.4f}, {y:+.4f})   각도 {math.degrees(yaw):+.2f}°")
    print(f"  쿼터니언 z {qz:+.7f}  w {qw:+.7f}")
    print(f"  팔 베이스 (파생) ({ax:+.4f}, {ay:+.4f})   ← 로그의 '팔 베이스 출발 자리' 와 대조할 것")

    if not a.waypoints:
        return 0
    if not a.out:
        print("--out 이 필요하다", file=sys.stderr)
        return 2
    import re
    src = open(a.waypoints, encoding="utf-8").read()
    m = re.search(r"  home:\n(?:    [^\n]*\n|      [^\n]*\n)+", src)
    if not m:
        print("경유점 yaml 에서 home 블록을 못 찾았다", file=sys.stderr)
        return 2
    block = (f"  home:\n"
             f"    # **레벨에서 읽은 값이다** — 손으로 적지 않는다.\n"
             f"    #   {a.level}\n"
             f"    #   루트 {used} · 각도 {math.degrees(yaw):+.2f}°\n"
             f"    #   다시 내려면: python3 simulation/isaac/tools/level_robot_pose.py <레벨> \\\n"
             f"    #       --waypoints <원본> --out <이 파일>\n"
             f"    position:\n      x: {x:.4f}\n      y: {y:.4f}\n      z: 0.0\n"
             f"    orientation:\n      x: 0.0\n      y: 0.0\n"
             f"      z: {qz:.7f}\n      w: {qw:.7f}\n"
             f"    behavior: as_is     # 보정 없이 이 좌표 그대로 간다 (원본과 같다)\n")
    head = ("# **레벨에서 읽어 만든 사본** — 손으로 고치지 않는다. 원본도 안 건드린다.\n"
            "# `home` 만 레벨의 로봇 배치에서 왔고 나머지는 원본 그대로다.\n#\n")
    open(a.out, "w", encoding="utf-8").write(head + src[:m.start()] + block + src[m.end():])
    print(f"\n썼다  {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
