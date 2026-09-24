"""녹화 시점 — **같은 판을 여러 각도에서** 찍는다.

왜 여러 각도인가: 한 각도로는 못 보는 것이 있다. 옆에서 보면 책이 들어가는 깊이가
보이지만 비뚤어짐은 안 보이고, 위에서 보면 비뚤어짐은 보이지만 높이가 안 보인다.
2026-09-24 에 "꽂혔다" 고 보고된 판에서 책이 85.8° 누워 있었는데, 그건 **숫자로만**
잡혔다. 눈으로 확인하려면 그 각도가 화면에 있어야 한다.

**자리를 코드에 박지 않는다.** 시점은 장면(로봇 AABB·서가 AABB)에서 파생한다 —
레벨이 바뀌면 시점도 따라간다. 이 밤에 열 번 본 병이 "레벨에서 잰 값을 상수로
박기" 였다.

Isaac 없이 도는 순수 기하다.
"""
from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

import numpy as np


def look_at_quat(eye: Sequence[float], target: Sequence[float],
                 up: Sequence[float] = (0.0, 0.0, 1.0)) -> np.ndarray:
    """`eye` 에서 `target` 을 보는 카메라 자세 → 쿼터니언 `(w, x, y, z)`.

    Isaac 의 `set_world_pose` 가 받는 순서다. USD 카메라는 **−z 방향**을 본다.

    눈과 목표가 같거나 시선이 `up` 과 나란하면 방향을 정할 수 없다 — 그때는
    **단위 쿼터니언**을 돌려준다. 아무 방향이나 내지 않는다.
    """
    e = np.asarray(eye, float)
    t = np.asarray(target, float)
    f = t - e
    n = float(np.linalg.norm(f))
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    f /= n
    u0 = np.asarray(up, float)
    r = np.cross(f, u0)
    rn = float(np.linalg.norm(r))
    if rn < 1e-9:                      # 바로 위/아래를 볼 때
        u0 = np.array([0.0, 1.0, 0.0])
        r = np.cross(f, u0)
        rn = float(np.linalg.norm(r))
        if rn < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])
    r /= rn
    u = np.cross(r, f)
    m = np.eye(3)
    m[:, 0], m[:, 1], m[:, 2] = r, u, -f
    tr = float(np.trace(m))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
        q = np.zeros(4)
        q[0] = (m[k, j] - m[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (m[j, i] + m[i, j]) / s
        q[k + 1] = (m[k, i] + m[i, k]) / s
    return q / (float(np.linalg.norm(q)) or 1.0)


#: 시점 이름. `all` 은 이 순서 전부
VIEW_NAMES = ("auto", "front", "side", "top", "shelf", "wide")


def view_pose(name: str, robot_aabb, shelf_aabb=None, home_xy=None
              ) -> Tuple[np.ndarray, np.ndarray]:
    """시점 이름 → `(눈, 보는 점)`. **전부 장면에서 파생한다.**

    `robot_aabb`·`shelf_aabb` 는 월드 AABB `(x0, y0, z0, x1, y1, z1)`.
    `home_xy` 는 출발 자리(키오스크) — `wide` 가 그 둘을 한 화면에 넣는다.
    """
    rb = np.asarray(robot_aabb, float)
    rmid = (rb[:3] + rb[3:]) / 2.0
    rad = float(np.linalg.norm(rb[3:] - rb[:3])) or 1.0
    smid = rmid if shelf_aabb is None else (np.asarray(shelf_aabb, float)[:3]
                                            + np.asarray(shelf_aabb, float)[3:]) / 2.0
    if name == "auto":        # 지금까지 쓰던 자동 구도 — 동작을 바꾸지 않으려고 남긴다
        return rmid + np.array([rad * 0.9, -rad * 1.0, rad * 0.55]), rmid
    if name == "front":       # 서가 쪽에서 로봇을 마주 본다 — 팔 전체와 트레이
        d = rmid - smid
        d[2] = 0.0
        d = d / (float(np.linalg.norm(d)) or 1.0)
        return rmid + d * rad * 1.3 + np.array([0.0, 0.0, rad * 0.45]), rmid
    if name == "side":        # 팔 +x 쪽 옆에서 — **꽂히는 깊이**가 보인다
        return rmid + np.array([rad * 1.4, 0.0, rad * 0.25]), rmid
    if name == "top":         # 바로 위에서 — **비뚤어짐**이 보인다
        return rmid + np.array([0.0, -0.001, rad * 1.5]), rmid
    if name == "shelf":       # 서가 앞면 가까이 — 책이 들어가는 순간
        if shelf_aabb is None:
            return view_pose("side", robot_aabb)
        sb = np.asarray(shelf_aabb, float)
        face_y = sb[1] if abs(sb[1] - rmid[1]) < abs(sb[4] - rmid[1]) else sb[4]
        toward = np.sign(rmid[1] - face_y) or 1.0
        look = np.array([smid[0], face_y, smid[2]])
        return look + np.array([0.0, toward * rad * 0.9, rad * 0.35]), look
    if name == "wide":        # 출발 자리까지 한 화면에
        if home_xy is None:
            return rmid + np.array([rad * 1.8, -rad * 2.0, rad * 1.1]), rmid
        h = np.array([float(home_xy[0]), float(home_xy[1]), rmid[2]], float)
        mid = (h + rmid) / 2.0
        span = float(np.linalg.norm(h[:2] - rmid[:2])) + rad
        return mid + np.array([span * 0.8, -span * 0.8, span * 0.5]), mid
    raise ValueError(f"모르는 시점: {name} (쓸 수 있는 것: {', '.join(VIEW_NAMES)})")


def parse_views(spec: str) -> Tuple[str, ...]:
    """`"side,top"` · `"all"` · 빈 문자열 → 시점 이름 튜플. **빈 값이면 빈 튜플**이다."""
    spec = (spec or "").strip()
    if not spec:
        return ()
    if spec == "all":
        return VIEW_NAMES
    out = []
    for part in spec.replace(" ", "").split(","):
        if not part:
            continue
        if part not in VIEW_NAMES:
            raise ValueError(f"모르는 시점: {part} (쓸 수 있는 것: {', '.join(VIEW_NAMES)})")
        if part not in out:
            out.append(part)
    return tuple(out)


def all_poses(names: Sequence[str], robot_aabb, shelf_aabb=None, home_xy=None
              ) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    return {n: view_pose(n, robot_aabb, shelf_aabb, home_xy) for n in names}
