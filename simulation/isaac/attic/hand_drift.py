"""손 안에서 책이 어긋났는가 — **어느 점을 재느냐**가 전부다. Isaac 없이 돈다.

2026-09-24 마찰 파지 판에서 같은 상태를 재는 세 자가 10배씩 달랐다:

    회전 0.3°  ·  원점 기준 0.1 cm  ·  형상중심 기준 1.0 cm  →  406 실패

셋 다 맞고 셋 다 틀리다. 원점 기준과 형상중심 기준은 **미끄러짐을 재지 않는다** —
책이 손 안에서 조금만 돌면 지렛대만큼 증폭된다. 이 레벨의 책은 prim 원점이 형상에서
75~177 cm 떨어져 있어(2026-09-22 실측) 0.3° 가 9 mm 로 찍힌다.

미끄러짐이란 **손가락이 닿은 자리가 책 위에서 옮겨가는 것**이다. 그러니 손가락이
잡은 그 점을 손 기준으로 보면 된다. 지렛대가 없으니 회전에 증폭되지 않는다.

어젯밤 원점 → 형상중심으로 바꾼 것은 자를 바꾼 것이 아니라 **지렛대의 한쪽 끝에서
다른 쪽 끝으로 옮긴 것**이었다. 길이는 그대로였다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from arm_geometry import R_from_quat


@dataclass(frozen=True)
class HandDrift:
    """기준 시점 대비 어긋남. 길이는 m, 각도는 도."""

    origin: float
    center: float
    grip: float
    rot_deg: float


def hand_frame_points(hand_p, hand_q, book_p, book_q, c_loc=None, up_loc=None, hz=0.0):
    """손 기준으로 본 (원점, 형상중심, 쥔점) 과 책의 상대 회전행렬."""
    Rh = R_from_quat(np.asarray(hand_q, float))
    Rb = R_from_quat(np.asarray(book_q, float))
    bp = np.asarray(book_p, float)
    hp = np.asarray(hand_p, float)
    o = bp
    c = bp if c_loc is None else bp + Rb @ np.asarray(c_loc, float)
    if c_loc is None or up_loc is None:
        g = c
    else:
        g = bp + Rb @ (np.asarray(c_loc, float) + np.asarray(up_loc, float) * float(hz))
    to_hand = Rh.T
    return (to_hand @ (o - hp), to_hand @ (c - hp), to_hand @ (g - hp), to_hand @ Rb)


def drift_between(ref, now) -> HandDrift:
    """`hand_frame_points` 두 벌 → 세 기준의 어긋남과 상대 회전각."""
    cos = (float(np.trace(np.asarray(ref[3]).T @ np.asarray(now[3]))) - 1.0) / 2.0
    return HandDrift(
        origin=float(np.linalg.norm(np.asarray(now[0]) - np.asarray(ref[0]))),
        center=float(np.linalg.norm(np.asarray(now[1]) - np.asarray(ref[1]))),
        grip=float(np.linalg.norm(np.asarray(now[2]) - np.asarray(ref[2]))),
        rot_deg=float(np.degrees(math.acos(min(1.0, max(-1.0, cos))))),
    )


def amplification_mm(lever_m: float, deg: float = 1.0) -> float:
    """지렛대 `lever_m` 인 기준이 `deg` 회전을 몇 mm 로 부풀리는가."""
    return float(lever_m) * math.radians(deg) * 1000.0
