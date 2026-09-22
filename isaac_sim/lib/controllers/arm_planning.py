"""결정론적 경로 계획: 직교 경유점을 촘촘히 이산화하고 IK 를 앞 해에서 이어 풀어 관절 경로로 만든다.

왜 이렇게 하나 (2026-09-17 실측)
- 실행 중 매 스텝 IK 를 새로 풀면 해가 가지를 바꿔 관절이 튀었다 (손목 세우기 구간 38회)
- 미리 풀어 두면 실행은 관절 보간만 하므로 튈 수 없고, 불연속을 계획 단계에서 잡는다

IK 함수를 주입받는다 — Isaac(Lula)이든 mock 이든 같은 계획기를 쓴다.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from arm_geometry import quat_angle, slerp

IKFn = Callable[[np.ndarray, np.ndarray, np.ndarray], Tuple[np.ndarray, bool]]
Waypoint = Tuple[np.ndarray, np.ndarray]          # (위치, 쿼터니언 wxyz)

STEP_M = 0.005          # 위치 이산화 간격
STEP_RAD = 0.035        # 자세 이산화 간격 (약 2°)
MAX_JOINT_STEP = 0.12   # 인접점 관절 변화가 이보다 크면 해 가지 전환(특이점 근처)으로 본다


@dataclass
class PathPlan:
    qs: Optional[List[np.ndarray]]
    worst_step: float = 0.0
    error_code: int = 0             # 0, 401(IK 실패), 402(불연속)
    error: str = ""

    @property
    def ok(self) -> bool:
        return self.error_code == 0


def plan_path(waypoints: Sequence[Waypoint], seed, ik: IKFn,
              step_m: float = STEP_M, step_rad: float = STEP_RAD,
              max_joint_step: float = MAX_JOINT_STEP) -> PathPlan:
    qs = [np.asarray(seed, float)]
    prev_p = prev_q = None
    worst = 0.0
    for p_, q_ in waypoints:
        p_ = np.asarray(p_, float)
        q_ = np.asarray(q_, float)
        if prev_p is None:
            n = 1
        else:
            n = max(1, int(math.ceil(np.linalg.norm(p_ - prev_p) / step_m)),
                    int(math.ceil(quat_angle(prev_q, q_) / step_rad)))
        for i in range(1, n + 1):
            t = i / n
            tp = p_ if prev_p is None else prev_p + (p_ - prev_p) * t
            tq = q_ if prev_q is None else slerp(prev_q, q_, t)
            sol, ok = ik(tp, tq, qs[-1])
            if not ok:
                return PathPlan(None, worst, 401, f"IK 실패 {np.round(tp, 3).tolist()}")
            worst = max(worst, float(np.max(np.abs(np.asarray(sol, float) - qs[-1]))))
            qs.append(np.asarray(sol, float))
        prev_p, prev_q = p_, q_
    if worst > max_joint_step:
        return PathPlan(None, worst, 402, f"인접점 관절 변화 {worst:.3f} rad > {max_joint_step}")
    return PathPlan(qs, worst)


def plan_chain(segments: Sequence[Tuple[str, Sequence[Waypoint]]], seed, ik: IKFn, **kw):
    """이름 붙은 구간들을 끝 자세를 이어 가며 계획한다. 하나라도 실패하면 그 구간 이름과 함께 반환"""
    plans = {}
    q = np.asarray(seed, float)
    worst = 0.0
    for name, wps in segments:
        pl = plan_path(wps, q, ik, **kw)
        if not pl.ok:
            return None, PathPlan(None, pl.worst_step, pl.error_code, f"{name}: {pl.error}")
        plans[name] = pl
        q = pl.qs[-1]
        worst = max(worst, pl.worst_step)
    return plans, PathPlan([q], worst)


def tucked_joint_moves(q_now, q_goal, q_stow, turn_threshold=0.5):
    """큰 1번 관절(몸통) 회전을 **팔을 접은 채** 하도록 관절 경유점을 만든다.

    곧장 보간하거나 팔을 높이 든 채 돌리면 손·손목이 주변 서가 윗판에 걸린다
    (2026-09-17 Isaac: 1권 시작 홈 이동 j1 1.2 rad 뒤처짐, 4권 7번 관절 2 rad 밀림 → 각속도 100~126%).
    stow 는 verify_stow.py 로 주변 겹침이 없음을 확인한 자세다.

    1번 관절 변화가 turn_threshold 이하면 접지 않고 곧장 간다. 반환: 경유 관절 자세 목록(마지막 = q_goal)
    """
    q_now = np.asarray(q_now, float); q_goal = np.asarray(q_goal, float); q_stow = np.asarray(q_stow, float)
    if abs(q_goal[0] - q_now[0]) <= turn_threshold:
        return [q_goal]
    tuck = q_stow.copy(); tuck[0] = q_now[0]
    # 1번(베이스 회전)과 **마지막 관절**(손목 돌림)은 목표값을 미리 맞춰 둔다.
    # 예전에는 turned[6] 로 박아 두어 7축 전용이었다 — 6축에서는 인덱스가 없어 죽는다
    # (2026-09-20 M0609 에서 IndexError). 관절 수에서 마지막을 집는다.
    turned = q_stow.copy(); turned[0] = q_goal[0]; turned[-1] = q_goal[-1]
    return [tuck, turned, q_goal]
