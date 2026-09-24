"""우리 로봇팔 동작 라이브러리 — Franka Panda 순기구학·역기구학·moveJ·moveL (Isaac 없이 numpy 만).

왜 만들었나 (2026-09-23): 지금까지 스캔 자세는 "자세 표 → Lula IK → 관절별 허용 오차" 로 풀어서
자세마다 관절이 150~230° 씩 돌고, 팔이 트레이 책·자기 몸체까지 내려가는 동작이 나왔다. 두산 API 의
moveJ/moveL 처럼 **"카메라를 여기서 저기로 수평으로 옮겨라"** 한 줄로 부르고, 그 사이 관절 경로가
연속(가지 안 바뀜)이 되도록 여기서 한 번에 계획한다. 실행은 기존 JointPath 프리미티브가 한다.

좌표는 전부 **팔 기준(arm_base_link = panda_link0)** 이다. 서가는 +Y, 위는 +Z, 오른쪽이 +X.

    fk(q)                          → (p, R)  손(panda_hand) 자세
    ik(p, R, q_seed)               → q       댐핑 최소자승, 관절 한계·시드 근접 유지
    move_j(q0, q1, step)           → [q]     관절 공간 직선 (등간격)
    move_l(q0, p1, R1, step)       → [q]     손을 직선으로 (경유점마다 IK, 이전 해를 시드로 → 가지 유지)
    camera_pose_for(...)           → (p, R)  카메라 자세 → 손 자세 (손-카메라 고정 변환)
    plan_sweep(...)                → 관절 경로 + 정지점 인덱스 (선반 한 단을 왼쪽→오른쪽 수평으로 훑기)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------- 로봇 상수 (Franka Panda, 수정 DH)
#: (a_{i-1}, d_i, alpha_{i-1}) — Franka 공식 문서의 Craig 표기. 플랜지(link8)까지 8줄.
_DH = [
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -math.pi / 2),
    (0.0, 0.316, math.pi / 2),
    (0.0825, 0.0, math.pi / 2),
    (-0.0825, 0.384, -math.pi / 2),
    (0.0, 0.0, math.pi / 2),
    (0.088, 0.0, math.pi / 2),
    (0.0, 0.107, 0.0),          # 플랜지 — 관절 없음
]
#: panda_hand 는 플랜지에서 z 축으로 -45° 돌아 있다 (franka_description)
_HAND_YAW = -math.pi / 4

#: 관절 한계. 공식 URDF 와 **이 레벨의 실제 한계**(2026-09-23 실측: 4번 -3.0~0.087, 6번 -0.087~3.0) 의 교집합.
Q_MIN = np.array([-2.9671, -1.8326, -2.9671, -3.0, -2.9671, -0.0175, -2.9671])
Q_MAX = np.array([2.9671, 1.8326, 2.9671, -0.0698, 2.9671, 3.0, 2.9671])
#: 한계에서 이만큼은 떨어져 있어야 "쓸 수 있는 해" 로 본다 (rad). 야간 실측 기준 0.15.
LIMIT_MARGIN = 0.10

#: 손 ↔ 카메라 (scan_planner 와 같은 값, 2026-09-22 실측). 손 기준 카메라 위치·축.
T_HAND_CAM = np.array([-0.0400, 0.0015, 0.0164])
R_HAND_CAM = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])   # 열 = 손 기준 camX, camY, camZ. 카메라는 자기 -Z 를 본다


# ---------------------------------------------------------------- 순기구학
def _mdh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0.0, a],
                     [st * ca, ct * ca, -sa, -sa * d],
                     [st * sa, ct * sa, ca, ca * d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk(q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """관절각 7개 → 손(panda_hand) 위치 p(3), 회전 R(3×3). 팔 기준."""
    q = np.asarray(q, float)
    T = np.eye(4)
    for i, (a, d, alpha) in enumerate(_DH):
        theta = float(q[i]) if i < 7 else 0.0
        T = T @ _mdh(a, d, alpha, theta)
    T = T @ _mdh(0.0, 0.0, 0.0, _HAND_YAW)
    return T[:3, 3].copy(), T[:3, :3].copy()


def fk_camera(q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """관절각 → 카메라 위치, 회전(열 = camX, camY, camZ). 팔 기준."""
    p, R = fk(q)
    return p + R @ T_HAND_CAM, R @ R_HAND_CAM


# ---------------------------------------------------------------- 역기구학
def _rot_error(R_now: np.ndarray, R_goal: np.ndarray) -> np.ndarray:
    """회전 오차를 축각 벡터(3) 로 — 작은 회전에서 각속도와 같다."""
    dR = R_goal @ R_now.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0)))
    if ang < 1e-9:
        return np.zeros(3)
    axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2.0 * math.sin(ang))
    return axis * ang


def fk_tool(q: Sequence[float], tool=None) -> Tuple[np.ndarray, np.ndarray]:
    """손(panda_hand) FK 에 손 기준 고정 변환 `tool = (T_hand_tool, R_hand_tool)` 을 붙인 프레임.

    `tool` 이 None 이면 `fk` 와 같다. 손끝(right_gripper, 손 z 로 0.1 m)을 목표로 받을 때
    **오차를 손끝에서 재기 위해** 쓴다 — 손에서 0.05 rad 를 허용하면 손끝은 5 mm 어긋난다.
    """
    p, R = fk(q)
    if tool is None:
        return p, R
    T, Rt = tool
    return p + R @ np.asarray(T, float), R @ np.asarray(Rt, float)


def _jacobian(q: np.ndarray, eps: float = 1e-6, tool=None) -> np.ndarray:
    """수치 야코비안 6×7 (위치 3 + 회전 3). 속도보다 단순함을 택했다 — 계획 단계에서만 쓴다."""
    p0, R0 = fk_tool(q, tool)
    J = np.zeros((6, 7))
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        p1, R1 = fk_tool(q + dq, tool)
        J[:3, i] = (p1 - p0) / eps
        J[3:, i] = _rot_error(R0, R1) / eps
    return J


@dataclass
class IKResult:
    q: Optional[np.ndarray]
    pos_err: float
    rot_err: float
    iters: int
    limit_margin: float          # 해의 관절 한계 여유 (rad). 작을수록 위험

    @property
    def ok(self) -> bool:
        return self.q is not None


def limit_margin(q: Sequence[float]) -> float:
    q = np.asarray(q, float)
    return float(np.min(np.minimum(q - Q_MIN, Q_MAX - q)))


def ik(p_goal: Sequence[float], R_goal: np.ndarray, q_seed: Sequence[float],
       pos_tol: float = 0.002, rot_tol: float = 0.01, max_iter: int = 300,
       damping: float = 0.05, seed_weight: float = 0.02,
       min_margin: float = LIMIT_MARGIN, tool=None) -> IKResult:
    """댐핑 최소자승 IK. **시드에 가까운 해**를 찾는다 (영공간에서 시드 쪽으로 당김) — 그래서
    이전 경유점의 해를 시드로 주면 경로가 연속이 되고 가지가 바뀌지 않는다.
    관절 한계는 매 반복 클램프하고, 한계 여유가 `min_margin` 미만이면 해로 치지 않는다
    (기본 LIMIT_MARGIN. 시뮬의 `ik_joints` 는 0 을 준다 — 자기 가드 `_arm_limits` 가 따로 있어
    Lula 경로와 같은 기준으로 거르기 위해서다)."""
    q = np.clip(np.asarray(q_seed, float).copy(), Q_MIN, Q_MAX)
    seed = q.copy()
    p_goal = np.asarray(p_goal, float)
    best = None
    for it in range(1, max_iter + 1):
        p, R = fk_tool(q, tool)          # tool 을 주면 목표·오차가 그 프레임(손끝)이다
        e = np.concatenate([p_goal - p, _rot_error(R, R_goal)])
        pe, re = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
        if best is None or pe + 0.1 * re < best[0]:
            best = (pe + 0.1 * re, q.copy(), pe, re, it)
        if pe < pos_tol and re < rot_tol:
            m = limit_margin(q)
            return IKResult(q if m >= min_margin else None, pe, re, it, m)
        J = _jacobian(q, tool=tool)
        JJt = J @ J.T + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, e)
        # 영공간: 시드 쪽으로 살짝 (한계 근처 관절을 가운데로 데려오는 효과도 있다)
        null = np.eye(7) - J.T @ np.linalg.solve(JJt, J)
        dq += null @ (seed_weight * (seed - q))
        step = float(np.max(np.abs(dq)))
        if step > 0.2:
            dq *= 0.2 / step
        q = np.clip(q + dq, Q_MIN, Q_MAX)
    _, qb, pe, re, it = best
    return IKResult(None, pe, re, it, limit_margin(qb))


#: 첫 자세를 풀 때 시드를 흔드는 관절과 폭 — 하나만 쓰면 홈의 가지에 갇혀 아래 판 자세를 못 푼다
#: (2026-09-23 탐색: 홈 시드로는 아래 판 x=-0.35 가 한계 여유 0 으로 전부 실패). scan_planner 의 방식.
SEED_JOINTS = (0, 2, 4, 6)
SEED_DELTAS = (-1.2, -0.6, 0.6, 1.2)


def seeds_around(q: Sequence[float]) -> List[np.ndarray]:
    q = np.asarray(q, float)
    out = [q.copy()]
    for j in SEED_JOINTS:
        for d in SEED_DELTAS:
            s = q.copy()
            s[j] = float(np.clip(s[j] + d, Q_MIN[j], Q_MAX[j]))
            out.append(s)
    return out


def ik_best(p_goal: Sequence[float], R_goal: np.ndarray, seeds: Sequence[Sequence[float]],
            prefer: Optional[Sequence[float]] = None, near_weight: float = 0.05) -> IKResult:
    """여러 시드로 풀어 **관절 한계 여유가 가장 큰 해**를 고른다. `prefer` 를 주면 그 자세에서
    멀어지는 만큼 감점한다 (여유 1 rad 당 near_weight rad 의 거리와 맞바꿈)."""
    best: Optional[IKResult] = None
    best_score = -1e9
    for s in seeds:
        r = ik(p_goal, R_goal, s)
        if not r.ok:
            if best is None:
                best = r
            continue
        score = r.limit_margin
        if prefer is not None:
            score -= near_weight * float(np.max(np.abs(r.q - np.asarray(prefer, float))))
        if score > best_score:
            best, best_score = r, score
    return best if best is not None else IKResult(None, 1e9, 1e9, 0, 0.0)


def rescue_seeds(p_goal: Sequence[float], R_goal: np.ndarray, q_seed: Sequence[float],
                 k: int = 3, distinct: float = 0.05, **ik_kw) -> List[np.ndarray]:
    """다른 풀이기(Lula)가 씨앗 전부에서 못 풀 때 **씨앗으로 건네줄 해**를 최대 k 개.

    여러 씨앗에서 풀어 한계 여유가 큰 순으로, 서로 `distinct` rad 넘게 다른 것만 남긴다.
    2026-09-25 00:40: 스윙 끝에서 DOWN→HORIZ 90° 뒤집기를 Lula 가 q_sw·흔들기·홈 씨앗
    어디서도 수렴 못 했는데, 같은 점은 여기서 여유 0.76 으로 풀렸다. 그 해를 씨앗으로
    주면 국소 풀이기는 거기서 바로 수렴한다 — 문턱이 아니라 씨앗을 바꾸는 것이다.
    """
    sols = []
    for s in seeds_around(q_seed):
        r = ik(p_goal, R_goal, s, **ik_kw)
        if not r.ok:
            continue
        if any(float(np.max(np.abs(r.q - o.q))) < distinct for o in sols):
            continue
        sols.append(r)
    sols.sort(key=lambda r: -r.limit_margin)
    return [r.q for r in sols[:k]]


# ---------------------------------------------------------------- 동작 생성
def move_j(q0: Sequence[float], q1: Sequence[float], step: float = 0.05) -> List[np.ndarray]:
    """관절 공간 직선. `step` rad(가장 많이 도는 관절 기준) 간격의 경유점."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    n = max(1, int(math.ceil(float(np.max(np.abs(q1 - q0))) / step)))
    return [q0 + (q1 - q0) * (i / n) for i in range(1, n + 1)]


def move_l(q0: Sequence[float], p1: Sequence[float], R1: np.ndarray, step: float = 0.02,
           R0: Optional[np.ndarray] = None) -> Tuple[List[np.ndarray], Optional[str]]:
    """손을 **직선**으로 p1/R1 까지. 경유점마다 IK, 시드는 직전 해 → 가지 유지.
    회전은 시작·끝 사이를 축각으로 보간한다. 실패하면 (거기까지의 경로, 사유)."""
    q = np.asarray(q0, float)
    p0, Rs = fk(q)
    if R0 is None:
        R0 = Rs
    p1 = np.asarray(p1, float)
    rot_vec = _rot_error(R0, R1)
    n = max(1, int(math.ceil(float(np.linalg.norm(p1 - p0)) / step)))
    out: List[np.ndarray] = []
    for i in range(1, n + 1):
        u = i / n
        p = p0 + (p1 - p0) * u
        R = _axis_angle(rot_vec * u) @ R0
        r = ik(p, R, q)
        if not r.ok:
            return out, (f"경유점 {i}/{n} IK 실패 (위치 오차 {r.pos_err * 1000:.1f} mm, 회전 {r.rot_err:.3f} rad, "
                         f"한계 여유 {r.limit_margin:.3f} rad)")
        q = r.q
        out.append(q)
    return out, None


def _axis_angle(v: np.ndarray) -> np.ndarray:
    ang = float(np.linalg.norm(v))
    if ang < 1e-12:
        return np.eye(3)
    k = v / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def camera_rotation(tilt_deg: float = 0.0) -> np.ndarray:
    """팔 기준 카메라 회전 — +Y 를 보되 위로 tilt 만큼 든다 (열 = camX, camY, camZ; 카메라는 -Z 를 본다)."""
    c, s = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, -s, -c],
                     [0.0, c, -s]])


def hand_pose_for_tool(tool_p: Sequence[float], R_arm_tool: np.ndarray,
                       T_hand_tool: Sequence[float], R_hand_tool: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """손에 붙은 프레임(도구·카메라·손끝)을 저기 두려면 **손(panda_hand)** 은 어디여야 하나.

    `fk`/`ik` 는 panda_hand 를 모형화한다. Lula 의 `right_gripper` 같은 손끝 프레임을 목표로
    받으면 손 기준 고정 변환 `(T_hand_tool, R_hand_tool)` 으로 손 자세로 바꿔 푼다.
    이 변환은 로봇을 띄운 뒤 Lula FK 두 프레임에서 **한 번 재서** 쓴다 — 손으로 적지 않는다.
    """
    R_arm_hand = np.asarray(R_arm_tool, float) @ np.asarray(R_hand_tool, float).T
    return np.asarray(tool_p, float) - R_arm_hand @ np.asarray(T_hand_tool, float), R_arm_hand


def hand_pose_for_camera(cam_p: Sequence[float], R_arm_cam: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """카메라를 여기 두고 저기를 보게 하려면 손은 어디여야 하나."""
    return hand_pose_for_tool(cam_p, R_arm_cam, T_HAND_CAM, R_HAND_CAM)


@dataclass
class SweepPlan:
    qs: List[np.ndarray] = field(default_factory=list)   # 관절 경로 전체 (진입 + 수평 이동)
    hold_idx: List[int] = field(default_factory=list)    # 정지(촬영)할 경유점 인덱스
    hold_x: List[float] = field(default_factory=list)    # 그 정지점의 카메라 x (팔 기준)
    reason: Optional[str] = None                          # 잘린 이유 (None 이면 끝까지)
    min_margin: float = 0.0                               # 경로 전체의 관절 한계 여유 최소


def plan_sweep(q_start: Sequence[float], shelf_face_y: float, board_z_arm: float,
               x_from: float, x_to: float, cam_standoff: float = 0.35, tilt_deg: float = 0.0,
               points: int = 5, book_center_h: float = 0.12, approach_step: float = 0.04) -> SweepPlan:
    """선반 한 단을 **왼쪽에서 오른쪽으로 수평으로** 훑는 관절 경로.

    카메라는 서가 앞면에서 `cam_standoff` 물러난 평면(y = face − standoff)에, 높이는 판 위 책 중심,
    +Y 를 수평으로(또는 tilt) 보며 x_from → x_to 로 직선 이동한다. `points` 곳에서 멈춰 찍는다.
    진입: 현재 자세 → 첫 카메라 자세는 **관절 공간 직선(moveJ)** — 짧고 예측 가능하다.
    수평 이동: moveL — 손이 직선을 그리므로 카메라 높이·방향이 흔들리지 않는다.
    """
    plan = SweepPlan()
    R_cam = camera_rotation(tilt_deg)
    y = shelf_face_y - cam_standoff
    z = board_z_arm + book_center_h
    xs = np.linspace(x_from, x_to, max(2, points))
    # 진입 자세 (첫 점)
    p_h, R_h = hand_pose_for_camera([xs[0], y, z], R_cam)
    # 첫 자세는 여러 시드로 풀어 **여유가 가장 큰 가지**를 고른다. 그다음부터는 직전 해를 시드로 이어간다.
    r = ik_best(p_h, R_h, seeds_around(q_start), prefer=q_start)
    if not r.ok:
        plan.reason = (f"첫 자세 IK 실패 x={xs[0]:+.2f} (위치 오차 {r.pos_err * 1000:.1f} mm, "
                       f"한계 여유 {r.limit_margin:.3f})")
        return plan
    plan.qs.extend(move_j(q_start, r.q, approach_step))
    plan.hold_idx.append(len(plan.qs) - 1)
    plan.hold_x.append(float(xs[0]))
    q = r.q
    # 수평 이동 — 점 사이는 moveL, 점마다 정지
    for x in xs[1:]:
        p_h, _ = hand_pose_for_camera([x, y, z], R_cam)
        seg, why = move_l(q, p_h, R_h, step=0.02, R0=R_h)
        plan.qs.extend(seg)
        if why is not None:
            plan.reason = f"x={x:+.2f} 로 가다 {why}"
            break
        q = seg[-1]
        plan.hold_idx.append(len(plan.qs) - 1)
        plan.hold_x.append(float(x))
    plan.min_margin = min((limit_margin(qq) for qq in plan.qs), default=0.0)
    return plan


def sweep_recipe(board_z_arm: float, book_center_h: float = 0.12) -> dict:
    """선반판 높이(팔 기준)별 **실측으로 풀리는** 카메라 배치 (2026-09-23, 서가 앞면 0.444 m, 홈 시드).

    · 아래 판(z_arm ≈ 0.22): 카메라를 낮게 두고 수평으로 보면 손이 베이스 옆 한계에 걸려 x 이동이 안 된다.
      **카메라를 0.80 m 높이에 두고 -40° 로 내려다보며** 앞면에서 0.55 m 물러난 평면을 지난다 — 여유 0.17.
    · 가운데 판(z_arm ≈ 0.76): 수평(0°)~-10°, 0.35 m 물러남 — 여유 0.17~0.18.
    카메라 높이는 시선이 앞면의 책 중심을 맞추도록 정한다: z = 책중심 + standoff·tan(−tilt).
    """
    if board_z_arm < 0.5:
        tilt, standoff = -40.0, 0.55
    else:
        tilt, standoff = -10.0, 0.35
    z_cam = board_z_arm + book_center_h + standoff * math.tan(math.radians(-tilt))
    return {"tilt_deg": tilt, "cam_standoff": standoff, "cam_z": z_cam}


def plan_board_sweep(q_start: Sequence[float], shelf_face_y: float, board_z_arm: float,
                     x_from: float = -0.35, x_to: float = 0.35, points: int = 5) -> SweepPlan:
    """한 단 스윕을 레시피대로 계획한다 — 호출자는 판 높이와 앞면 거리만 준다."""
    rc = sweep_recipe(board_z_arm)
    return plan_sweep(q_start, shelf_face_y, rc["cam_z"] - 0.12, x_from, x_to,
                      cam_standoff=rc["cam_standoff"], tilt_deg=rc["tilt_deg"], points=points)


def path_stats(qs: Sequence[np.ndarray]) -> dict:
    """경로 요약 — 이웃 경유점 사이 최대 관절 변화, 전체 길이, 한계 여유 최소."""
    if len(qs) < 2:
        return {"steps": len(qs), "max_step_rad": 0.0, "length_rad": 0.0,
                "min_margin": limit_margin(qs[0]) if qs else 0.0}
    steps = [float(np.max(np.abs(b - a))) for a, b in zip(qs[:-1], qs[1:])]
    return {"steps": len(qs), "max_step_rad": max(steps), "length_rad": sum(steps),
            "min_margin": min(limit_margin(q) for q in qs)}
