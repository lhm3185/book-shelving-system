"""M0609 운동학 + 받침판 여유 — Isaac 없이 돈다 (numpy, scipy).

값 출처
  관절 원점·축 : isaac_sim/assets/.../doosan-robot2/urdf/m0609.urdf  (그대로 옮김)
  충돌 구      : isaac_sim/assets/.../descriptor/m0609_description.yaml
                 ※ 파일 스스로 "approximate seed values" 라고 적어 둔 근사값이다.
                 ※ link_4 구는 팔뚝 축에서 옆(+y)으로 최대 0.24 m 벗어나 있다 (계산상).
                   Isaac 에서 메쉬와 겹쳐 보기 전에는 여유 '값'을 믿지 말 것.
                   (가지 판정은 구가 아니라 관절 원점 높이로 하므로 이 문제와 무관하다)

좌표: 팔 기준(base_link). 원점 = 받침판 윗면 (deck_z 0.655 == 팔 베이스). 받침판 = z 0 평면.
검산: 홈 [-0.1513,-1.4798,1.2828,-0.0001,-2.9449,2.9901] → link_6 [-0.4748, 0.0788, 0.4117]
      (v21 §2-4 기재값과 0.1 mm 까지 일치)
"""
import itertools
import numpy as np
from scipy.optimize import least_squares

J = [((0, 0, 0.1345), (0, 0, 0)),
     ((0, 0.0062, 0), (0, -1.571, -1.571)),
     ((0.411, 0, 0), (0, 0, 1.571)),
     ((0, -0.368, 0), (1.571, 0, 0)),
     ((0, 0, 0), (-1.571, 0, 0)),
     ((0, -0.121, 0), (1.571, 0, 0))]
NAMES = ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
LO = np.array([-6.2832, -6.2832, -2.618, -6.2832, -6.2832, -6.2832])
HI = np.array([6.2832, 6.2832, 2.618, 6.2832, 6.2832, 6.2832])

SPH = {
    "link_1": [((0, 0, .03), .08), ((0, .03, 0), .08)],
    "link_2": [((.10, 0, .04), .08), ((.20, 0, .09), .07), ((.31, 0, .12), .07), ((.39, 0, .15), .06)],
    "link_3": [((0, 0, .04), .07), ((0, -.01, .09), .06)],
    "link_4": [((0, .05, -.05), .07), ((0, .10, -.14), .07), ((0, .16, -.23), .06), ((0, .24, -.31), .06)],
    "link_5": [((0, 0, .02), .06), ((0, 0, .06), .05)],
    "link_6": [((0, 0, -.03), .05), ((0, 0, -.07), .045)],
}


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _T(xyz, R):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz; return M


def _Rz(q):
    c, s = np.cos(q), np.sin(q); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk_all(q):
    M = np.eye(4); out = {}
    for (xyz, r), qi, n in zip(J, q, NAMES):
        M = M @ _T(xyz, _rpy(*r)) @ _T((0, 0, 0), _Rz(qi))
        out[n] = M.copy()
    return out


def pose6(q):
    M = fk_all(q)["link_6"]; return M[:3, 3], M[:3, :3]


def ik(tp, tR, seed):
    """위치 + 자세 '완전 일치'. 자세 잔차는 R - R_target (외적 합은 180° 뒤집힌 손도 0 이 되므로 쓰지 않는다)."""
    def res(q):
        p, R = pose6(q); return np.concatenate([p - tp, 0.3 * (R - tR).ravel()])
    s = least_squares(res, np.clip(seed, LO + 1e-6, HI - 1e-6), bounds=(LO, HI),
                      xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=3000)
    return s.x, float(np.linalg.norm(s.fun))


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def all_solutions(tp, tR):
    """같은 손 자세를 내는 서로 다른 해 전부. 6축이라 보통 정확히 8개."""
    th = np.arctan2(tp[1], tp[0]); sols = []
    for j1, j2, j3, j4, j5 in itertools.product(
            [th - np.pi/2, th + np.pi/2, th, th + np.pi], np.linspace(-2.8, 2.8, 8),
            [-2.2, -1.0, 1.0, 2.2], [-2.0, 0.0, 2.0], [-1.8, 1.8]):
        q, err = ik(tp, tR, [j1, j2, j3, j4, j5, 0.0])
        if err < 1e-7 and not any(np.max(np.abs(_wrap(q - o))) < 2e-3 for o in sols):
            sols.append(q)
    return sols


def elbow_branch(q):
    """'up' = 팔꿈치가 어깨–손목 선보다 위. 관절 원점만 쓰므로 충돌 구 정확도와 무관."""
    f = fk_all(q)
    sh, el, wr = f["link_2"][:3, 3], f["link_3"][:3, 3], f["link_5"][:3, 3]
    line = wr - sh; t = np.dot(el - sh, line) / np.dot(line, line)
    up = (el - (sh + t * line))[2] > 0
    ua = el - sh
    return ("up" if up else "down"), float(np.degrees(np.arctan2(ua[2], np.hypot(ua[0], ua[1]))))


def deck_clearance(q, plate_half=0.5):
    """받침판 발자국 위에 있는 충돌 구의 (중심 z − 반지름) 최소값과 그 링크."""
    f = fk_all(q); worst = (np.inf, None)
    for n, sph in SPH.items():
        for c, rad in sph:
            p = (f[n] @ np.array([*c, 1.0]))[:3]
            if abs(p[0]) <= plate_half + rad and abs(p[1]) <= plate_half + rad and p[2] - rad < worst[0]:
                worst = (p[2] - rad, n)
    return worst


def sigma_min(q, h=1e-6):
    p0, R0 = pose6(q); Jm = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float); dq[i] += h
        p1, R1 = pose6(dq); dR = R1 @ R0.T
        Jm[:3, i] = (p1 - p0) / h
        Jm[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * h)
    return float(np.linalg.svd(Jm, compute_uv=False)[-1])


def track_line(q0, tp, tR, dz=-0.10, n=40):
    """손 자세 고정, 손을 수직 dz 만큼 직선 이동(movel)하며 같은 가지를 따라간다."""
    q = np.array(q0); jump = 0.0; smin = np.inf; cl = deck_clearance(q)[0]
    for k in range(1, n + 1):
        qn, err = ik(tp + np.array([0, 0, dz * k / n]), tR, q)
        if err > 1e-7:
            return None
        jump = max(jump, float(np.max(np.abs(qn - q))))
        smin = min(smin, sigma_min(qn)); cl = min(cl, deck_clearance(qn)[0]); q = qn
    return {"max_step": jump, "sigma_min": smin, "min_clearance": cl, "q_end": q}
