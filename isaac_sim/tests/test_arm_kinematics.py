"""arm_kinematics 검증 — Isaac 없이 numpy 만으로 돈다."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib", "controllers"))
import arm_kinematics as ak  # noqa: E402

HOME = np.array([2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033])   # arm.yaml: 트레이 위 30 cm 내려다보기
HOME_I24 = np.array([-2.3634, -0.7300, -0.6933, -2.1884, -0.4396, 1.5845, -0.5540])  # .4 팀이 고른 새 홈 후보


def test_fk_home_looks_down_over_tray():
    p, R = ak.fk(HOME)
    # 손 +Z(접근축) 가 아래를 본다, 손은 팔 뒤쪽(-X) 트레이 위 0.3~0.5 m
    assert R[:, 2] @ np.array([0, 0, -1.0]) > 0.9, R[:, 2]
    assert -0.75 < p[0] < -0.2 and -0.2 < p[1] < 0.35 and 0.25 < p[2] < 0.6, p


def test_fk_i24_same_hand_pose_as_home():
    # .4 팀 실측: i=24 후보는 손 자세가 홈과 같다 (home_tip·HOME_O 불변)
    p0, R0 = ak.fk(HOME)
    p1, R1 = ak.fk(HOME_I24)
    assert np.linalg.norm(p0 - p1) < 0.03, (p0, p1)
    assert np.linalg.norm(ak._rot_error(R0, R1)) < 0.15


@pytest.mark.parametrize("seed", range(6))
def test_ik_roundtrip(seed):
    rng = np.random.default_rng(seed)
    q = ak.Q_MIN + (ak.Q_MAX - ak.Q_MIN) * rng.uniform(0.25, 0.75, 7)
    p, R = ak.fk(q)
    r = ak.ik(p, R, q + rng.normal(0, 0.15, 7))
    assert r.ok, (r.pos_err, r.rot_err, r.limit_margin)
    p2, R2 = ak.fk(r.q)
    assert np.linalg.norm(p2 - p) < 0.003
    assert np.linalg.norm(ak._rot_error(R2, R)) < 0.02


def test_move_l_is_straight_and_continuous():
    p0, R0 = ak.fk(HOME)
    p1 = p0 + np.array([0.15, 0.0, 0.0])
    qs, why = ak.move_l(HOME, p1, R0, step=0.02)
    assert why is None, why
    st = ak.path_stats([HOME] + qs)
    assert st["max_step_rad"] < 0.25, st
    for i, q in enumerate(qs):
        p, _ = ak.fk(q)
        u = (i + 1) / len(qs)
        assert np.linalg.norm(p - (p0 + (p1 - p0) * u)) < 0.004


def test_camera_mount_matches_runtime_camera():
    assert np.allclose(ak.T_HAND_CAM, [0.03, 0.0115, 0.02])
    assert np.allclose(ak.R_HAND_CAM, np.diag([1.0, -1.0, -1.0]))


def test_production_sweep_observes_configured_lower_row():
    """shelf_01 도착 자세에서 지정된 하단 배치 영역을 본다."""
    face = 0.625
    z_arm = 0.497 - 0.286
    rc = ak.sweep_recipe(z_arm)
    plan = ak.plan_board_sweep(HOME, face, z_arm, -0.35, -0.35, points=1)
    assert plan.reason is None, plan.reason
    assert len(plan.hold_idx) == 1
    st = ak.path_stats(plan.qs)
    assert st["max_step_rad"] < 0.3, st            # 튀는 관절 없음
    assert plan.min_margin >= ak.LIMIT_MARGIN, plan.min_margin
    # 정지점마다 카메라가 레시피대로 놓이고, 시선이 앞면의 책 중심을 지나는지
    view = np.array([0.0, math.cos(math.radians(rc["tilt_deg"])), math.sin(math.radians(rc["tilt_deg"]))])
    for idx, x in zip(plan.hold_idx, plan.hold_x):
        cp, cR = ak.fk_camera(plan.qs[idx])
        assert abs(cp[0] - x) < 0.01 and abs(cp[1] - (face - rc["cam_standoff"])) < 0.01 and abs(cp[2] - rc["cam_z"]) < 0.01, cp
        assert (-cR[:, 2]) @ view > 0.99                       # 시선 = -camZ
        hit = cp + view * (rc["cam_standoff"] / view[1])       # 앞면(y = face)과 만나는 점
        assert abs(hit[2] - (z_arm + 0.12)) < 0.02, hit        # = 그 판의 책 중심 높이
    # 손이 트레이·차체 쪽(팔 뒤, 데크 높이)으로 내려가지 않는다.
    for q in plan.qs:
        p, _ = ak.fk(q)
        assert not (p[1] < -0.15 and p[2] < 0.25), p
        cp, _ = ak.fk_camera(q)
        assert cp[2] > 0.55, cp


def test_sweep_reports_reach_limit_instead_of_garbage():
    plan = ak.plan_board_sweep(HOME, 0.625, 0.211, -0.35, 0.35, points=5)
    assert plan.reason is not None                 # 관절 한계 밖 영역은 잘렸다고 보고한다
    assert len(plan.hold_idx) >= 1                 # 안전하게 닿는 첫 관측점은 보존된다
