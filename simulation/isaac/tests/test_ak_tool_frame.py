"""② 의 프레임 산수 — 손끝 목표를 손 자세로 바꾸는 것과 한계 여유 필터 끄기.

`arm_kinematics` 는 손(panda_hand)을 모형화하고 시뮬의 IK 목표는 손끝(right_gripper)으로 온다.
그 사이 고정 변환을 잘못 쓰면 매 해가 10 cm 씩 어긋난다 — 그리고 `check_ak_model` 이
처음에 바로 그 실수를 했다(ee_frame 과 견줌). 여기서 산수를 지킨다.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak  # noqa: E402

HOME = np.array([2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033])
#: 손 기준 손끝 오프셋 흉내 — Franka 손끝은 손 z 로 약 0.1034 m
T_TOOL = np.array([0.0, 0.0, 0.1034])
R_TOOL = np.eye(3)


def test_손끝_목표를_손_자세로_바꾸면_FK_가_그_손끝을_준다():
    """손 자세 q 에서 손끝이 어디인지 알면, 그 손끝을 목표로 되돌린 손 자세는 원래 손 자세다."""
    p_hand, R_hand = ak.fk(HOME)
    p_tool = p_hand + R_hand @ T_TOOL
    R_tool = R_hand @ R_TOOL
    p_back, R_back = ak.hand_pose_for_tool(p_tool, R_tool, T_TOOL, R_TOOL)
    assert np.linalg.norm(p_back - p_hand) < 1e-9
    assert np.abs(R_back - R_hand).max() < 1e-9


def test_손끝_변환을_빼먹으면_10cm_어긋난다():
    """이게 check_ak_model 이 처음 낸 오독이다 — 변환 없이 견주면 모델이 다른 것처럼 보인다."""
    p_hand, R_hand = ak.fk(HOME)
    p_tool = p_hand + R_hand @ T_TOOL
    assert abs(np.linalg.norm(p_tool - p_hand) - 0.1034) < 1e-9


def test_카메라_변환은_같은_산수를_쓴다():
    p_hand, R_hand = ak.fk(HOME)
    cam_p, R_cam = ak.fk_camera(HOME)
    p_back, R_back = ak.hand_pose_for_camera(cam_p, R_cam)
    assert np.linalg.norm(p_back - p_hand) < 1e-9 and np.abs(R_back - R_hand).max() < 1e-9


def test_한계여유_필터를_0_으로_주면_한계_근처_해도_해다():
    """시뮬은 자기 가드(_arm_limits)로 거르므로 ak 안의 0.10 rad 필터는 끈다 — Lula 와 같은 기준."""
    q = ak.Q_MAX - 0.03                      # 한계에서 0.03 rad — 기본 필터(0.10)에 걸린다
    q[3] = ak.Q_MAX[3] - 0.03
    p, R = ak.fk(q)
    strict = ak.ik(p, R, q)
    loose = ak.ik(p, R, q, min_margin=0.0)
    assert not strict.ok and strict.limit_margin < ak.LIMIT_MARGIN
    assert loose.ok and np.linalg.norm(ak.fk(loose.q)[0] - p) < 0.003
