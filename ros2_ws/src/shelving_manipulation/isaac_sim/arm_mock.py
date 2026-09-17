"""시뮬레이터 없이 도는 가짜 백엔드.

GPU PC 는 밤에 꺼지고 낮에는 강의로 막혀 있다. 동선 로직을 개인 PC 에서
검증할 수 있어야 개발이 멈추지 않는다.

여기서 검증되는 것: 순서, 전이 조건, 제한 시간, 실패 전파, 취소.
여기서 검증되지 않는 것: 실제 IK 해, 충돌, 물리. 그건 Isaac Sim 에서 본다.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


class MockArmBackend:
    """관절 7개 + 그리퍼를 가진 최소 모델.

    IK 는 "위치를 그대로 따라간다"고 가정한다. 기구학을 흉내 내려 들면
    mock 자체가 버그의 원천이 된다.
    """

    def __init__(self, dt: float = 1.0 / 60.0, ik_fails_at: Optional[np.ndarray] = None,
                 gripper_speed_m_s: float = 0.1):
        self._dt = dt
        self._joints = np.zeros(7, dtype=float)
        self._ee_pos = np.array([0.4, 0.0, 0.4], dtype=float)
        self._ee_quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
        self._gripper = 0.05
        self._gripper_target = 0.05
        self._gripper_speed = gripper_speed_m_s
        # 특정 좌표 근처에서 IK 가 실패하도록 만들어 실패 경로를 시험한다
        self.ik_fails_at = ik_fails_at
        self.commanded_ik = []

    @property
    def dt(self) -> float:
        return self._dt

    def get_joint_positions(self) -> np.ndarray:
        return self._joints.copy()

    def set_joint_targets(self, positions: np.ndarray) -> None:
        self._joints = np.asarray(positions, dtype=float).copy()

    def get_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._ee_pos.copy(), self._ee_quat.copy()

    def compute_ik(self, position, orientation, seed=None):
        position = np.asarray(position, dtype=float)
        if self.ik_fails_at is not None:
            if float(np.linalg.norm(position - self.ik_fails_at)) < 0.02:
                return np.zeros(7), False
        self.commanded_ik.append(position.copy())
        # IK 가 풀렸다는 것은 그 지점에 도달한다는 뜻으로 본다
        self._ee_pos = position.copy()
        self._ee_quat = np.asarray(orientation, dtype=float).copy()
        return np.zeros(7), True

    def set_gripper_width(self, width_m: float) -> None:
        self._gripper_target = float(width_m)

    def get_gripper_width(self) -> float:
        # 즉시 도달하지 않는다. 실제 그리퍼도 시간이 걸리고,
        # 즉시 도달하면 settle 로직이 검증되지 않는다
        delta = self._gripper_target - self._gripper
        step = self._gripper_speed * self._dt
        self._gripper += float(np.clip(delta, -step, step))
        return self._gripper


def run(controller, max_steps: int = 5000):
    """큐가 빌 때까지 굴린다. 테스트 전용 도우미."""
    from arm_primitives import Status

    steps = 0
    while steps < max_steps:
        status = controller.update()
        steps += 1
        if status is Status.FAILED:
            return Status.FAILED, steps
        if controller.idle:
            return Status.SUCCEEDED, steps
    return Status.RUNNING, steps
