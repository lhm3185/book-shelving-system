"""동선 로직 단위 테스트. 블렌더도 Isaac Sim 도 필요 없다."""

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ISAAC_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ISAAC_ROOT / "controllers"))

from arm_mock import MockArmBackend, run                                  # noqa: E402
from arm_primitives import (ArmController, MoveJoint, MoveLinear, Sequence,  # noqa: E402
                            SetGripper, Status, Wait)


@pytest.fixture
def config():
    path = ISAAC_ROOT / "config" / "arm.yaml"
    return yaml.safe_load(path.read_text())


@pytest.fixture
def arm(config):
    return ArmController(MockArmBackend(), config)


BOOK = (np.array([0.45, 0.10, 0.20]), np.array([0.0, 1.0, 0.0, 0.0]))
SLOT = (np.array([0.60, -0.10, 0.90]), np.array([0.0, 1.0, 0.0, 0.0]))


# ------------------------------------------------------------ 기본 동작

def test_config_has_required_keys(config):
    for key in ("poses", "speed", "gripper", "grasp", "insert", "tolerance", "timeout"):
        assert key in config, f"config/arm.yaml 에 {key} 가 없다"
    assert len(config["poses"]["home"]) == 7
    assert len(config["poses"]["stow"]) == 7


def test_idle_controller_succeeds(arm):
    assert arm.idle
    assert arm.update() is Status.SUCCEEDED


def test_move_joint_reaches_target(arm):
    arm.enqueue(MoveJoint([0.1] * 7))
    status, steps = run(arm)
    assert status is Status.SUCCEEDED
    assert steps > 1, "즉시 끝나면 보간이 동작하지 않은 것이다"
    assert np.allclose(arm.ctx.backend.get_joint_positions(), 0.1, atol=0.02)


def test_move_linear_follows_straight_path(arm):
    """경유점들이 시작과 끝을 잇는 직선 위에 있어야 한다."""
    backend = arm.ctx.backend
    start = backend.get_ee_pose()[0].copy()
    target = np.array([0.5, 0.2, 0.3])
    arm.enqueue(MoveLinear((target, np.array([0.0, 1.0, 0.0, 0.0]))))
    status, _ = run(arm)
    assert status is Status.SUCCEEDED

    direction = target - start
    direction /= np.linalg.norm(direction)
    for point in backend.commanded_ik:
        offset = point - start
        perpendicular = offset - np.dot(offset, direction) * direction
        assert np.linalg.norm(perpendicular) < 1e-6, "직선을 벗어났다"


def test_gripper_waits_for_settle(arm):
    arm.enqueue(SetGripper(0.012))
    status, steps = run(arm)
    assert status is Status.SUCCEEDED
    assert abs(arm.ctx.backend.get_gripper_width() - 0.012) < 0.005
    # settle 0.3s = 18 스텝, 이동 시간까지 더해 그보다 커야 한다
    assert steps > 18


def test_wait_consumes_time(arm):
    arm.enqueue(Wait(0.5))
    status, steps = run(arm)
    assert status is Status.SUCCEEDED
    assert steps >= 30            # 0.5s / (1/60)


# ------------------------------------------------------------ 상위 동작

def test_grasp_sequence_order(arm):
    """열고 → 접근점 → 책 → 닫고 → 들어올린다."""
    seq = arm.grasp(BOOK)
    assert [step.name for step in seq.steps] == [
        "set_gripper", "move_linear", "move_linear", "set_gripper", "move_linear"]


def test_grasp_executes_every_step(arm):
    """건너뛰는 단계 없이 전부 실행되는지 인덱스 전이로 확인한다."""
    seq = arm.grasp(BOOK)
    arm.enqueue(seq)
    visited = []
    for _ in range(3000):
        arm.update()
        if not arm.idle:
            visited.append(seq._index)
        else:
            break
    assert sorted(set(visited)) == list(range(len(seq.steps)))


def test_grasp_approaches_from_above(arm):
    arm.enqueue(arm.grasp(BOOK))
    status, _ = run(arm)
    assert status is Status.SUCCEEDED
    points = arm.ctx.backend.commanded_ik
    # 첫 경유점은 책보다 위에 있어야 한다
    assert points[0][2] > BOOK[0][2]
    # 마지막은 들어올린 뒤라 다시 위에 있다
    assert points[-1][2] > BOOK[0][2]


def test_insert_goes_deeper_then_retreats(arm, config):
    arm.enqueue(arm.insert(SLOT))
    status, _ = run(arm)
    assert status is Status.SUCCEEDED

    axis = np.asarray(config["insert"]["axis"], dtype=float)
    depths = [float(np.dot(p - SLOT[0], axis)) for p in arm.ctx.backend.commanded_ik]
    assert max(depths) >= config["insert"]["depth_m"] - 1e-6, "삽입 깊이에 못 미친다"
    assert depths[-1] < 0, "마지막에 후퇴하지 않았다"


def test_insert_is_slower_than_free_motion(arm, config):
    """'저속 삽입' 요구사항이 설정에 반영돼 있는지."""
    assert config["speed"]["insert_scale"] < config["speed"]["approach_scale"]
    assert config["speed"]["insert_scale"] < 1.0


# ------------------------------------------------------------ 실패 처리

def test_ik_failure_propagates(config):
    backend = MockArmBackend(ik_fails_at=np.array([0.45, 0.10, 0.30]))
    arm = ArmController(backend, config)
    arm.enqueue(arm.grasp(BOOK))
    status, _ = run(arm)
    assert status is Status.FAILED
    assert "IK 실패" in arm.error
    assert "grasp" in arm.error, "어느 복합 동작에서 실패했는지 남아야 한다"


def test_failure_clears_queue(config):
    backend = MockArmBackend(ik_fails_at=np.array([0.45, 0.10, 0.30]))
    arm = ArmController(backend, config)
    arm.enqueue(arm.grasp(BOOK))
    arm.enqueue(arm.insert(SLOT))
    run(arm)
    assert arm.idle, "실패 후 남은 동작이 계속 실행되면 안 된다"


def test_timeout_fails(arm):
    """도달할 수 없는 관절 목표는 제한 시간에 걸려야 한다."""
    arm.enqueue(MoveJoint([100.0] * 7, timeout_s=0.2))
    status, _ = run(arm)
    assert status is Status.FAILED
    assert "제한 시간" in arm.error


def test_cancel_stops_everything(arm):
    arm.enqueue(arm.grasp(BOOK))
    arm.enqueue(arm.insert(SLOT))
    for _ in range(10):
        arm.update()
    assert not arm.idle
    arm.cancel()
    assert arm.idle
    assert arm.update() is Status.SUCCEEDED


# ------------------------------------------------------------ 큐

def test_queue_runs_in_order(arm):
    arm.enqueue(arm.home())
    arm.enqueue(arm.stow())
    status, _ = run(arm)
    assert status is Status.SUCCEEDED


def test_phase_reports_current_step(arm):
    arm.enqueue(arm.grasp(BOOK))
    arm.update()
    assert arm.phase.startswith("grasp:")


def test_progress_is_monotonic(arm):
    arm.enqueue(arm.grasp(BOOK))
    last = -1.0
    for _ in range(3000):
        arm.update()
        if arm.idle:
            break
        assert arm.progress >= last - 1e-9
        last = arm.progress


def test_stow_differs_from_home(config):
    assert config["poses"]["home"] != config["poses"]["stow"], \
        "주행 자세와 기본 자세가 같으면 stow 의 의미가 없다"


# ------------------------------------------------------------ 오류 코드 M4xx

def test_ik_failure_sets_m401(config):
    backend = MockArmBackend(ik_fails_at=np.array([0.45, 0.10, 0.30]))
    arm = ArmController(backend, config)
    arm.enqueue(arm.grasp(BOOK))
    run(arm)
    assert arm.error_code == 401


def test_timeout_sets_m404(arm):
    arm.enqueue(MoveJoint([100.0] * 7, timeout_s=0.2))
    run(arm)
    assert arm.error_code == 404


def test_cancel_sets_m412(arm):
    arm.enqueue(arm.grasp(BOOK))
    for _ in range(5):
        arm.update()
    arm.cancel()
    assert arm.error_code == 412


def test_error_code_resets_on_next_motion(config):
    backend = MockArmBackend(ik_fails_at=np.array([0.45, 0.10, 0.30]))
    arm = ArmController(backend, config)
    arm.enqueue(arm.grasp(BOOK)); run(arm)
    assert arm.error_code == 401
    arm.enqueue(arm.home()); run(arm)
    assert arm.error_code == 0


def test_error_table_is_consistent():
    from arm_errors import M4, Safe, by_code
    codes = [e.code for e in M4.values()]
    assert len(codes) == len(set(codes)), "코드 중복"
    assert all(400 < c < 500 for c in codes), "M4xx 범위 밖"
    # runbook: 책을 잡고 있을 수 있는 실패는 그리퍼를 열어 버리는 안전동작이면 안 된다
    for name in ("BOOK_DROPPED", "MOTION_TIMEOUT", "JOINT_SPEED_EXCEEDED", "CANCELLED"):
        assert M4[name].safe in (Safe.HOLD_GRIP, Safe.HOLD)
    assert by_code(408).name == "RELEASE_DRAG"


def test_move_joint_closes_residual_when_tracking_lags(config):
    """실제 관절이 지령보다 뒤처져도 목표에 도달해야 한다 (트레이 홈 이동 시간 초과 버그)"""
    class LaggingBackend(MockArmBackend):
        def set_joint_targets(self, positions):
            # 지령과 실제의 차이가 2cm(rad) 미만이면 움직이지 않는다 — 위치 제어의 정상상태 오차를 흉내 낸다.
            # 옛 구현(실제 위치 + 한 걸음 0.01 rad)은 이 영역 안에만 목표를 줘서 영영 움직이지 못했다
            cur = self.get_joint_positions()
            err = np.asarray(positions, float) - cur
            self._joints = cur + np.where(np.abs(err) >= 0.02, err - np.sign(err) * 0.02, 0.0)
    # 실제로 막혔던 조건: 속도 배율 0.6 (한 걸음 0.01 rad), 허용 오차 0.03 rad
    config = dict(config); config["tolerance"] = dict(config["tolerance"], joint_rad=0.03)
    arm = ArmController(LaggingBackend(), config)
    target = [0.5, -0.3, 0.2, -1.0, 0.1, 1.2, -0.08]
    arm.enqueue(MoveJoint(target, speed_scale=0.6, timeout_s=10))
    status, _ = run(arm)
    assert status is Status.SUCCEEDED
    assert np.allclose(arm.ctx.backend.get_joint_positions(), target, atol=0.031)


def test_move_joint_command_never_exceeds_speed(config):
    seen = []
    class Recording(MockArmBackend):
        def set_joint_targets(self, positions):
            seen.append(np.asarray(positions, float).copy()); super().set_joint_targets(positions)
    arm = ArmController(Recording(), config)
    arm.enqueue(MoveJoint([1.0] * 7, speed_scale=0.5))
    run(arm)
    step = config["speed"]["joint_rad_s"] * 0.5 / 60.0
    diffs = [np.max(np.abs(b - a)) for a, b in zip(seen[:-1], seen[1:])]
    assert max(diffs) <= step + 1e-9


def _record_moves(config, target, speed_scale=1.0):
    seen = []
    class Recording(MockArmBackend):
        def set_joint_targets(self, positions):
            seen.append(np.asarray(positions, float).copy()); super().set_joint_targets(positions)
    arm = ArmController(Recording(), config)
    start = arm.ctx.backend.get_joint_positions().copy()
    arm.enqueue(MoveJoint(target, speed_scale=speed_scale))
    status, _ = run(arm)
    return status, start, np.array(seen)


def test_move_joint_accel_limited(config):
    """지령 속도가 한 스텝에 가속도 한계 이상 바뀌지 않는다 (시작·정지 급변 → 각속도 튐 M403)"""
    status, start, seen = _record_moves(config, [1.0, -0.5, 0.3, -1.5, 0.2, 1.4, 0.5])
    assert status is Status.SUCCEEDED
    dt = 1.0 / 60.0
    path = np.vstack([start, seen])
    vel = np.max(np.abs(np.diff(path, axis=0)), axis=1) / dt
    acc = np.abs(np.diff(vel)) / dt
    a_max = config["speed"].get("joint_rad_s2", 2.0)
    assert vel[0] <= a_max * dt * 1.01          # 정지 상태에서 출발
    assert acc.max() <= a_max * 1.01


def test_move_joint_joints_arrive_together(config):
    """관절 공간 직선: 모든 관절이 같은 진행률로 움직인다"""
    target = np.array([0.8, -0.2, 0.0, -1.0, 0.0, 1.0, 0.1])
    status, start, seen = _record_moves(config, target)
    delta = target - start
    moving = np.abs(delta) > 1e-6
    ratios = (seen[:, moving] - start[moving]) / delta[moving]
    assert np.allclose(ratios, ratios[:, :1], atol=1e-9)
