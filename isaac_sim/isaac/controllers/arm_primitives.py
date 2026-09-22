"""로봇팔 동작 단위(프리미티브) 정의와 논블로킹 실행기.

설계 원칙 (팀 합의, 2026-09-16)
- **블로킹 함수를 만들지 않는다.** Isaac Sim 스탠드얼론은 매 스텝 `world.step()` 을 도는
  단일 루프라, 동작이 끝날 때까지 기다리는 함수를 쓰면 시뮬 전체가 멈춘다.
  `enqueue()` 로 쌓고 매 스텝 `update()` 가 RUNNING/SUCCEEDED/FAILED 를 돌려준다.
- **FSM 로직을 넣지 않는다.** 여기는 "어떻게 움직일지"만 담는다.
  "언제 무엇을"은 FSM 담당자 코드가 정한다.
- **숫자를 코드에 박지 않는다.** 속도·높이·폭·깊이·제한시간은 전부 `arm_config.yaml` 에 둔다.
  코드에 박으면 튜닝할 때마다 같은 줄에서 merge 충돌이 난다.
- **제한 시간은 스텝 수로 센다.** 벽시계 시간을 쓰면 렌더가 느린 PC 에서 멀쩡한 동작이
  실패로 판정된다.

좌표계 규약
- 모든 Pose 는 `(position[3], quaternion[4])` 이고 쿼터니언은 **(w, x, y, z)** 다.
  Isaac Sim 기본 규약과 같다.
- 기준 프레임은 `arm_config.yaml` 의 `frame` 값이며, 기본은 로봇팔 베이스다.
  **TCP 오프셋은 이 모듈이 처리한다** — 호출자는 "책 중심"을 주면 된다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional, Protocol, Sequence, Tuple

import numpy as np

Pose = Tuple[np.ndarray, np.ndarray]     # (position[3], quaternion[4] wxyz)


class Status(Enum):
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


# ----------------------------------------------------------------- 백엔드

class ArmBackend(Protocol):
    """실제 로봇팔과 mock 이 공유하는 최소 인터페이스.

    이 경계 덕분에 동선 로직을 GPU 없이 단위 테스트할 수 있다.
    """

    def get_joint_positions(self) -> np.ndarray: ...

    def set_joint_targets(self, positions: np.ndarray) -> None: ...

    def get_ee_pose(self) -> Pose: ...

    def compute_ik(self, position: np.ndarray, orientation: np.ndarray,
                   seed: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]: ...

    def set_gripper_width(self, width_m: float) -> None: ...

    def get_gripper_width(self) -> float: ...

    @property
    def dt(self) -> float: ...


# ----------------------------------------------------------------- 컨텍스트

@dataclass
class Context:
    backend: ArmBackend
    config: dict
    step: int = 0                 # enqueue 이후 흐른 스텝 수
    error: str = ""
    error_code: int = 0           # M4xx (arm_errors.py). 0 = 오류 없음

    def cfg(self, *path, default=None):
        """`cfg("speed", "linear")` 처럼 중첩 키를 안전하게 읽는다"""
        node = self.config
        for key in path:
            if not isinstance(node, dict) or key not in node:
                return default
            node = node[key]
        return node

    def steps_for(self, seconds: float) -> int:
        return max(1, int(round(seconds / max(self.backend.dt, 1e-6))))


# ----------------------------------------------------------------- 프리미티브

class Primitive:
    """모든 동작 단위의 기반. 상태를 스스로 들고 있고 매 스텝 갱신된다."""

    name = "primitive"

    def __init__(self, timeout_s: Optional[float] = None):
        self.timeout_s = timeout_s
        self._elapsed = 0
        self._limit = 0
        self._started = False

    # 하위 클래스가 구현한다
    def on_start(self, ctx: Context) -> None: ...

    def on_update(self, ctx: Context) -> Status:
        return Status.SUCCEEDED

    def start(self, ctx: Context) -> None:
        self._elapsed = 0
        timeout = self.timeout_s
        if timeout is None:
            timeout = ctx.cfg("timeout", "default_s", default=10.0)
        self._limit = ctx.steps_for(timeout)
        self._started = True
        self.on_start(ctx)

    def update(self, ctx: Context) -> Status:
        if not self._started:
            self.start(ctx)
        self._elapsed += 1
        if self._elapsed > self._limit:
            ctx.error = f"{self.name}: 제한 시간 초과 ({self._limit} 스텝)"
            ctx.error_code = 404
            return Status.FAILED
        return self.on_update(ctx)

    @property
    def progress(self) -> float:
        if self._limit <= 0:
            return 0.0
        return min(1.0, self._elapsed / self._limit)


class MoveJoint(Primitive):
    """관절 공간 보간 이동. 자세가 크게 바뀌는 이동에 쓴다.

    지령 목표는 **출발 자세에서 목표까지 관절 공간 직선** 위를 사다리꼴 속도로 진행시킨다.
    - 실제 관절 위치를 기준으로 "현재 + 한 걸음"을 주면, 관절이 조금만 뒤처져도 목표가 같이 끌려와
      남은 오차가 영영 줄지 않는다 (2026-09-17 트레이 홈 이동에서 7번 관절이 0.077 rad 남긴 채 시간 초과)
    - 가감속 없이 곧장 최고 속도로 지령하면 위치 제어가 뒤처졌다가 한꺼번에 따라잡아
      실제 각속도가 URDF 한계의 120% 까지 튄다 (같은 날 책 1권 회귀 시험, M403)
    - 모든 관절이 같은 진행률로 움직여 동시에 도착한다 (관절마다 따로 끝나면 경로 모양이 바뀐다)
    """

    name = "move_joint"

    def __init__(self, target: Sequence[float], speed_scale: float = 1.0,
                 timeout_s: Optional[float] = None):
        super().__init__(timeout_s)
        self.target = np.asarray(target, dtype=float)
        self.speed_scale = speed_scale
        self._cmd: Optional[np.ndarray] = None

    def on_start(self, ctx: Context) -> None:
        self._start = np.asarray(ctx.backend.get_joint_positions(), dtype=float).copy()
        self._cmd = self._start.copy()
        n = min(len(self._start), len(self.target))
        self._delta = self.target[:n] - self._start[:n]
        self._length = float(np.max(np.abs(self._delta))) if n else 0.0   # 가장 많이 도는 관절 기준
        self._s = 0.0       # 진행 거리 (rad, 가장 많이 도는 관절)
        self._v = 0.0

    def on_update(self, ctx: Context) -> Status:
        n = len(self._delta)
        max_speed = ctx.cfg("speed", "joint_rad_s", default=1.0) * self.speed_scale
        accel = ctx.cfg("speed", "joint_rad_s2", default=2.0) * self.speed_scale
        dt = ctx.backend.dt
        tolerance = ctx.cfg("tolerance", "joint_rad", default=0.01)

        remaining = self._length - self._s
        if remaining > 1e-12:
            # 남은 거리에서 스텝마다 accel*dt 씩 줄여 정확히 멈출 수 있는 최대 속도 (이산 시간 제동식)
            adt = accel * dt
            v_goal = min(max_speed, adt * (float(np.sqrt(2.0 * remaining / (adt * dt) + 0.25)) - 0.5))
            self._v = float(np.clip(v_goal, self._v - accel * dt, self._v + accel * dt))
            self._v = max(self._v, accel * dt)      # 도착 직전 속도가 0 으로 수렴해 끝나지 않는 것을 막는다
            self._s = min(self._length, self._s + self._v * dt)
            self._cmd[:n] = self._start[:n] + self._delta * (self._s / self._length)
        else:
            self._cmd[:n] = self.target[:n]
        ctx.backend.set_joint_targets(self._cmd)

        commanded_done = self._s >= self._length
        actual = ctx.backend.get_joint_positions()
        if commanded_done and np.all(np.abs(self.target[:n] - actual[:n]) <= tolerance):
            return Status.SUCCEEDED
        return Status.RUNNING


class MoveLinear(Primitive):
    """직교 공간 **직선** 이동.

    슬롯 삽입·후퇴처럼 경로 모양이 중요한 구간에 쓴다.
    RMPflow 는 부드럽지만 직선을 보장하지 않아, 슬롯에 곡선으로 들어가면
    옆 책이나 슬롯 벽을 긁는다.
    """

    name = "move_linear"

    def __init__(self, target: Pose, speed_scale: float = 1.0,
                 timeout_s: Optional[float] = None):
        super().__init__(timeout_s)
        self.target_pos = np.asarray(target[0], dtype=float)
        self.target_quat = np.asarray(target[1], dtype=float)
        self.speed_scale = speed_scale
        self._start_pos: Optional[np.ndarray] = None
        self._distance = 0.0
        self._travelled = 0.0

    def on_start(self, ctx: Context) -> None:
        self._start_pos = np.asarray(ctx.backend.get_ee_pose()[0], dtype=float)
        self._distance = float(np.linalg.norm(self.target_pos - self._start_pos))
        self._travelled = 0.0

    def on_update(self, ctx: Context) -> Status:
        speed = ctx.cfg("speed", "linear_m_s", default=0.1) * self.speed_scale
        tolerance = ctx.cfg("tolerance", "position_m", default=0.002)

        if self._distance <= tolerance:
            return Status.SUCCEEDED

        self._travelled = min(self._distance, self._travelled + speed * ctx.backend.dt)
        ratio = self._travelled / self._distance
        waypoint = self._start_pos + (self.target_pos - self._start_pos) * ratio

        joints, ok = ctx.backend.compute_ik(
            waypoint, self.target_quat, seed=ctx.backend.get_joint_positions())
        if not ok:
            ctx.error = f"{self.name}: IK 실패 (목표 {np.round(waypoint, 4).tolist()})"
            ctx.error_code = 401
            return Status.FAILED
        ctx.backend.set_joint_targets(joints)

        if ratio >= 1.0:
            actual = np.asarray(ctx.backend.get_ee_pose()[0], dtype=float)
            if float(np.linalg.norm(actual - self.target_pos)) <= tolerance:
                return Status.SUCCEEDED
        return Status.RUNNING


class SetGripper(Primitive):
    """그리퍼 폭 지정. 닫힘 판정은 폭이 목표에 닿았는지로 본다."""

    name = "set_gripper"

    def __init__(self, width_m: float, settle_s: Optional[float] = None,
                 timeout_s: Optional[float] = None):
        super().__init__(timeout_s)
        self.width_m = float(width_m)
        self.settle_s = settle_s
        self._settle_left = 0

    def on_start(self, ctx: Context) -> None:
        ctx.backend.set_gripper_width(self.width_m)
        settle = self.settle_s
        if settle is None:
            settle = ctx.cfg("gripper", "settle_s", default=0.3)
        self._settle_left = ctx.steps_for(settle)

    def on_update(self, ctx: Context) -> Status:
        tolerance = ctx.cfg("gripper", "tolerance_m", default=0.004)
        # **폭 도달을 못 읽는 그리퍼가 있다.** RG2 는 폐루프 평행 링크라 명령한 관절각이
        # 그대로 읽히지 않아, 폭 기준으로 기다리면 영원히 끝나지 않는다
        # (2026-09-20 M0609: approach 단계가 제한 시간 초과 — JointPath 가 아니라 여기였다).
        # 그런 로봇은 **정착 시간만** 기다린다.
        if not ctx.cfg("gripper", "check_width", default=True):
            self._settle_left -= 1
            return Status.SUCCEEDED if self._settle_left <= 0 else Status.RUNNING
        reached = abs(ctx.backend.get_gripper_width() - self.width_m) <= tolerance
        if reached:
            # 닿자마자 다음 동작으로 넘어가면 물체가 흔들린 채로 들린다.
            # 잠깐 유지해 안정시킨다
            self._settle_left -= 1
            if self._settle_left <= 0:
                return Status.SUCCEEDED
        return Status.RUNNING


class Wait(Primitive):
    """지정 시간 대기. 물리 안정화용."""

    name = "wait"

    def __init__(self, seconds: float):
        super().__init__(timeout_s=seconds + 1.0)
        self.seconds = seconds
        self._left = 0

    def on_start(self, ctx: Context) -> None:
        self._left = ctx.steps_for(self.seconds)

    def on_update(self, ctx: Context) -> Status:
        self._left -= 1
        return Status.SUCCEEDED if self._left <= 0 else Status.RUNNING


class Sequence(Primitive):
    """프리미티브 여러 개를 순서대로 실행하는 복합 동작.

    grasp·insert 같은 상위 동작을 이걸로 조립한다.
    실패하면 즉시 멈추고 실패한 단계 이름을 남긴다.
    """

    name = "sequence"

    def __init__(self, name: str, steps: List[Primitive]):
        super().__init__(timeout_s=None)
        self.name = name
        self.steps = steps
        self._index = 0

    def start(self, ctx: Context) -> None:
        # 복합 동작은 자체 제한 시간을 두지 않는다. 각 단계가 알아서 판정한다
        self._started = True
        self._index = 0
        self._elapsed = 0
        self._limit = 1 << 30
        if self.steps:
            self.steps[0].start(ctx)

    def update(self, ctx: Context) -> Status:
        if not self._started:
            self.start(ctx)
        if self._index >= len(self.steps):
            return Status.SUCCEEDED

        current = self.steps[self._index]
        status = current.update(ctx)
        if status is Status.FAILED:
            ctx.error = f"{self.name} > {ctx.error or current.name}"
            return Status.FAILED
        if status is Status.SUCCEEDED:
            self._index += 1
            if self._index >= len(self.steps):
                return Status.SUCCEEDED
            self.steps[self._index].start(ctx)
        return Status.RUNNING

    @property
    def current_phase(self) -> str:
        if self._index < len(self.steps):
            return self.steps[self._index].name
        return "done"

    @property
    def progress(self) -> float:
        if not self.steps:
            return 1.0
        return min(1.0, self._index / len(self.steps))


# ----------------------------------------------------------------- 실행기

class ArmController:
    """프리미티브 큐를 들고 매 스텝 한 칸씩 굴린다.

    사용법 (Isaac Sim 루프 안에서):

        arm = ArmController(backend, config)
        arm.enqueue(arm.grasp(book_pose))
        while app.is_running():
            world.step(render=True)
            status = arm.update()
            if status is Status.FAILED:
                print(arm.error)
    """

    def __init__(self, backend: ArmBackend, config: dict):
        self.ctx = Context(backend=backend, config=config)
        self._queue: List[Primitive] = []
        self._active: Optional[Primitive] = None
        self._last_status = Status.SUCCEEDED

    # ---- 큐 조작

    def enqueue(self, primitive: Primitive) -> None:
        self._queue.append(primitive)

    def cancel(self) -> None:
        """현재 동작과 대기 중인 동작을 모두 버린다.

        처음부터 넣어 둔다. 나중에 붙이면 정지 처리가 곳곳에 흩어진다.
        """
        if self._active is not None or self._queue:
            self.ctx.error = "취소됨"
            self.ctx.error_code = 412
        self._queue.clear()
        self._active = None
        self._last_status = Status.SUCCEEDED

    @property
    def idle(self) -> bool:
        return self._active is None and not self._queue

    @property
    def error(self) -> str:
        return self.ctx.error

    @property
    def error_code(self) -> int:
        return self.ctx.error_code

    @property
    def phase(self) -> str:
        if self._active is None:
            return "idle"
        if isinstance(self._active, Sequence):
            return f"{self._active.name}:{self._active.current_phase}"
        return self._active.name

    @property
    def progress(self) -> float:
        return self._active.progress if self._active else 1.0

    def update(self) -> Status:
        """매 시뮬 스텝에 한 번 호출한다."""
        if self._active is None:
            if not self._queue:
                return Status.SUCCEEDED
            self._active = self._queue.pop(0)
            self.ctx.error = ""
            self.ctx.error_code = 0
            self._active.start(self.ctx)

        self.ctx.step += 1
        status = self._active.update(self.ctx)
        self._last_status = status

        if status is Status.SUCCEEDED:
            self._active = None
            return Status.SUCCEEDED if not self._queue else Status.RUNNING
        if status is Status.FAILED:
            # 실패하면 남은 큐를 버린다. 무슨 일이 있었는지 모른 채
            # 다음 동작을 이어가는 것이 가장 위험하다
            self._queue.clear()
            self._active = None
        return status

    # ---- 상위 동작 조립

    def _named_pose(self, key: str) -> Optional[np.ndarray]:
        pose = self.ctx.cfg("poses", key)
        return np.asarray(pose, dtype=float) if pose is not None else None

    def home(self) -> Primitive:
        target = self._named_pose("home")
        if target is None:
            raise KeyError("arm_config.yaml 에 poses.home 이 없다")
        return MoveJoint(target, timeout_s=self.ctx.cfg("timeout", "move_joint_s", default=8.0))

    def stow(self) -> Primitive:
        """주행 안전 자세. **Nav2 목표를 주기 전에 반드시 선행해야 한다.**"""
        target = self._named_pose("stow")
        if target is None:
            raise KeyError("arm_config.yaml 에 poses.stow 가 없다")
        return MoveJoint(target, timeout_s=self.ctx.cfg("timeout", "move_joint_s", default=8.0))

    def grasp(self, book_pose: Pose, width_m: Optional[float] = None) -> Primitive:
        """책 중심 Pose 를 받아 파지까지 끝낸다.

        호출자는 TCP 오프셋을 신경 쓰지 않는다. 여기서 처리한다.
        """
        cfg = self.ctx
        approach = cfg.cfg("grasp", "approach_m", default=0.10)
        lift = cfg.cfg("grasp", "lift_m", default=0.12)
        open_w = cfg.cfg("gripper", "open_m", default=0.05)
        close_w = width_m if width_m else cfg.cfg("gripper", "grasp_m", default=0.012)

        position, quat = np.asarray(book_pose[0], float), np.asarray(book_pose[1], float)
        tcp = np.asarray(cfg.cfg("tcp", "offset_m", default=[0.0, 0.0, 0.0]), float)
        target = position + tcp

        pre = target + np.array([0.0, 0.0, approach])
        up = target + np.array([0.0, 0.0, lift])

        return Sequence("grasp", [
            SetGripper(open_w),
            MoveLinear((pre, quat), timeout_s=cfg.cfg("timeout", "move_linear_s", default=10.0)),
            MoveLinear((target, quat), speed_scale=cfg.cfg("speed", "approach_scale", default=0.4)),
            SetGripper(close_w),
            MoveLinear((up, quat), speed_scale=cfg.cfg("speed", "lift_scale", default=0.5)),
        ])

    def insert(self, slot_pose: Pose, depth_m: Optional[float] = None) -> Primitive:
        """슬롯 앞까지 간 뒤 **직선으로** 밀어 넣고, 놓고, 직선으로 뺀다."""
        cfg = self.ctx
        depth = depth_m if depth_m else cfg.cfg("insert", "depth_m", default=0.12)
        standoff = cfg.cfg("insert", "standoff_m", default=0.15)
        open_w = cfg.cfg("gripper", "open_m", default=0.05)
        axis = np.asarray(cfg.cfg("insert", "axis", default=[1.0, 0.0, 0.0]), float)
        axis = axis / (np.linalg.norm(axis) or 1.0)

        position, quat = np.asarray(slot_pose[0], float), np.asarray(slot_pose[1], float)
        pre = position - axis * standoff
        deep = position + axis * depth

        return Sequence("insert", [
            MoveLinear((pre, quat), timeout_s=cfg.cfg("timeout", "move_linear_s", default=10.0)),
            MoveLinear((deep, quat), speed_scale=cfg.cfg("speed", "insert_scale", default=0.25)),
            SetGripper(open_w),
            Wait(cfg.cfg("insert", "release_wait_s", default=0.3)),
            MoveLinear((pre, quat), speed_scale=cfg.cfg("speed", "retreat_scale", default=0.5)),
        ])
