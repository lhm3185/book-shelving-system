"""
PlaceBook 실행 추적 — 오류 코드, 단계, 시뮬 작업 명령 규약.

ROS 에 의존하지 않는다. 같은 파일을 manipulation_node(시스템 Python 3.12)와
Isaac Sim 쪽 작업 실행기(내장 Python 3.11)가 함께 import 한다.

실행 구조 (2026-09-17 확정, 방식 '나')
    FSM --PlaceBook action--> manipulation_node --작업 명령(JSON, std_msgs/String)--> Isaac
                                               <--진행 상태(JSON, std_msgs/String)--
    궤적은 Isaac 안에서 사전 계획·실행한다. 매 스텝 관절 목표를 브리지로 보내지 않는다.
"""

from dataclasses import dataclass, field
import json
from typing import Dict, List, Optional

# ------------------------------------------------------------------ 오류 코드 (03 문서 9절 M4xx)

HOLD = 'HOLD'                    # 움직이지 않는다 (출발 전 또는 계획 단계 실패)
HOLD_GRIP = 'HOLD_GRIP'          # 그 자리 정지, 그리퍼를 열지 않는다 (책을 잡고 있을 수 있음)
BACK_OFF = 'BACK_OFF'            # 전진 중단 후 짧게 후퇴
RETREAT_HOME = 'RETREAT_HOME'    # 빈손이 확인되면 후퇴 후 홈


@dataclass(frozen=True)
class ArmError:
    code: int
    name: str
    retry: bool           # FSM 이 같은 목표로 재시도해도 되는가
    safe: str             # 실패 직후 안전 동작
    meaning: str


ERRORS: Dict[int, ArmError] = {e.code: e for e in [
    ArmError(401, 'IK_FAILED', True, HOLD,
             '목표 자세에 IK 해가 없다 (도달 범위 밖, 관절 한계)'),
    ArmError(402, 'PATH_DISCONTINUOUS', False, HOLD,
             '사전 계획 경로의 인접점 관절 변화가 기준 초과. 실행하지 않는다'),
    ArmError(403, 'JOINT_SPEED_EXCEEDED', False, HOLD_GRIP,
             '실행 중 관절 각속도가 URDF 한계의 80% 초과'),
    ArmError(404, 'MOTION_TIMEOUT', True, HOLD_GRIP,
             '제한 시간 안에 끝나지 않음 (충돌·추종 실패·시뮬 응답 없음)'),
    ArmError(405, 'GRASP_FAILED', True, RETREAT_HOME,
             '들어 올렸는데 책이 손과 함께 올라오지 않음'),
    ArmError(406, 'BOOK_DROPPED', False, HOLD_GRIP,
             '운반 중 책이 손에서 이탈 — 사람 확인 필요'),
    ArmError(407, 'INSERT_BLOCKED', True, BACK_OFF,
             '끼우기·밀기가 목표 깊이에 못 미침'),
    ArmError(408, 'RELEASE_DRAG', True, HOLD,
             '놓고 빠지는 동안 책이 딸려 나옴'),
    ArmError(409, 'PLACEMENT_NOT_VERIFIED', True, RETREAT_HOME,
             '최종 책 자세가 칸 안·세워짐·책등 방향 조건을 만족하지 않음'),
    ArmError(410, 'TARGET_INVALID', False, HOLD,
             '목표가 잘못됨 (frame_id, 치수, 칸 폭·높이, 방향, 신뢰도, 작업 범위)'),
    ArmError(411, 'NOT_READY', True, HOLD,
             '선행 조건 미충족 (시뮬 미연결, 트레이 책 없음)'),
    ArmError(412, 'CANCELLED', True, HOLD_GRIP,
             '취소 요청으로 중단. 책 보유 여부 확인 후 재개'),
]}

OK = 0


def error_name(code: int) -> str:
    return ERRORS[code].name if code in ERRORS else ('OK' if code == OK else f'UNKNOWN_{code}')


# ------------------------------------------------------------------ 단계

# PlaceBook 피드백 단계 (03 문서 8절, 순서 고정)
PHASES = (
    'DETECTING_BOOK',
    'PLANNING_GRASP',
    'APPROACHING_BOOK',
    'GRASPING',
    'MOVING_TO_PRE_INSERT',
    'INSERTING',
    'RELEASING',
    'RETREATING',
    'VERIFYING',
)

# 시뮬 내부 동작 단위 → 팀 단계.
# 책등 밀기(touch, push)는 놓은 뒤에 하는 두 번째 삽입이라 INSERTING 으로 다시 보고한다.
# 진행률(progress)은 시뮬 동작 순서로 계산하므로 단계 이름이 되돌아가도 줄지 않는다.
SIM_PHASE_TO_PHASE = {
    'plan': 'PLANNING_GRASP',
    'home': 'APPROACHING_BOOK',
    'approach': 'APPROACHING_BOOK',
    'down': 'APPROACHING_BOOK',
    'grip': 'GRASPING',
    'attach': 'GRASPING',
    'lift': 'GRASPING',
    'carry_rotate': 'MOVING_TO_PRE_INSERT',
    'wedge': 'INSERTING',
    'detach': 'RELEASING',
    'release': 'RELEASING',
    'back': 'RELEASING',
    'touch': 'INSERTING',
    'push': 'INSERTING',
    'retreat': 'RETREATING',
    'return': 'RETREATING',
    'verify': 'VERIFYING',
}

# 시뮬 동작 순서. 진행률 계산과 mock 실행에 쓴다
SIM_PHASE_ORDER = (
    'plan', 'home', 'approach', 'down', 'grip', 'attach', 'lift', 'carry_rotate',
    'wedge', 'detach', 'release', 'back', 'touch', 'push', 'retreat', 'return', 'verify',
)


def phase_of(sim_phase: str) -> str:
    return SIM_PHASE_TO_PHASE.get(sim_phase, 'PLANNING_GRASP')


def progress_of(sim_phase: str) -> float:
    if sim_phase not in SIM_PHASE_ORDER:
        return 0.0
    return round(SIM_PHASE_ORDER.index(sim_phase) / len(SIM_PHASE_ORDER), 3)


# ------------------------------------------------------------------ 시뮬 작업 규약

COMMAND_PLACE = 'place_book'
COMMAND_CANCEL = 'cancel'
#: 서가를 훑어 빈칸을 찾게 한다. **베이스는 그대로**, 팔만 자세를 바꾼다.
#: 각 자세에서 잠깐 멈춰 비전이 찍을 시간을 준다 (dwell_s, 기본 1.0초).
COMMAND_SCAN = 'scan_shelf'
# 서가 수평 스윕 스캔 — arm_kinematics 로 판마다 왼쪽→오른쪽 직선(moveL) 경로를 한 번에 계획해 훑는다 (2026-09-23)
COMMAND_SWEEP = 'scan_sweep'
#: franka 로봇팔 베이스를 90도 회전시킨다. 작업 시작 전에 호출된다.
COMMAND_ROTATE_BASE = 'rotate_base'

# 시뮬 상태 status 값
SIM_IDLE = 'IDLE'
SIM_RUNNING = 'RUNNING'
SIM_SUCCEEDED = 'SUCCEEDED'
SIM_FAILED = 'FAILED'
SIM_CANCELLED = 'CANCELLED'
SIM_TERMINAL = (SIM_SUCCEEDED, SIM_FAILED, SIM_CANCELLED)


def encode(message: dict) -> str:
    return json.dumps(message, ensure_ascii=False, sort_keys=True)


def decode(text: str) -> Optional[dict]:
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def cancel_command(token: str, job_id: str) -> dict:
    return {'type': COMMAND_CANCEL, 'token': token, 'job_id': job_id}


def publish_feedback_safely(goal_handle, message, on_error=None):
    """피드백을 보낸다. **실패해도 작업을 죽이지 않는다.**

    `(보냈나, 왜 안 보냈나)` 를 돌려준다.

    왜: 2026-09-24 에 `manipulation_node` 가 판 도중 죽었다. 스택이 잘린 줄 알았는데
    아니었다 — `_execute → _feedback → publish_feedback` 이 **전체**였고, rclpy 가
    `RCLError` 를 던진 자리가 바로 거기였다. 목표가 이미 끝났거나(취소·중단) 노드가
    내려가는 중이면 핸들이 무효가 되고, 그 예외가 `_execute` 밖으로 튀어 실행
    스레드를 끝낸다.

    **피드백은 진행 상황을 알리는 장식이다.** 그것 때문에 책을 쥔 채로 노드가
    죽으면 안 된다. 그래서 두 겹으로 막는다:

    1. 목표가 이미 끝났으면 **보내지 않는다** (`is_active` 가 있으면 본다)
    2. 그래도 터지면 **삼키고 이유만 남긴다**

    조용히 삼키지는 않는다 — 몇 번 못 보냈는지가 보여야 나중에 원인을 판다.
    """
    active = getattr(goal_handle, 'is_active', True)
    if not active:
        return False, '목표가 이미 끝났다'
    try:
        goal_handle.publish_feedback(message)
        return True, ''
    except Exception as exc:      # noqa: BLE001 - 장식이 작업을 죽이면 안 된다
        why = f'{type(exc).__name__}: {exc}'
        if on_error is not None:
            on_error(why)
        return False, why


def pick_cancel(inbox, token):
    """대기 중인 명령들에서 **내 토큰의 취소만** 꺼낸다.

    `(남은 명령들, 취소가 있었나)` 를 돌려준다. 다른 명령은 건드리지 않는다 —
    꺼내 버리면 제 차례에 처리될 것이 사라진다.

    왜 필요한가: 시뮬 실행기의 긴 동작(회전·자리 맞추기)은 스스로 수백 스텝을
    돌린다. 그동안 평소 명령을 받는 루프가 돌아오지 않아 **취소가 받아지지도
    않는다.** 조작 노드가 30 초에 포기하고 취소를 보내도 시뮬은 끝까지 돈다
    (2026-09-24 VD2: 418 초). 그래서 긴 동작이 중간에 직접 이걸 부른다.
    """
    keep, hit = [], False
    for text in inbox:
        cmd = decode(text)
        if cmd and cmd.get('type') == COMMAND_CANCEL and cmd.get('token') == token:
            hit = True
            continue
        keep.append(text)
    return keep, hit


# ------------------------------------------------------------------ 실행 추적

@dataclass
class Outcome:
    """PlaceBook 결과 필드와 1:1."""

    success: bool
    failed_phase: str
    placement_verified: bool
    error_code: int
    message: str


@dataclass
class PlaceTracker:
    """
    시뮬 상태를 받아 피드백·결과를 만든다. 시각은 호출자가 넣는다(단위시험 가능).

    시뮬이 heartbeat_timeout_s 동안 이 작업의 상태를 보내지 않으면 404 로 끝낸다.
    goal_timeout_s 를 넘기면 취소를 요청하고, 시뮬이 안전 동작을 마친 CANCELLED 를 404 로 바꿔 보고한다.
    """

    token: str
    job_id: str
    started_at: float
    heartbeat_timeout_s: float
    goal_timeout_s: float
    phase: str = 'DETECTING_BOOK'
    progress: float = 0.0
    last_seen: Optional[float] = None
    outcome: Optional[Outcome] = None
    timed_out: bool = False
    cancel_requested: bool = False
    history: List[str] = field(default_factory=list)

    def on_sim_state(self, state: dict, now: float) -> bool:
        """이 작업의 상태면 반영하고 True. 다른 작업·형식 오류는 무시."""
        if self.outcome is not None or state.get('token') != self.token:
            return False
        self.last_seen = now
        sim_phase = str(state.get('phase', ''))
        if sim_phase:
            self.phase = phase_of(sim_phase)
            self.progress = max(self.progress, progress_of(sim_phase))
            if not self.history or self.history[-1] != self.phase:
                self.history.append(self.phase)
        status = state.get('status')
        if status == SIM_SUCCEEDED:
            verified = bool(state.get('placement_verified', False))
            if verified:
                self.progress = 1.0
                self.outcome = Outcome(True, '', True, OK, str(state.get('message', '배치 확인')))
            else:
                self.outcome = Outcome(False, 'VERIFYING', False, 409,
                                       str(state.get('message', '') or ERRORS[409].meaning))
        elif status == SIM_FAILED:
            code = int(state.get('error_code', 0)) or 404
            self.outcome = Outcome(False, self.phase, False, code,
                                   str(state.get('message', '') or error_name(code)))
        elif status == SIM_CANCELLED:
            if self.timed_out:
                self.outcome = Outcome(False, self.phase, False, 404,
                                       f'작업 제한 시간 {self.goal_timeout_s:.0f}s 초과 — 안전 정지 완료')
            else:
                self.outcome = Outcome(False, self.phase, False, 412,
                                       str(state.get('message', '') or '취소 — 안전 정지 완료'))
        return True

    def check(self, now: float) -> Optional[str]:
        """시간 조건 검사. 'cancel' 을 돌려주면 호출자가 시뮬에 취소를 보낸다."""
        if self.outcome is not None:
            return None
        reference = self.last_seen if self.last_seen is not None else self.started_at
        if now - reference > self.heartbeat_timeout_s:
            code = 404 if self.last_seen is not None else 411
            self.outcome = Outcome(False, self.phase, False, code,
                                   f'시뮬 응답 없음 {now - reference:.1f}s'
                                   + ('' if self.last_seen is not None else ' — 작업 실행기 미연결'))
            return None
        if not self.timed_out and now - self.started_at > self.goal_timeout_s:
            self.timed_out = True
            self.cancel_requested = True
            return 'cancel'
        return None

    def request_cancel(self) -> bool:
        """처음 요청일 때만 True (시뮬에 한 번만 보낸다)."""
        if self.outcome is not None or self.cancel_requested:
            return False
        self.cancel_requested = True
        return True


# ------------------------------------------------------------------ mock 실행기

class MockSimExecutor:
    """
    Isaac 없이 같은 상태 규약을 흉내 낸다.

    fail_at: 이 시뮬 동작에서 fail_code 로 실패 (실패 입력 시험용)
    unverified: 모든 동작을 마치고 placement_verified=False 로 끝낸다 (409 시험용)
    """

    def __init__(self, step_s: float = 0.2, fail_at: str = '', fail_code: int = 0,
                 unverified: bool = False):
        self.step_s = step_s
        self.fail_at = fail_at
        self.fail_code = fail_code
        self.unverified = unverified
        self.command: Optional[dict] = None
        self.started_at = 0.0
        self.cancel_at: Optional[float] = None
        self.done: Optional[dict] = None

    def handle(self, command: dict, now: float) -> None:
        if command.get('type') == COMMAND_PLACE:
            self.command, self.started_at, self.cancel_at, self.done = command, now, None, None
        elif (command.get('type') == COMMAND_CANCEL and self.command is not None
              and command.get('token') == self.command.get('token') and self.done is None):
            self.cancel_at = now

    def poll(self, now: float) -> dict:
        if self.command is None:
            return {'token': None, 'status': SIM_IDLE}
        if self.done is not None:
            return self.done
        base = {'token': self.command['token'], 'job_id': self.command.get('job_id', '')}
        index = min(int((now - self.started_at) / self.step_s), len(SIM_PHASE_ORDER))
        if self.cancel_at is not None:
            # 안전 동작(짧은 정지) 한 스텝 뒤 취소 완료
            if now - self.cancel_at >= self.step_s:
                phase = SIM_PHASE_ORDER[min(index, len(SIM_PHASE_ORDER) - 1)]
                self.done = dict(base, status=SIM_CANCELLED, phase=phase, message='mock 취소 — 정지')
                return self.done
        # 폴링 간격이 동작 시간보다 길어도 실패 지점을 건너뛰지 않는다
        if self.fail_at in SIM_PHASE_ORDER and index >= SIM_PHASE_ORDER.index(self.fail_at):
            code = self.fail_code or 404
            self.done = dict(base, status=SIM_FAILED, phase=self.fail_at, error_code=code,
                             message=f'mock 실패 주입 {error_name(code)}')
            return self.done
        if index >= len(SIM_PHASE_ORDER):
            self.done = dict(base, status=SIM_SUCCEEDED, phase='verify',
                             placement_verified=not self.unverified,
                             message='mock 배치 완료' if not self.unverified else 'mock 배치 확인 실패')
            return self.done
        phase = SIM_PHASE_ORDER[index]
        return dict(base, status=SIM_RUNNING, phase=phase)
