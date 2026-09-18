"""로봇팔 오류 코드 M4xx (팀 문서 03_ros_interfaces.md 9절 범위).

각 코드는 실제로 겪었거나 팀 runbook 이 요구한 실패를 기준으로 정했다.
`PlaceBook` 결과의 error_code·failed_phase·message 로 그대로 나간다.

안전 규칙 (runbook 10절)
- 책을 잡은 상태에서는 임의로 그리퍼를 열지 않는다
- 삽입 중 문제가 생기면 전진을 멈추고 짧게 후퇴한다
- IK 또는 충돌 검사가 실패하면 로봇팔을 움직이지 않는다
"""

from dataclasses import dataclass
from enum import Enum


class Safe(Enum):
    """실패 직후 로봇팔이 취할 안전 동작"""
    HOLD = "HOLD"                    # 움직이지 않는다 (아직 출발 전이거나 계획 단계 실패)
    HOLD_GRIP = "HOLD_GRIP"          # 그 자리 정지, 그리퍼는 연 상태로 두지 않는다 (책을 잡고 있을 수 있음)
    BACK_OFF = "BACK_OFF"            # 전진 중단 후 짧게 후퇴
    RETREAT_HOME = "RETREAT_HOME"    # 빈손이 확인되면 후퇴 후 홈


@dataclass(frozen=True)
class ArmError:
    code: int
    name: str
    phase: str            # PlaceBook 피드백 단계 이름
    retry: bool           # FSM 이 같은 목표로 재시도해도 되는가
    safe: Safe
    meaning: str


M4 = {e.name: e for e in [
    ArmError(401, "IK_FAILED", "PLANNING_GRASP", True, Safe.HOLD,
             "목표 자세에 IK 해가 없다 (도달 범위 밖, 관절 한계)"),
    ArmError(402, "PATH_DISCONTINUOUS", "PLANNING_GRASP", False, Safe.HOLD,
             "사전 계획 경로의 인접점 관절 변화가 기준 초과 — 특이점·해 가지 전환. 실행하지 않는다"),
    ArmError(403, "JOINT_SPEED_EXCEEDED", "", False, Safe.HOLD_GRIP,
             "실행 중 관절 각속도가 URDF 한계의 80% 초과 (튐)"),
    ArmError(404, "MOTION_TIMEOUT", "", True, Safe.HOLD_GRIP,
             "제한 시간 안에 목표에 도달하지 못함 (충돌·추종 실패)"),
    ArmError(405, "GRASP_FAILED", "GRASPING", True, Safe.RETREAT_HOME,
             "닫은 뒤 들어 올렸는데 책이 손과 함께 올라오지 않음"),
    ArmError(406, "BOOK_DROPPED", "MOVING_TO_PRE_INSERT", False, Safe.HOLD_GRIP,
             "운반 중 책이 손에서 이탈 — 사람 확인 필요"),
    ArmError(407, "INSERT_BLOCKED", "INSERTING", True, Safe.BACK_OFF,
             "끼우기·밀기가 목표 깊이에 못 미침 (슬롯 충돌·좁음)"),
    ArmError(408, "RELEASE_DRAG", "RETREATING", True, Safe.HOLD,
             "놓고 빠지는 동안 책이 딸려 나옴 (2026-09-17 실제 발생, 경로 분리로 해결)"),
    ArmError(409, "PLACEMENT_NOT_VERIFIED", "VERIFYING", True, Safe.RETREAT_HOME,
             "최종 책 자세가 슬롯 안·세워짐·책등 방향 조건을 만족하지 않음"),
    ArmError(410, "TARGET_INVALID", "PLANNING_GRASP", False, Safe.HOLD,
             "TargetSlot 이 책보다 좁거나 frame_id 가 arm_base_link 가 아님, 또는 TF 가 오래됨"),
    ArmError(411, "NOT_READY", "", True, Safe.HOLD,
             "선행 조건 미충족 (로봇 초기화 전, 다른 동작 실행 중)"),
    ArmError(412, "CANCELLED", "", True, Safe.HOLD_GRIP,
             "취소·정지 요청으로 중단. 책 보유 여부 확인 후 재개"),
]}


def by_name(name: str) -> ArmError:
    return M4[name]


def by_code(code: int) -> ArmError:
    for e in M4.values():
        if e.code == code:
            return e
    raise KeyError(code)
