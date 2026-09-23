"""동작 설정 — **숫자의 유일한 출처**.

지금까지 상수가 호출부에 흩어져 있었다. 그래서 한 곳을 고치면 다른 곳이 조용히
틀어졌다. `SIM_PICK_Y` 만 옮기고 `GOAL_Y` 를 안 옮겨 책을 서가 앞 20 cm 허공에 놓은
사고가 두 번 났다(한 번은 짝 규칙을 넣는 과정에서 냈다).

그래서 두 가지 규칙이 있다:

1. **상수를 박지 않고 관계를 계산한다.** 두 움직이는 것 사이의 *관계*를 상수로 박으면
   한쪽이 움직일 때 반드시 썩는다. 오늘만 베이스가 −3.324 → −3.019 → −3.0695 로
   세 번 바뀌었다.
2. **스위치를 손으로 나열하지 않는다.** 검증 조합은 이름 있는 프리셋으로 고정한다.
   비전팀이 `SIM_GRIP_ROT90` 하나를 안 켜서 그리퍼가 안 돌고 내려가는 것으로 하루를
   썼다 — 손으로 나열하는 방식 자체가 결함이다.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, replace
from typing import Sequence

# --------------------------------------------------------------------- 짝 규칙
#: 베이스를 월드 y 로 Δ 옮기면 팔 기준 삽입 목표를 −Δ 옮겨야 **월드 삽입 지점이 고정**된다.
#: 이 두 값이 그 관계의 기준점이다. **GOAL_Y 를 따로 적지 않는다** — 여기서 만든다.
PAIR_PICK_Y = -3.019
PAIR_GOAL_Y = 0.5495


def goal_y_for(pick_y: float) -> float:
    """선 자리(월드 y) → 팔 기준 삽입 목표 y. 짝 규칙."""
    return PAIR_GOAL_Y + (PAIR_PICK_Y - float(pick_y))


# --------------------------------------------------------------------- 여유 기준
#: **계획 거부선.** 이보다 여유가 적으면 경로 자체를 내놓지 않는다.
#: 근거: `JointPath` 의 완료 판정 오차가 **0.02 rad** 이다. 여유가 그보다 작으면
#: "도착했다" 와 "한계에 붙었다" 를 구분할 수 없다. 그 위 최소값을 잡았다.
REJECT_BELOW = 0.03

#: **합격선.** 계획은 나오되 이 밑이면 `MotionResult.ok=False` 로 판정한다.
#: 근거: AMR 도착 오차 흡수분 ±30 mm × 0.0015 rad/mm = 0.045.
#: 거부선과 나눠 둔 이유 — 합격선을 거부선 자리에 넣으면 **재려던 것을 못 재게 막는다**.
#: (yaml 기본 홈의 전 구간 최소가 0.128 이라 0.15 로 거부하면 기준선조차 못 잰다.)
TARGET_MARGIN = 0.15


@dataclass(frozen=True)
class MotionConfig:
    """한 판에 쓰는 숫자 전부. 프리셋으로 고정하고 인자로만 바꾼다."""

    # --- 자세 -------------------------------------------------------------
    q_home: Sequence[float] = (2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033)
    q_stow: Sequence[float] = (0.0, -1.75, 0.0, -2.8, 0.0, 1.1, 0.79)

    # --- 자리 -------------------------------------------------------------
    #: 파지·반납을 하는 월드 자리. `goal_y` 는 여기서 파생된다 (짝 규칙)
    pick_x: float = 2.535
    pick_y: float = -3.0695
    pick_yaw_deg: float = 0.0

    #: 팔 기준 삽입 칸 x·z. y 는 `goal_y` 로 파생한다
    goal_x: float = -0.3497
    goal_z: float = 0.3399

    # --- 여유 -------------------------------------------------------------
    reject_below: float = REJECT_BELOW
    target_margin: float = TARGET_MARGIN

    # --- 검증 조합 (손으로 나열하지 않는다) --------------------------------
    grip_rot90: bool = True
    grasp_kinematic: bool = True
    #: 콜리전 근사 교정 (2026-09-24 승인, M1~M5 다섯 판). 레벨 파일은 안 고친다
    book_coll: str = "boundingCube"
    shelf_coll: str = "none"
    shelf_row_measure: bool = True
    carry_mode: str = "swing"
    return_mode: str = "swing"
    release_open_first: bool = True
    speed_scale: float = 0.5
    max_step_rad: float = 0.12
    joint_segs: tuple = ("approach", "carry_rotate", "return")

    # --- 스캔 -------------------------------------------------------------
    scan_board_z: float = 0.498
    scan_x_from: float = -0.35
    scan_x_to: float = 0.35
    scan_points: int = 5

    # --- 파지·삽입 ---------------------------------------------------------
    approach_z: float = 0.13
    lift_z: float = 0.17
    insert_depth: float = 0.10
    settle_s: float = 2.0

    @property
    def goal_y(self) -> float:
        """**적지 않는다. 파생한다.** `SIM_GOAL_Y` 가 있으면 그것을 쓴다."""
        env = os.environ.get("SIM_GOAL_Y")
        return float(env) if env else goal_y_for(self.pick_y)

    @property
    def pick_spot(self) -> tuple:
        return (self.pick_x, self.pick_y, self.pick_yaw_deg)

    @property
    def slot_pose_arm(self) -> tuple:
        """팔 기준 삽입 목표 (x, y, z)."""
        return (self.goal_x, self.goal_y, self.goal_z)

    def env(self) -> dict:
        """이 설정에 해당하는 환경변수. **스위치를 손으로 나열하는 대신 여기서 만든다.**"""
        e = {
            "SIM_PICK_Y": f"{self.pick_y:.6g}",
            "SIM_GRIP_ROT90": "1" if self.grip_rot90 else "0",
            "SIM_GRASP_KINEMATIC": "1" if self.grasp_kinematic else "0",
            "SIM_CARRY_MODE": self.carry_mode,
            "SIM_RETURN_MODE": self.return_mode,
            "SIM_RELEASE_OPEN_FIRST": "1" if self.release_open_first else "0",
            "SIM_SPEED_SCALE": f"{self.speed_scale:g}",
            "SIM_MAX_STEP": f"{self.max_step_rad:g}",
            "SIM_JOINT_SEGS": ",".join(self.joint_segs),
            "SIM_BOOK_COLL": self.book_coll,
            "SIM_SHELF_COLL": self.shelf_coll,
            "SIM_SHELF_ROW_MEASURE": "1" if self.shelf_row_measure else "0",
        }
        # **GOAL_Y 는 넣지 않는다** — pick_from_vision 이 SIM_PICK_Y 에서 만든다.
        # 둘을 같이 넣으면 한쪽만 고치는 사고가 다시 난다.
        return e


#: 오늘 검증된 조합 (v11~v15 연속 5/5, 새 레벨 2/2). **이 이름으로 부른다.**
PRESET_DEMO = MotionConfig()

#: 베이스를 옛 자리로 되돌린 것 — 회귀 비교용. 도착 오차에 약하다(+30 mm 에서 여유 0.000)
PRESET_LEGACY = replace(PRESET_DEMO, pick_y=-3.049)

#: 콜리전 교정 **전** 조합 — 9/24 새벽 기준선(열다섯 판). 회귀 비교용.
#: 이것으로 돌리면 꽂힌 책이 2~3° 눕고 선반 판 위 18 mm 에 뜬다. 겹침 임계까지의
#: 예비가 최악 −0.1 mm 라 "통과했다" 가 "여유가 있다" 를 뜻하지 않는다.
PRESET_PRE_COLL_FIX = replace(PRESET_DEMO, book_coll="", shelf_coll="",
                              shelf_row_measure=False)


# --------------------------------------------------------------------- 치환 스위치
def motion_v2(env: str | None = None) -> frozenset:
    """`SIM_MOTION_V2` 를 읽는다 — **어느 구간을 새 경로로 돌릴지**.

    빈 값(기본)이면 **전부 구 경로**다. 이게 이 밤의 안전장치다 — 어떤 구간이 깨져도
    스위치를 비우면 오늘의 검증된 동작으로 돌아온다.

        ""            전부 구 경로
        "home,stow"   그 둘만 새 경로
        "all"         전부 새 경로
    """
    raw = os.environ.get("SIM_MOTION_V2", "") if env is None else env
    raw = (raw or "").strip()
    if not raw:
        return frozenset()
    parts = {p.strip() for p in raw.replace(" ", "").split(",") if p.strip()}
    return frozenset(ALL_SEGMENTS) if "all" in parts else frozenset(parts)


#: 치환 대상 구간 이름. `cycle.py` 의 단계 이름과 같아야 한다
ALL_SEGMENTS = ("home", "stow", "receive", "scan", "pick", "carry", "insert", "retreat")


def uses_v2(segment: str, env: str | None = None) -> bool:
    return segment in motion_v2(env)
