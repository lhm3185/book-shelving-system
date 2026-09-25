"""트레이 → 서가 도서관식 꽂기 장면·계획·실행 조립 (Isaac Sim 5.1.0).

multi_book.py 에서 검증한 코드(트레이 4권 4/4, 튐 0)를 작업 명령 단위로 쓸 수 있게 옮겼다.
SimulationApp 을 만든 뒤에 import 해야 한다.

좌표: 명령은 arm_base_link(= panda_link0) 기준, 물체는 AABB 중심 (FRAMES_CONTRACT).
"""
import math
import os
import sys

import numpy as np

from shelf_gap import book_in_shelf, boards_from_zs, nearest_board, classify_place, side_clearances, skew_deg_from_spans
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
import yaml

# 저장소 코드를 그대로 쓴다 (~/arm 복사 없음): 같은 폴더의 팔 모듈들
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_geometry import R_from_quat, quat_angle, quat_from_R, slerp  # noqa: E402
from arm_planning import carry_ladder, plan_first, tucked_joint_moves  # noqa: E402
from tray_delivery import (  # noqa: E402
    deliver_from_to, drift_mm, TRAY_FROM_FALLBACK, TRAY_TO_XY)
from arm_primitives import ArmController, MoveJoint, Primitive, SetGripper, Sequence, Status, Wait  # noqa: E402

# 로봇마다 다른 이름은 프로파일 한 곳에서 온다 (config/robot_profiles.py).
# 기본은 검증이 끝난 franka. 새 로봇은 ARM_ROBOT=m0609 로 고른다.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"))
from robot_profiles import profile  # noqa: E402

BOT = profile()
R = BOT.root
BASE_LINK = R + "/" + BOT.base_link
HAND_LINK = R + "/" + BOT.hand_link
# 어느 서가 앞에서 꽂는가. 로봇을 다른 서가 앞에 세우면 **앞면 y 가 달라진다** —
# 박아 두면 엉뚱한 서가의 앞면을 읽어 책을 허공에 놓는다 (2026-09-20 실제로 겪었다).
SHELF = os.environ.get("SIM_SHELF_PRIM", "/World/bookshelves/shelf_brown__book_shelf_01")
# 꽂을 선반판 윗면의 **월드 z**. 로봇 키가 바뀌면 같은 계약 z 라도 다른 단을 가리킨다
# (M0609 는 팔 베이스가 37 cm 높아 Franka 가 쓰던 단이 팔 기준 -0.04 가 된다).
SHELF_ROW_Z = float(os.environ.get("SIM_SHELF_ROW_Z", 0.355 * 1.4))
BOOK_SRC = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
ARM_JOINTS = BOT.arm_joints
FINGERS = BOT.grip_joints
TRAY_FOLLOW_MIN_M = float(os.environ.get("SIM_TRAY_FOLLOW_MIN", "0.002"))

#: 반납기에서 트레이가 로봇 위로 미끄러져 오는 연출. 레벨에 반납기 트레이가 있을 때만 돈다.
#: `SIM_TRAY_DELIVERY=0` 으로 끄면 예전처럼 트레이가 처음부터 로봇 위에 있다.
TRAY_DELIVERY = os.environ.get("SIM_TRAY_DELIVERY", "1") != "0"
#: 레벨에서 반납기 위 트레이를 찾을 경로 (이것이 출발 자리가 된다)
#:
#: **기본값이 `tray_v1` 이라 사고가 났다** (2026-09-22). 레벨에는 `tray_v3` 가 들어 있는데
#: 환경변수를 안 주면 없는 `tray_v1` 을 찾고, 못 찾으면 **조용히 옛 에셋을 스폰하는 경로로
#: 빠졌다.** 그래서 화면에는 스케일도 콜리전도 다른 트레이가 나왔고, 로그는 멀쩡했다.
#: 기본값을 레벨과 맞추고, 없으면 아래에서 **죽는다** (조용한 대체 금지).
KIOSK_TRAY = os.environ.get("SIM_KIOSK_TRAY", "/World/tray_books/tray_v3")
#: 미끄러져 오는 데 걸리는 시간 (초). 컨베이어처럼 일정한 속도로 온다
TRAY_DELIVERY_S = float(os.environ.get("SIM_TRAY_DELIVERY_S", "2.5"))
#: 이송을 **시작하기 전에** 기다리는 시간 (초).
#: 레벨의 책은 트레이 바닥에서 살짝 떠 있다. 시뮬이 시작되면 떨어져 칸에 앉는데,
#: 그 전에 트레이가 출발하면 **책만 제자리에 남는다** (2026-09-22 실측).
TRAY_SETTLE_S = float(os.environ.get("SIM_TRAY_SETTLE_S", "2.0"))

def _xyz(name, default):
    """환경변수 "x,y,z" → 배열. 비어 있으면 기본값."""
    v = os.environ.get(name, "").strip()
    if not v:
        return np.array(default, float)
    return np.array([float(t) for t in v.replace(" ", "").split(",")], float)

#: 이송 시작·도착 좌표 (월드). **레벨이 정답이다** — 판단은 `tray_delivery` 에 있다.
#: 여기서 상수로 박았다가 레벨이 바뀌자 그대로 썩었다 (2026-09-23 밤의 410·409).
TRAY_FROM = _xyz("SIM_TRAY_FROM", TRAY_FROM_FALLBACK)
TRAY_TO = _xyz("SIM_TRAY_TO", list(TRAY_TO_XY) + [TRAY_FROM_FALLBACK[2]])
#: 트레이 위치를 주기적으로 찍는다 (떨어지는 시점을 잡기 위해)
TRAY_WATCH = os.environ.get("SIM_TRAY_WATCH", "1") != "0"
#: **기준선 모드.** 트레이·책에 코드가 아무것도 하지 않는다 (이송·추종·계측 전부 꺼짐).
#: "책이 칸에 들어가는가" 만 눈으로 보기 위한 것이다. 기본은 꺼짐.
HANDS_OFF = os.environ.get("SIM_HANDS_OFF", "0") != "0"
#: 주행 중 트레이·책을 **좌표로** 끌고 다니는 부분만 끈다 (`SIM_TRAY_CARRY=0`).
#: `SIM_TRAY_FOLLOW` 와 다르다 — 그쪽은 앵커 설정 자체를 건너뛰어 **이송까지 꺼진다.**
#: 끄면 트레이는 마찰로만 실려 간다. 물리적으로 맞는 방식이고 화면도 자연스럽다.
TRAY_CARRY = os.environ.get("SIM_TRAY_CARRY", "1") != "0"
#: 이송 중 트레이를 데크에서 띄워 둘 높이 (m).
#: **키네마틱 트레이가 데크에 닿은 채 움직이면 동적인 로봇을 밀어 버린다**
#: (2026-09-22 실측: 로봇이 −x 로 밀려나면서 트레이를 받았다).
#: 띄운 채로 목표에 멈춘 뒤 동적으로 바꾸면 이 높이만큼만 내려앉는다.
TRAY_CLEAR_M = float(os.environ.get("SIM_TRAY_CLEAR", "0.004"))
#: **레벨에 놓인 트레이와 그 위의 책을 그대로 쓴다.** 새로 만들지 않는다.
#: 레벨에 이미 책이 칸에 꽂혀 있는데 굳이 복제본을 만들어 세워 넣을 이유가 없다 —
#: 그 과정에서 자세·치수·콜리전이 원본과 달라진다.
USE_LEVEL_TRAY = os.environ.get("SIM_USE_LEVEL_TRAY", "1") != "0"
#: 꽂은 책이 **옆 책 속에 박혔는지**를 기하로 본다 (`jam_report`).
#: 서가 낱권 책에는 콜리전이 없어서 물리로는 절대 안 막힌다 — 끄면 꽉 찬 칸에
#: 꽂아도 성공으로 보고된다. 그래서 기본은 켜짐이다.
JAM_CHECK = os.environ.get("SIM_JAM_CHECK", "1") != "0"
#: 허용 침투 (m). 서가 책들끼리 이미 0.7~4.6 mm 겹쳐 있어서(9/23 실측) 그보다 낮출 수 없다.
JAM_TOL = float(os.environ.get("SIM_JAM_TOL", "0.005"))
#: 꽂힌 책의 **수평 비뚤어짐** 허용치(도). 계약의 `yaw_tolerance` 0.10 rad = 5.73° 를
#: 그대로 쓴다 — 새로 지어낸 값이 아니라 계획이 이미 쓰던 값이다.
#: 실측 정상은 0.34~0.40° 라 14배 여유가 있다.
#: 왜 따로 보나: `upright` 는 **z 높이만** 본다. 책이 z 축 둘레로 90° 돌아 누워도
#: 높이는 그대로라 통과한다 (2026-09-24 LIVE4: 85.8° 로 꽂혔는데 upright True).
#: 그때 `depth` 하나가 잡았다 — **다섯 중 하나뿐**이면 여유가 없다.
SKEW_TOL_DEG = float(os.environ.get("SIM_SKEW_TOL_DEG", "5.73"))
#: 서가에 **빈칸을 더 만든다** — `SIM_SHELF_GAP="<시작 인덱스>,<목표 폭 mm>[,<층>]"`.
#:
#: **레벨에는 이미 빈칸이 있다.** 판 z 1.581 에 81.1 / 59.9 / 36.8 / 36.1 mm,
#: 판 z 1.042 의 오른쪽 옆판 옆에 96.0 mm (2026-09-23 실측, `measure_shelf.py`).
#: 그러니 이 스위치는 "없는 빈칸을 만든다" 가 아니라 **원하는 폭의 빈칸이 없을 때
#: 골라서 낸다** 는 뜻이다. 있는 빈칸으로 충분하면 안 써도 된다.
#:
#: 왜 폭 기준인가: 서가 책 두께가 7.7 mm(잡지)~44.8 mm(하드커버)로 **6배** 차이 난다.
#: "몇 권 빼기"로 정의하면 같은 지시가 전혀 다른 난이도가 된다 — 잡지 한 권을 빼면
#: 7.7 mm 구멍, 하드커버 한 권을 빼면 44.8 mm 구멍이다. 그래서 **열고 싶은 폭**을
#: 말하면 필요한 최소 권수를 알아서 뺀다.
#:
#: 층 기본값은 `secondFloor` 다. 층 이름과 실제 높이는 레벨이 바뀌면 달라지므로
#: **`config/ground_truth_slots.yaml` 을 보고 정할 것** (그 파일도 실측에서 만든다).
SHELF_GAP = os.environ.get("SIM_SHELF_GAP", "").strip()
DECK_Z = BOT.deck_z         # 트레이가 놓이는 면의 월드 높이 (로봇별)
# 링크가 이보다 낮으면 받침판을 뚫는 것으로 본다 (팔 베이스가 판 위에 바로 붙어 있다)
# 팔이 받침판을 이만큼까지 파고드는 것은 눈감아 준다 (충돌 구가 근사값이라 여유가 필요하다)
DECK_SINK_M = float(os.environ.get("SIM_DECK_SINK", "0.02"))
TIP_DOWN = 0.035            # 책등 윗면에서 손끝이 내려가 잡는 깊이
#: 파지 직전에 책을 세워 바로잡는 문턱 (rad). 이보다 작게 기운 것은 건드리지 않는다 —
#: 물고 있는 책을 돌리면 그 회전이 곧 손 안에서의 어긋남이 되어 `406` 이 난다
UPRIGHT_SNAP_RAD = math.radians(float(os.environ.get("SIM_UPRIGHT_SNAP_DEG", "25")))
#: 파지 물림축을 90° 돌린다. 칸이 늘어선 방향이 팔 기준 x 라고 가정해 왔는데, 이 레벨의
#: 트레이는 책의 얇은 축이 y 를 향한다 (2026-09-22 실측: 회전 뒤 책 크기 [0.231, 0.059, 0.154]).
#: 그대로 두면 손가락이 책 **옆면**을 물어 운반 중 미끄러진다 (`406`).
GRIP_ROT90 = os.environ.get("SIM_GRIP_ROT90", "0") != "0"
#: 파지·삽입 **작업 중에만** 차체를 월드에 고정한다 (`SIM_FIX_BASE=0` 으로 끈다).
#: 로봇은 베이스가 떠 있는 아티큘레이션이라 팔이 크게 돌면 반작용으로 차체가 들린다.
#: 원래는 바퀴 접촉·마찰이 눌러 주는데, 이 시뮬은 차체를 물리로 굴리지 않고
#: `set_world_pose` 로 순간이동시켜서 접촉이 제 일을 못 한다 (2026-09-22 실측: 팔이
#: 도는 동안 AMR 이 바닥에서 떴다). **주행 중에는 풀어 둔다** — 고정한 채 순간이동시키면
#: 조인트와 싸워 주행이 깨진다.
FIX_BASE = os.environ.get("SIM_FIX_BASE", "1") != "0"
#: 작업 중 **아티큘레이션 자세를 매 스텝 다시 쓰는** 방식으로도 차체를 고정할 것인가.
#: **기본 꺼짐.** 이 로봇에는 맞지 않는다.
#:
#: 이 AMR 은 **떠 있는 베이스**라 차체가 루트 변환이 아니라 `dummy_base_x/y` 프리즈매틱
#: 조인트로 움직인다 (`base_idx`). 그런데 `hold_base_tick` 은 `robot.set_world_pose()` 로
#: **아티큘레이션 프레임 자체**를 붙잡는다. 조인트는 그대로 값을 들고 있으므로 둘이
#: 싸우고, 팔 베이스가 차대 원점 쪽으로 끌려간다.
#:
#: 2026-09-24 실측 (비전 브랜치를 합친 뒤 이 호출이 처음 살아났다):
#:     팔 베이스 월드 x  2.8349 → 2.5730   (**262 mm**)
#:     팔–차대 오프셋    +0.300 → +0.038   (같은 262 mm — 오프셋이 통째로 먹혔다)
#:     꽂을 월드 x       2.496 → 2.234
#:     그 뒤 관절 7개 NaN · articulation 붕괴 (panda_link0 · base_link · world ·
#:     dummy_base_x/y 까지 `Invalid PhysX transform`)
#:
#: 차체 고정은 `lock_base_mass()`(질량을 올려 팔 반작용에 안 들리게)가 한다. 그쪽은
#: 물리 그대로라 솔버와 싸우지 않는다 — 열다섯 판이 그 방식으로 통과했다.
#: **떠 있는 베이스에서는 켜지 말 것.** 2026-09-24 에 켠 판(V3)과 끈 판(V4)을 견줬다:
#:     켜짐   `Invalid PhysX transform` **28,202줄** · 관절 7개 NaN · articulation 붕괴
#:     꺼짐   **0줄** · 삽입까지 정상 진행
#: 켜려면 `SIM_HOLD_BASE_TICK=1`. 떠 있는 베이스가 아닌 로봇에서만 의미가 있다.
HOLD_BASE_TICK = os.environ.get("SIM_HOLD_BASE_TICK", "0") != "0"
#: 고정할 차체 링크
BASE_LOCK_LINK = os.environ.get("SIM_BASE_LOCK_LINK", "base_link")
#: 차체 링크에 줄 질량 (kg). 팔 반작용으로 들리지 않을 만큼 무겁게 한다.
#: 조인트로 묶으면 솔버와 싸워 **덜덜 떨다 주저앉고**, 루트 XForm 을 붙잡으면
#: 아티큘레이션 링크가 따라오지 않아 **효과가 없다** (둘 다 2026-09-22 실측).
#: 질량은 솔버와 싸우지 않는다. 실제 AMR 도 무겁다.
BASE_MASS_KG = float(os.environ.get("SIM_BASE_MASS", "300"))
#: 모든 구간 속도에 곱하는 배율. 낮추면 관절 각속도 한계(`403`)에 덜 걸린다.
#: 1.0 이 지금까지 쓰던 값이다 — 기본값을 바꾸지 않는다.
SPEED_SCALE = float(os.environ.get("SIM_SPEED_SCALE", "1.0"))
GRIP_CLEAR = 0.005
SPINE_INSET = 0.02          # 계획상 최종 책등이 서가 앞면에서 들어가는 거리
MEASURED_INSET = 0.024      # 실측 최종 책등 위치 (밀기 후). 꽂힌 책 AABB 중심 → 서가 앞면 역산에 쓴다
DIV_H, DIV_T, DIV_GAP = 0.07, 0.01, 0.003
VEL_LIMIT = np.array(BOT.vel_limit)             # URDF 실측 (로봇별, 프로파일에서)
# 계획 인접점 최대 관절 변화 (초과 = 불연속, M402). 순간이동을 잡으려는 값이지,
# 빠른 구간을 막으려는 값이 아니다. 손목 특이점에서 0.124 rad(7°) 가 남는데
# 32배까지 잘게 나눠도 줄지 않는다 — 진짜 작은 불연속이다 (2026-09-21 실측).
MAX_STEP = float(os.environ.get("SIM_MAX_STEP", "0.12"))
#: 스윙 되돌림에서 손목을 돌리는 반지름 (팔 기준, m). 오프라인 표에서 HORIZ 여유가 0.6 대로
#: 넉넉하고, 매달린 책 끝(y +0.19)이 서가 앞면(0.444)에 못 미치는 값 (2026-09-25).
SWING_TURN_R = 0.50
# 관절 경로 제한 시간. **안전장치이지 성능 목표가 아니다** — 팔이 명령 속도를
# 그대로 따라가지 못하면 계산한 시간보다 오래 걸린다. 2.0/3.0 으로는 팔이
# 경유점 24/36 까지 잘 가고 있는데도 404 로 잘렸다 (2026-09-21 실측).
TIME_FACTOR = float(os.environ.get("SIM_TIME_FACTOR", "3.5"))
TIME_PAD = float(os.environ.get("SIM_TIME_PAD", "5.0"))
GRASP_JOINT = "/World/bs_grasp_joint"
# 잡은 책을 키네마틱으로 들고 가 볼 수 있다. **기본은 꺼짐** — 켜 봤더니 더 나빠졌다
# (2026-09-21 실측: 책이 뒤집혀 돌고 57.6cm 떨어졌다. 끄면 4.7cm 로 일정하다).
# **원인은 아직 모른다.** 한때 "책 prim 아래에 강체가 하나 더 있다"고 적었는데 **틀렸다** —
# 에셋에는 물리 API 가 전혀 없고(usdc 토큰 확인), 강체는 우리가 book_i 에 하나만 붙인다.
# 돌아가는 중인 동적 강체를 키네마틱으로 바꾸는 것 자체가 문제일 수 있다.
GRASP_KINEMATIC = os.environ.get("SIM_GRASP_KINEMATIC", "0") != "0"
# 그리퍼 접근축을 물림축과 직교화한다 (기본 꺼짐 — 파지 자세가 90° 바뀐다)
GRIP_ORTHO = os.environ.get("SIM_GRIP_ORTHO", "0") != "0"
# IK 결과가 **팔꿈치↑ 가지**인지 검사한다. 끄면 팔꿈치↓ 해가 조용히 섞인다
BRANCH_GUARD = os.environ.get("SIM_BRANCH_GUARD", "1") != "0"
# 홈을 어떻게 정하나. ik = 트레이 위 한 점으로 IK (옛 방식, 실행 검증됨)
#                  joints = YAML 관절각 + FK (자기모순 없음, 실행은 아직 안 됨)



def named(primitive, name):
    """시뮬 상태 보고용 동작 이름 (book_placer.SIM_PHASE_ORDER 와 같은 이름)"""
    primitive.name = name
    return primitive


class JointPath(Primitive):
    """미리 계획한 관절 경유점을 일정한 관절 속도로 따라간다 (실행 중 IK 없음)"""

    def __init__(self, name, qs, speed=0.35):
        length = sum(float(np.max(np.abs(b - a))) for a, b in zip(qs[:-1], qs[1:]))
        self._plan_len = length
        super().__init__(max(6.0, length / speed * TIME_FACTOR + TIME_PAD))
        self.name = name
        self.qs = [np.asarray(q, float) for q in qs]
        self.speed = speed

    def on_start(self, ctx):
        self._pts = [ctx.backend.get_joint_positions()] + self.qs
        self._i = 0
        self._target = self._pts[0].copy()
        # 제한 시간은 만들 때 **계획된 경유점만으로** 계산했다. 실제로는 현재 자세에서
        # 첫 경유점까지의 간격이 더 붙는데, IK 가 먼 해를 고르면 그 간격이 3 rad 를 넘는다
        # (2026-09-20 M0609: 경로 길이 0.7 rad 인데 첫 간격만 3.4 rad → 제한 시간 초과).
        # 실제 경로 길이로 다시 잡는다.
        real = sum(float(np.max(np.abs(b - a))) for a, b in zip(self._pts[:-1], self._pts[1:]))
        need = max(6.0, real / self.speed * TIME_FACTOR + TIME_PAD)
        self._limit = max(getattr(self, "_limit", 0), ctx.steps_for(need))
        if real > 1.5:
            ctx.backend.say(f"[진단] {self.name}: 실제 경로 {real:.2f} rad "
                            f"(계획 {self._plan_len:.2f}), 제한 {need:.0f}초로 확장")

    def on_update(self, ctx):
        budget = self.speed * ctx.backend.dt
        while budget > 1e-9 and self._i < len(self._pts) - 1:
            nxt = self._pts[self._i + 1]
            gap = float(np.max(np.abs(nxt - self._target)))
            if gap <= budget:
                self._target = nxt.copy(); self._i += 1; budget -= gap
            else:
                self._target = self._target + (nxt - self._target) * (budget / gap); budget = 0.0
        ctx.backend.set_joint_targets(self._target)
        # 도착 판정은 **설정의 허용 오차**를 쓴다. 예전에는 0.02 가 박혀 있어서
        # 정착 오차가 그보다 큰 로봇(M0609 는 0.06)에서는 영원히 도착하지 못하고
        # 제한 시간 초과(M404)가 났다 (2026-09-20 실측).
        tol = ctx.cfg("tolerance", "joint_rad", default=0.02)
        now = ctx.backend.get_joint_positions()
        err = float(np.max(np.abs(now - self._pts[-1])))
        if self._i >= len(self._pts) - 1 and err <= tol:
            return Status.SUCCEEDED
        # [진단] 왜 안 끝나는지 주기적으로 남긴다 — 경유점을 못 넘는 것인지,
        # 넘었는데 팔이 안 따라오는 것인지 가른다
        self._n = getattr(self, "_n", 0) + 1
        if self._n % 120 == 0:
            say = getattr(ctx.backend, "say", None)
            msg = (f"[진단] {self.name}: 경유점 {self._i}/{len(self._pts)-1}, "
                   f"최대오차 {err:.4f} (허용 {tol}), "
                   f"현재 {np.round(now, 3).tolist()} 목표 {np.round(self._pts[-1], 3).tolist()}")
            (say or print)(msg)
            # 경유점을 다 넘었는데도 팔이 안 따라오면 **무엇엔가 막힌 것**이다.
            # 관절값만 보면 어느 물체인지 영영 모른다 — 겹치는 대상을 같이 남긴다
            # (2026-09-20: 서가인 줄 알고 로봇을 두 번 옮겼는데 아니었다).
            clash = getattr(ctx.backend, "clash_report", None)
            if clash is not None and self._i >= len(self._pts) - 1:
                (say or print)(clash())
        return Status.RUNNING


class Call(Primitive):
    def __init__(self, name, fn):
        super().__init__(timeout_s=1.0)
        self.name = name
        self._fn = fn

    def on_update(self, ctx):
        self._fn()
        return Status.SUCCEEDED


def _resolve_kiosk_tray(stage, say):
    """레벨에서 반납기 트레이를 찾아 `KIOSK_TRAY` 를 확정한다. **이름을 고정하지 않는다.**

    전에는 `tray_v1` 이 하드코딩돼 있었고, 레벨에 그 이름이 없자 코드가 **말없이 옛 에셋을
    스폰**했다. 화면에는 스케일도 콜리전도 다른 트레이가 나오는데 로그는 멀쩡해서 며칠을
    엉뚱한 곳에서 원인을 찾았다 (2026-09-22). 에셋을 v3→v4 로 바꿀 때마다 같은 사고가 난다.

    **어떤 사용보다도 먼저** 불러야 한다. 늦게 부르면 앞쪽 코드가 옛 이름으로 이미 판단해
    버린다.
    """
    path = KIOSK_TRAY
    if stage.GetPrimAtPath(path).IsValid():
        return path
    folder = stage.GetPrimAtPath(path.rsplit("/", 1)[0])
    cands = [c for c in (folder.GetChildren() if folder.IsValid() else [])
             if c.GetTypeName() == "Xform" and "tray" in c.GetName().lower()]
    if len(cands) == 1:
        globals()["KIOSK_TRAY"] = str(cands[0].GetPath())
        say(f"**트레이를 이름으로 못 찾아 폴더에서 찾았다**: {KIOSK_TRAY} "
            f"(원래 찾던 경로 {path} 가 레벨에 없다)")
    elif len(cands) > 1:
        raise RuntimeError(f"트레이 후보가 여럿이다: {[c.GetName() for c in cands]} — "
                           f"SIM_KIOSK_TRAY 로 하나를 지정할 것")
    return KIOSK_TRAY


class BookScene:
    """레벨 + 트레이 + 책 N권 + 북엔드. 로봇·IK·팔 제어기까지 준비한다"""

    def __init__(self, app, usd, tray_usd, tray_center, n_books, place_dx, say, before_reset=None,
                 book_variants=None):
        self.app, self.say = app, say
        open_stage(usd); app.update()
        while is_stage_loading():
            app.update()
        st = self.stage = get_current_stage()
        # **켜진 스위치를 한 줄로 남긴다.** 환경변수가 실제로 전달됐는지 로그로 확인할 수
        # 없어서, 켰다고 믿고 엉뚱한 곳을 판 적이 있다 (2026-09-22).
        say(f"스위치: 레벨트레이={USE_LEVEL_TRAY} 이송={TRAY_DELIVERY} 추종={TRAY_CARRY} "
            f"손대지않음={HANDS_OFF} 키네마틱파지={GRASP_KINEMATIC} 그리퍼90도={GRIP_ROT90} "
            f"차체고정={FIX_BASE} 관절구간={os.environ.get('SIM_JOINT_SEGS', '(기본)')} "
            f"운반={os.environ.get('SIM_CARRY_MODE', 'joint')} 복귀={os.environ.get('SIM_RETURN_MODE', 'joint')} "
            f"경로검사={os.environ.get('SIM_PATH_AUDIT', '0')} 겹침검사={JAM_CHECK}")
        _resolve_kiosk_tray(st, say)
        self._cache = create_bbox_cache()
        # 서가 빈칸은 **원래 없다** — 있어야 하면 우리가 낸다 (SIM_SHELF_GAP, 런타임 비활성)
        self.shelf_gap = None
        self.apply_shelf_gap()

        # 책 원본: 기본은 한 종류, --book-variants 를 주면 레벨 /World/books 의 여러 종류를 돌려 쓴다
        sources = []
        for name in (book_variants or [BOOK_SRC]):
            # **레벨이 평탄화(flatten)되어 있으면 책 prim 에 참조가 없다** (2026-09-20, 담당자
            # Collected 레벨). 그때는 이름이 곧 원본 USD 인 것으로 보고 직접 읽는다.
            if name.endswith((".usd", ".usda", ".usdc")):
                src = os.path.expanduser(name)
                if not os.path.exists(src):
                    say(f"책 원본 파일 없음, 건너뜀: {src}")
                    continue
                sources.append((None, src, np.array([1.0, 0.0, 0.0, 0.0])))
                continue
            path = name if name.startswith("/") else "/World/books/" + name
            prim = st.GetPrimAtPath(path)
            if not prim.IsValid():
                say(f"책 원본 없음, 건너뜀: {path}")
                continue
            ref = None
            for spec in prim.GetPrimStack():
                for r in (list(spec.referenceList.GetAddedOrExplicitItems())
                          + list(spec.payloadList.GetAddedOrExplicitItems())):
                    ref = r.assetPath
            if ref is None:
                say(f"책 원본에 참조 없음, 건너뜀: {path}")
                continue
            ref = os.path.normpath(os.path.join(os.path.dirname(st.GetRootLayer().realPath), ref))
            sources.append((path, ref, SingleXFormPrim(path).get_world_pose()[1]))
        # 레벨 트레이를 쓸 때는 책을 복제하지 않으므로 원본이 없어도 된다
        _lt = st.GetPrimAtPath(KIOSK_TRAY)
        if not sources and not (USE_LEVEL_TRAY and _lt and _lt.IsValid()):
            raise RuntimeError("쓸 수 있는 책 원본이 없다")
        shelf = self.aabb(SHELF)
        self.shelf_aabb_world = np.asarray(shelf, float).copy()
        self.shelf_prim = SHELF          # 현재 서가. 명령의 shelf_id 로 바뀔 수 있다 (set_shelf)
        # **레벨 책을 그대로 쓸 때는 끄지 않는다** — 끄면 쓸 책이 사라진다
        _keep_level = USE_LEVEL_TRAY and _lt and _lt.IsValid()
        for p in (([] if _keep_level else [s[0] for s in sources if s[0] is not None])
                  + ([str(c.GetPath()) for c in st.GetPrimAtPath("/World/fixtures").GetChildren()]
                     if st.GetPrimAtPath("/World/fixtures").IsValid() else [])):
            st.GetPrimAtPath(p).SetActive(False)

        # **출발 방향 스위치** — `SIM_ROBOT_YAW=<도>`: 로봇을 **팔 베이스를 축으로** 제자리 회전시킨다.
        # 이 레벨(level_franka0)은 로봇이 yaw 0° 로 놓여 있어 데크가 반납기 출구와 어긋난다. 반납기에서
        # 트레이를 -x 로 밀어 얹으려면 데크가 반납기 앞에 와야 하고, 그러려면 yaw 90° 로 서야 한다
        # (.4 PC 9/22 결론: "AMR 은 z 축 90° 로 시작해야 트레이가 데크에 실린다"). 트레이 배치·이송 목표는
        # 이 뒤에서 계산되므로 돌린 자세가 그대로 기준이 된다. 기본은 비움 = 레벨 그대로.
        _yaw_env = os.environ.get("SIM_ROBOT_YAW", "").strip()
        if _yaw_env:
            try:
                _rad = math.radians(float(_yaw_env))
                _rp, _rq = SingleXFormPrim(R).get_world_pose()
                _rp, _rq = np.asarray(_rp, float), np.asarray(_rq, float)
                # 축: 기본은 **차체(루트) 제자리 회전** — 레벨이 정한 카트 자리를 지킨다. 팔 베이스를 축으로
                # 돌리면 차체가 0.6 m 옮겨져 반납기(x 5.44~)와 겹쳐 시작하자마자 튕겼다 (2026-09-23 실측).
                # SIM_ROBOT_YAW_PIVOT=arm 이면 팔 베이스를 축으로 (팔 베이스 자리를 지킬 때).
                if os.environ.get("SIM_ROBOT_YAW_PIVOT", "root").strip() == "arm":
                    _pv, _ = SingleXFormPrim(BASE_LINK).get_world_pose()
                    _pv = np.asarray(_pv, float)
                else:
                    _pv = _rp.copy()
                _c, _s = math.cos(_rad), math.sin(_rad)
                _d = _rp - _pv
                _np = _pv + np.array([_c * _d[0] - _s * _d[1], _s * _d[0] + _c * _d[1], _d[2]])
                _cw, _sz = math.cos(_rad / 2.0), math.sin(_rad / 2.0)
                _w, _x, _y, _z = _rq                                        # Isaac: (w, x, y, z)
                _nq = np.array([_cw * _w - _sz * _z, _cw * _x - _sz * _y,
                                _cw * _y + _sz * _x, _cw * _z + _sz * _w], float)
                SingleXFormPrim(R).set_world_pose(_np, _nq)
                _pv2, _ = SingleXFormPrim(BASE_LINK).get_world_pose()
                say(f"출발 방향 회전: {float(_yaw_env):+.1f}° (축 {os.environ.get('SIM_ROBOT_YAW_PIVOT', 'root')} {np.round(_pv[:2], 3).tolist()}, "
                    f"회전 뒤 팔 베이스 {np.round(np.asarray(_pv2, float)[:2], 3).tolist()}) [SIM_ROBOT_YAW]")
            except (TypeError, ValueError) as _exc:
                say(f"SIM_ROBOT_YAW 형식 오류 '{_yaw_env}' ({_exc}) — 돌리지 않는다")

        # 트레이
        #
        # 예전에는 월드 좌표(--tray-center)와 항등 자세로 놓았다. 로봇이 다른 자리·다른
        # 방향으로 서는 순간 트레이가 엉뚱한 데 생긴다 (2026-09-20: 37 cm 아래 + 방향 90° 틀어짐).
        # 좌표 계약이 `arm_base_link` 기준이므로 **트레이도 팔 기준으로 놓는다.**
        _bl = SingleXFormPrim(BASE_LINK)
        _bp, _bq = _bl.get_world_pose()
        _bp = np.asarray(_bp, float)
        _BR = R_from_quat(np.asarray(_bq, float))
        # 팔 기준 트레이 중앙 — book_profiles.yaml 의 칸 좌표와 같은 값이어야 한다
        _tray_rel = np.array([float(tray_center[0]), float(tray_center[1]), 0.0])
        _R0 = R_from_quat(np.asarray(_bq, float))
        _yaw0 = math.atan2(float(_R0[1, 0]), float(_R0[0, 0]))
        _Ry0 = np.array([[math.cos(_yaw0), -math.sin(_yaw0), 0.0],
                         [math.sin(_yaw0), math.cos(_yaw0), 0.0],
                         [0.0, 0.0, 1.0]])
        _tray_w = _bp + _Ry0 @ _tray_rel
        _level_tray = st.GetPrimAtPath(KIOSK_TRAY)
        self.use_level_tray = bool(USE_LEVEL_TRAY and _level_tray and _level_tray.IsValid())
        # **조용히 대체하지 않는다.** 예전에는 레벨 트레이를 못 찾으면 말없이 옛 에셋을
        # 스폰했다. 화면에는 엉뚱한 트레이(스케일·콜리전 다름)가 나오는데 로그는 멀쩡해서
        # 며칠을 엉뚱한 곳에서 원인을 찾았다 (2026-09-22). 이제는 여기서 죽는다.
        if USE_LEVEL_TRAY and not self.use_level_tray:
            _sib = [c.GetName() for c in st.GetPrimAtPath(
                KIOSK_TRAY.rsplit("/", 1)[0]).GetChildren()] if st.GetPrimAtPath(
                    KIOSK_TRAY.rsplit("/", 1)[0]).IsValid() else []
            raise RuntimeError(
                f"레벨에 트레이가 없다: {KIOSK_TRAY}\n"
                f"  같은 폴더에 있는 것: {_sib}\n"
                f"  SIM_KIOSK_TRAY 로 올바른 경로를 주거나, 정말 스폰하려면 "
                f"SIM_USE_LEVEL_TRAY=0 을 명시할 것")
        if self.use_level_tray:
            # **레벨 트레이를 그대로 쓴다.** 자세도 건드리지 않는다 (레벨이 정답이다)
            self.tray = KIOSK_TRAY
            _tb = self.aabb(self.tray)
            _tray_home = _tray_w.copy()      # 팔 기준 칸이 가리키는 **최종** 자리
            _tray_w = (_tb[:3] + _tb[3:]) / 2.0
            say(f"레벨 트레이를 그대로 쓴다: {KIOSK_TRAY} "
                f"중앙 {np.round(_tray_w[:2], 3).tolist()} 바닥면 z {_tb[2]:.4f} "
                f"[SIM_USE_LEVEL_TRAY]")
        else:
            self.tray = "/World/bs_tray"
            add_reference_to_stage(tray_usd, self.tray)
        # **트레이는 수평이어야 한다.** 팔 베이스 자세를 그대로 쓰면 거기 섞인 뒤집힘·기울기가
        # 트레이에 그대로 들어가 책이 미끄러진다 (2026-09-20: 책 6권이 한쪽에 뭉쳤다).
        # 방향(yaw)만 따르고 나머지는 버린다.
        _yaw = _yaw0
        _tray_q = np.array([math.cos(_yaw / 2), 0.0, 0.0, math.sin(_yaw / 2)])
        if not self.use_level_tray:
            SingleXFormPrim(self.tray).set_world_pose(
                np.array([_tray_w[0], _tray_w[1], DECK_Z]), _tray_q)
        # **바닥면을 데크에 맞춘다.** DECK_Z 에 놓는 것은 트레이의 *원점* 인데,
        # 원점이 바닥면에 있다는 보장이 없다 (에셋·스케일이 바뀌면 달라진다).
        # 실제로 v3 트레이를 1.25배로 키우자 트레이가 바닥에 내려앉았다 (2026-09-22).
        # AABB 를 재서 바닥면이 데크 윗면에 오도록 한 번 더 올린다.
        _tb = self.aabb(self.tray)
        _lift = 0.0 if self.use_level_tray else DECK_Z - float(_tb[2])
        if abs(_lift) > 1e-4:
            _p0, _q0 = SingleXFormPrim(self.tray).get_world_pose()
            SingleXFormPrim(self.tray).set_world_pose(
                np.array([_p0[0], _p0[1], float(_p0[2]) + _lift]), _q0)
            say(f"트레이 바닥면을 데크에 맞춘다: 원점 z {float(_p0[2]):.4f} "
                f"→ {float(_p0[2]) + _lift:.4f} (바닥면 {float(_tb[2]):.4f} → {DECK_Z:.4f})")
        say(f"트레이 자세: yaw {math.degrees(_yaw):.1f}° (수평 유지)")
        # **월드 값을 따로 들고 있는다** — 아래 계산들은 월드 기준이라 팔 기준 값을 그대로
        # 쓰면 x,y 는 팔 기준·z 는 월드인 잡종 좌표가 된다 (2026-09-20 실제로 IK 실패).
        self.tray_center_w = _tray_w.copy()
        self.tray_yaw = float(_yaw0)      # 파지·삽입 자세의 기준 방향
        # **팔 기준 값은 주행해도 안 변한다.** refresh_base() 가 이것으로 월드 값을 다시 만든다
        self._tray_rel = _tray_rel.copy()
        say(f"트레이 배치: 팔 기준 {np.round(_tray_rel[:2], 4).tolist()} "
            f"→ 월드 {np.round(_tray_w[:2], 3).tolist()}, 면 z {DECK_Z:.3f}")
        # **반납기에서 x 축으로만 밀어 얹는다.** 레벨의 트레이는 이미 데크 높이·최종 y 에
        # 놓여 있으므로(반납기 출구가 데크 높이라는 설정), 남은 것은 x 방향 직선 이동뿐이다.
        # 컨베이어처럼 일정한 속도로 간다.
        #
        # 레벨 트레이는 **모양만** 쓰고, 실제로 움직이는 것은 우리 트레이(bs_tray)다.
        # 둘이 같은 자리에 있으면 하나로 보인다 — 레벨 쪽은 숨긴다.
        self._deliver = None
        if TRAY_DELIVERY:
            _k = st.GetPrimAtPath(KIOSK_TRAY)
            if _k and _k.IsValid():
                _now_p, _now_q = SingleXFormPrim(self.tray).get_world_pose()
                # **레벨에서 잰 좌표를 그대로 쓴다.** 팔 기준으로 계산해 맞추려다
                # 여러 번 어긋났다 — 레벨 배치가 정답이다.
                # 상수로 박아 두면 레벨이 바뀔 때 조용히 썩는다 (3.6 cm → 410).
                _frm, _to, _src = deliver_from_to(_now_p)
                self._deliver = {"from": _frm, "to": _to,
                                 "q": np.asarray(_now_q, float), "t": 0, "n": 1}
                if not self.use_level_tray:
                    _k.SetActive(False)   # 복제 트레이를 쓸 때만 레벨 쪽을 숨긴다
                say(f"트레이 이송 {np.round(_frm, 6).tolist()} → "
                    f"{np.round(_to, 6).tolist()} "
                    f"({float(np.linalg.norm(_to - _frm)):.3f} m, "
                    f"{TRAY_DELIVERY_S:.1f}초, 출발 자리 출처: {_src})")
                _drift = drift_mm(_frm)
                if _drift > 5.0:
                    say(f"  주의: 출발 z {_frm[2]:.6f} 가 기록해 둔 레벨 값 "
                        f"{TRAY_FROM_FALLBACK[2]:.6f} 에서 {_drift:.1f} mm 다르다 — "
                        f"레벨이 바뀌었으면 정상, 아니면 확인할 것")
            else:
                say(f"반납기 트레이가 없다 ({KIOSK_TRAY}) — 트레이를 처음부터 로봇 위에 둔다")
        # **트레이를 키네마틱 강체로** 만든다. 동적 강체 + 고정 조인트로 묶어 봤더니
        # 트레이가 흔들려 책이 칸에서 36cm 벗어났다 (2026-09-21 실측).
        # 키네마틱은 물리가 밀지 못하고 우리가 매 스텝 자세를 준다 — 북엔드와 같은 방식이다.
        # 기준 상대 자세는 재생·정착이 끝난 뒤에 잡는다 (아래).
        self._tray_joint = None
        self._tray_anchor = None
        self._tray_follow = os.environ.get("SIM_TRAY_FOLLOW", "1") != "0"
        # **강체를 붙이지 않는다.** 키네마틱 강체로 만들어 봤더니 그리퍼 축 계산이
        # `접근축 를 못 구한다 (길이 5.07e-07)` 로 죽었다 — ee_frame 과 hand_link 가
        # 같은 자리로 나온다 (2026-09-21 실측, SIM_TRAY_FOLLOW=0 이면 정상).
        # 트레이는 정적 콜라이더 그대로 두고 자세만 매 스텝 옮긴다. 그 위의 책은
        # 마찰에 기대지 않고 follow_tray() 가 같은 변위로 직접 옮긴다.

        pitch = nslots = floor_top = None
        # **진단용 스위치.** SIM_TRAY_COLLIDER=0 이면 트레이 콜라이더를 안 붙인다.
        # "팔이 트레이에 막히는가" 를 5분 만에 가르기 위한 것 — 책은 트레이를 통과해
        # 받침판 위로 떨어지므로 파지는 못 하지만, **접근 자세에 도달하는지**는 볼 수 있다.
        # (2026-09-20: 서가·게인·질량·이웃책·받침판을 전부 배제하고 트레이만 남았다)
        _tray_coll = os.environ.get("SIM_TRAY_COLLIDER", "1") != "0"
        if not _tray_coll:
            say("**트레이 콜라이더 없음** (SIM_TRAY_COLLIDER=0) — 진단 전용, 시연에 쓰지 말 것")
        for p in Usd.PrimRange(st.GetPrimAtPath(self.tray)):
            # **레벨 트레이의 콜리전은 건드리지 않는다.** 레벨이 정답이다.
            #
            # 여기서 `none`(삼각망)을 덮어쓰고 있었다. 스폰하던 `bs_tray` 는 정적
            # 콜라이더라 `none` 이 맞았지만, 레벨의 `tray_v3` 는 **동적 강체**라
            # `none` 은 불법이고 PhysX 가 **볼록 껍질로 조용히 대체**한다:
            #   "triangle mesh collision (approximation None/MeshSimplification)
            #    cannot be a part of a dynamic body, falling back to convexHull"
            # 그러면 칸이 메워진 덩어리가 되어 책이 칸에 안 들어가고 위에 얹힌다
            # (2026-09-22 실측: 책 바닥이 칸 바닥 0.320 이 아니라 0.415 에서 멈췄다).
            # 파일에는 convexDecomposition 이 적혀 있는데 화면은 달라서 며칠을 헤맸다.
            if p.IsA(UsdGeom.Mesh) and _tray_coll and not self.use_level_tray:
                UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
            for a in p.GetAttributes():
                n = a.GetName()
                if n.endswith("tray_pitch"): pitch = float(a.Get())
                if n.endswith("tray_slots"): nslots = int(a.Get())
                if n.endswith("floor_top_z"): floor_top = float(a.Get())
        # 레벨 트레이에는 이 사용자 속성이 없을 수 있다 — 없으면 **실측값**으로 채운다.
        # (칸 좌표는 book_profiles.yaml 이 정하므로 여기 값은 스폰용 보조다)
        if pitch is None:
            pitch = 0.075
        if nslots is None:
            nslots = 6
        if floor_top is None:
            _tbz = self.aabb(self.tray)
            floor_top = float(_tbz[2]) - DECK_Z + 0.02
        # 칸은 **팔 기준 x 축**을 따라 늘어선다 (월드 x 가 아니다)
        slot_rel = [np.array([tray_center[0] + (i + 0.5 - nslots / 2) * pitch,
                              tray_center[1], 0.0]) for i in range(nslots)]
        _Ryaw = np.array([[math.cos(_yaw), -math.sin(_yaw), 0.0],
                          [math.sin(_yaw), math.cos(_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        slot_w = [_bp + _Ryaw @ r for r in slot_rel]
        slot_x = [float(w[0]) for w in slot_w]      # 호환용 (월드 x)
        # **칸 좌표를 찍어 둔다.** 책은 원점 위 5 m 에 잠깐 띄웠다가 여기로 옮겨진다.
        # 그 이동이 실패하면 책이 맵 중앙 공중에 남는데, 화면만 봐서는 원인을 못 가른다
        say(f"[진단] 트레이 중앙(월드) {np.round(_tray_w[:2], 3).tolist()}  "
            f"칸 {nslots}개  간격 {pitch:.3f}")
        say(f"[진단] 칸 월드 x {[round(float(v[0]), 3) for v in slot_w]}")
        say(f"[진단] 칸 월드 y {[round(float(v[1]), 3) for v in slot_w]}")

        # 책
        self.books = []
        self.dims = {}          # 책마다 치수가 다르다 (두께 T, 세운 높이 L, 깊이 W)
        self.grasp_local = {}   # 책 좌표계의 (중심, 위 방향, 반높이)
        self.upright_q = {}     # 트레이에서 세운 자세 (월드, 채택 시점)
        self.upright_rel = {}   # 같은 자세를 **트레이 기준**으로 (로봇이 돌아도 유효하다)
        self.slot_pose = {}     # 트레이 칸에 놓았을 때의 (위치, 자세) — 데이터 촬영에서 재배치에 쓴다
        if self.use_level_tray:
            # **레벨 책을 그대로 채택한다.** 복제하지도, 세우지도, 옮기지도 않는다.
            # 레벨에서 이미 칸에 꽂혀 있으므로 그 자세가 정답이다 — 우리가 다시 세우면
            # 원본과 달라진다 (자세·치수·콜리전).
            _lb = [c for c in st.GetPrimAtPath(KIOSK_TRAY).GetParent().GetChildren()
                   if c.GetTypeName() == "Xform" and str(c.GetPath()) != KIOSK_TRAY]
            _lb.sort(key=lambda c: self.aabb(str(c.GetPath()))[1])   # 칸 순서 (월드 y)
            for c in _lb[:n_books]:
                path = str(c.GetPath())
                b = self.aabb(path)
                ctr = (b[:3] + b[3:]) / 2.0
                pos_w, q_w = SingleXFormPrim(path).get_world_pose()
                pos_w = np.asarray(pos_w, float); q_w = np.asarray(q_w, float)
                Rw = R_from_quat(q_w)
                self.books.append(path)
                self.upright_q[path] = q_w.copy()
                # **세운 자세를 트레이 기준으로도 남긴다.** 월드 기준만 들고 있으면
                # 로봇이 돌 때 기준이 같이 돌지 않아, 파지 직전 보정이 책을 89.3°
                # 억지로 돌려 손에서 빠뜨린다 (2026-09-22 실측: yaw 90°→0° 주행 뒤 `406`).
                _Rt0 = R_from_quat(np.asarray(
                    SingleXFormPrim(KIOSK_TRAY).get_world_pose()[1], float))
                self.upright_rel[path] = _Rt0.T @ Rw
                self.slot_pose[path] = (pos_w.copy(), q_w.copy())
                self.grasp_local[path] = (Rw.T @ (ctr - pos_w),
                                          Rw.T @ np.array([0.0, 0.0, 1.0]),
                                          (b[5] - b[2]) / 2)
                d = (b[3] - b[0], b[4] - b[1], b[5] - b[2])
                if min(d) < 0.005:
                    raise RuntimeError(f"레벨 책 치수가 비었다 {path} {d}")
                self.dims[path] = d
                # **책 콜리전 근사를 바꾼다** (`SIM_BOOK_COLL`, 기본 그대로 둠).
                #
                # 레벨 책은 `convexHull` 이다. 그런데 바로 위 스폰 경로(561~564)에는
                # 이런 주석이 이미 붙어 있다:
                #
                #   "표지가 둥근 책은 convexHull 이면 흔들려 넘어진다. 책은 상자에
                #    가까우므로 boundingCube 로 두면 트레이에 그대로 서 있는다."
                #
                # 2026-09-24: 꽂힌 책이 무엇을 해도 2.1~3.5° 로 기운다. 도착 yaw ·
                # 파지 정렬 · 파지 시점 기울기 · 손 안 자세 · 떠오름 16 mm · 콜라이더
                # 면 위치를 전부 지웠는데 안 움직인다. **끌개가 있다**는 뜻이고,
                # 둥근 표지의 볼록 껍질은 바로 그런 끌개다 — 밑면이 평평하지 않으면
                # 어디서 놓든 같은 각도로 눕는다. 위 주석이 이미 그렇게 말하고 있다.
                #
                # `boundingCube` 는 동적 강체에도 합법이다 (상자라서). `none` 과 달리
                # PhysX 가 조용히 갈아치우지 않는다.
                _bc = os.environ.get("SIM_BOOK_COLL", "").strip()
                if _bc:
                    _n = 0
                    for _m in Usd.PrimRange(c):
                        if _m.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.MeshCollisionAPI.Apply(_m).CreateApproximationAttr().Set(_bc)
                            _n += 1
                    say(f"[책콜리전] {path.rsplit('/', 1)[-1][:34]} — {_n}개를 '{_bc}' 로")
                say(f"[레벨책] {path.rsplit('/', 1)[-1][:40]}  중심 {np.round(ctr, 3).tolist()}  "
                    f"치수 {[round(float(v), 3) for v in d]}")
            if not self.books:
                raise RuntimeError(f"레벨 트레이에 책이 없다 ({KIOSK_TRAY} 의 형제)")
            say(f"레벨 책 {len(self.books)}권을 그대로 쓴다")

        for i in range(0 if self.use_level_tray else min(n_books, nslots)):
            path = f"/World/bs_books/book_{i}"
            src_path, book_ref, src_q = sources[i % len(sources)]
            add_reference_to_stage(book_ref, path)
            prim = st.GetPrimAtPath(path)
            UsdPhysics.RigidBodyAPI.Apply(prim); UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(0.5)
            for p in Usd.PrimRange(prim):
                if p.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(p)
                    # 표지가 둥근 책은 convexHull 이면 흔들려 넘어진다 (종류를 섞으면 특히).
                    # 책은 상자에 가까우므로 boundingCube 로 두면 트레이에 그대로 서 있는다.
                    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("boundingCube")
            xf = SingleXFormPrim(path); xf.set_world_pose(np.array([0.0, 0.0, 5.0 + i]), src_q)
            # 에셋마다 원본이 놓인 방향이 달라서(눕힌 것도 있다) 트레이 기준으로 세운다:
            # x = 두께(가장 작은 변), y = 세운 높이(가장 큰 변), z = 깊이(중간)
            q_up = self.upright_quat(path, np.asarray(src_q, float), np.array([0.0, 0.0, 5.0 + i]))
            xf.set_world_pose(np.array([0.0, 0.0, 5.0 + i]), q_up)
            b = self.aabb(path); c = (b[:3] + b[3:]) / 2
            target = np.array([slot_w[i][0], slot_w[i][1],
                               DECK_Z + floor_top + (b[5] - b[2]) / 2 + 0.002])
            pos, q = xf.get_world_pose(); xf.set_world_pose(np.array(pos) + (target - c), q)
            self.books.append(path)
            self.slot_pose[path] = (target.copy(), None)   # 트레이 칸 자세 (자세는 아래에서 채운다)
            _after = self.aabb(path)
            _ac = (_after[:3] + _after[3:]) / 2.0
            _err = float(np.linalg.norm(_ac - target))
            say(f"[진단] book_{i} 목표 {np.round(target, 3).tolist()} "
                f"→ 실제 {np.round(_ac, 3).tolist()}  차이 {_err*1000:.1f} mm"
                + ("  **옮겨지지 않았다**" if _err > 0.05 else ""))
            # 파지점을 책 자신의 좌표로 저장한다. 트레이에서 몇 도만 기울어도
            # AABB 중심은 실제 책 중심과 어긋나 손가락이 책을 밀어낸다 (실측 M406).
            b = self.aabb(path); c = (b[:3] + b[3:]) / 2
            pos_w, q_w = xf.get_world_pose()
            Rw = R_from_quat(np.asarray(q_w, float))
            self.upright_q[path] = np.asarray(q_w, float).copy()
            self.slot_pose[path] = (np.asarray(pos_w, float).copy(), np.asarray(q_w, float).copy())
            self.grasp_local[path] = (Rw.T @ (c - np.asarray(pos_w, float)),
                                      Rw.T @ np.array([0.0, 0.0, 1.0]),
                                      (b[5] - b[2]) / 2)
            d = (b[3] - b[0], b[4] - b[1], b[5] - b[2])
            if min(d) < 0.005:
                raise RuntimeError(f"책 {src_path} 의 치수가 비었다 {d} — 참조 파일 확인: {book_ref}")
            self.dims[path] = d

        # 트레이 칸막이는 두지 않는다: 칸 간격 0.075 m 안에서는 손가락이 지나갈 폭이 남지 않아
        # 칸막이를 세우면 파지 경로를 막는다 (실측: down 단계 시간 초과). 대신 책 충돌을 상자로 근사해 세워 둔다.
        self.world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
        self.robot = SingleArticulation(prim_path=BOT.articulation_root, name="rf")
        link0_x = float(SingleXFormPrim(BASE_LINK).get_world_pose()[0][0])
        self.tray_floor_z = DECK_Z + floor_top
        self.slot_x = slot_x
        self.tray_y = float(self.tray_center_w[1])      # 월드 y
        self.T, self.L, self.W = self.dims[self.books[0]]   # 기본값 (홈 자세·여러 종류일 때의 대표값)
        self.T_max = max(d[0] for d in self.dims.values())
        self.W_max = max(d[2] for d in self.dims.values())
        self.shelf_front_y = float(shelf[1])

        # **북엔드를 만들지 않는다** (2026-09-22). 예전에는 꽂은 책이 넘어지지 않게
        # `/World/bs_bookends` 에 큐브 한 쌍씩을 스폰했는데, 레벨에 없는 물체가
        # 장면에 끼어드는 것이라 협업에 방해가 된다. 서가는 레벨이 정답이다.
        floor_z = self.shelf_floor_z = SHELF_ROW_Z
        # **선반 판 높이를 레벨에서 잰다** (`SIM_SHELF_ROW_MEASURE=1`, 기본 꺼짐).
        #
        # 기본값이 `0.355 * 1.4` 다 — 어디서 온 곱셈인지 코드에 없고, 레벨이 바뀌면
        # 조용히 썩는다. 오늘 밤에만 같은 병이 여덟 번 나왔다(트레이 출발점, 꽂을 x,
        # 스윕 기준점, 406 기준점, move_rig, 파지 벌림, 파지 정렬 기준자세, 그리고 이것).
        #
        # 재는 법: 서가 메시의 점들 중 지금 값 근처의 **수평면**을 찾는다. 이 메시는
        # 92점짜리 저폴리라 판 윗면이 같은 z 에 8점씩 모여 있다 (2026-09-24 실측:
        # z 0.4425 여덟 점 = 판 아랫면, z 0.4976 여덟 점 = 판 윗면).
        #
        # 주의: 이것은 **시각** 형상이다. 물리가 쓰는 콜라이더가 다르면 소용없다 —
        # `SIM_SHELF_COLL=none` 과 **같이** 써야 둘이 맞는다. 따로 쓰면 오늘의 18 mm 가
        # 그대로 남는다.
        if os.environ.get("SIM_SHELF_ROW_MEASURE", "0") != "0":
            _m = self.measure_shelf_board_z(SHELF_ROW_Z)
            if _m is None:
                say(f"[선반] 판 윗면을 못 쟀다 — 설정값 {SHELF_ROW_Z:.4f} 을 그대로 쓴다")
            else:
                say(f"[선반] 판 윗면 실측 {_m:.4f} (설정 {SHELF_ROW_Z:.4f}, "
                    f"차이 {(_m - SHELF_ROW_Z)*1000:+.1f} mm) [SIM_SHELF_ROW_MEASURE]")
                floor_z = self.shelf_floor_z = float(_m)
        self.bookends = {}      # 비워 둔다 — fit_bookends() 는 아무것도 하지 않는다

        # **서가 콜라이더 근사를 바꾼다** (`SIM_SHELF_COLL`, 기본 그대로 둠).
        #
        # 2026-09-24 실측: 놓은 책이 **18 mm 떠오른 자리(z 0.5155)에 앉는다.** 네 판이
        # 모두 같은 자리다. 그런데 선반 판의 **시각** 윗면은 0.4976 이다 (메시 점에서
        # 직접 읽음, 8점). 계획은 SHELF_ROW_Z(0.497)를 겨누므로, 책은 쿠킹된 콜라이더
        # 속 18 mm 아래로 들어갔다가 놓는 순간 밀려 나온다. 그 튕김 뒤의 자유 낙하가
        # 매번 다르게 기울어 꽂힌 기울기가 0.5~3.1° 로 흔들린다.
        #
        # 서가 전체에 콜라이더가 **하나**뿐이고 근사가 `convexDecomposition` 이다.
        # 2.5 m 짜리 통짜 메시를 VHACD 로 복셀 근사하면 얇은 선반 판에는 복셀 한 겹이
        # 그대로 두께로 붙는다 — 그 한 겹이 18 mm 다.
        #
        # 서가는 **정적**이므로 `none`(삼각망 그대로)이 합법이고, 그러면 판 윗면이
        # 시각 형상과 같아진다. 동적 강체에 `none` 을 주면 PhysX 가 조용히 볼록 껍질로
        # 갈아치운다 — 위 트레이 주석의 그 함정이라 **정적인 것만** 바꾼다.
        _shelf_coll = os.environ.get("SIM_SHELF_COLL", "").strip()
        if _shelf_coll:
            _root = st.GetPrimAtPath(os.environ.get(
                "SIM_SHELF_COLL_ROOT", SHELF.rsplit("/", 1)[0]))
            _n, _skip = 0, 0
            if _root and _root.IsValid():
                for _p in Usd.PrimRange(_root):
                    if not _p.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    if _p.HasAPI(UsdPhysics.RigidBodyAPI):
                        _skip += 1        # 동적이면 건드리지 않는다 (조용히 갈아치워진다)
                        continue
                    UsdPhysics.MeshCollisionAPI.Apply(_p).CreateApproximationAttr().Set(_shelf_coll)
                    _n += 1
                    say(f"[서가콜리전] {str(_p.GetPath()).rsplit('/', 1)[-1]} → {_shelf_coll}")
            say(f"[서가콜리전] {_n}개를 '{_shelf_coll}' 로 바꿨다"
                + (f" (동적이라 건너뛴 것 {_skip}개)" if _skip else "")
                + " [SIM_SHELF_COLL]")

        # 팔 구동 게인 — **옛** 에셋(강성 40~1135)은 위치 지령을 못 따라가서 올려야 했다.
        # 지금 받은 에셋은 기본값이 2.3e3~6.5e4 라 덮어쓸 이유가 줄었고, 덮어쓰면 USD 의 도(°)
        # 단위가 라디안으로 환산되며 57배가 되어 감쇠가 5.7e6 까지 올라간다 (2026-09-20 실측).
        # ARM_DRIVE_STIFFNESS=0 으로 끄고 에셋 기본값을 쓸 수 있게 한다.
        # 목표값 0 맞추기는 **게인과 무관하게 항상** 한다 — joint_3·joint_5 의 목표(90°)를
        # 안 맞추면 재생 순간 팔이 85° 튀며 로봇이 흔들린다 (2026-09-18 실측).
        _stiff = float(os.environ.get("ARM_DRIVE_STIFFNESS", BOT.drive_stiffness))
        _damp = float(os.environ.get("ARM_DRIVE_DAMPING", BOT.drive_damping))
        _maxf = float(os.environ.get("ARM_DRIVE_MAXFORCE", "0"))
        _tuned = []
        _preserve_targets = os.environ.get("SIM_PRESERVE_DRIVE_TARGETS", "0") == "1"
        for _p in Usd.PrimRange(st.GetPrimAtPath(R)):
            if _p.GetName() not in set(ARM_JOINTS):
                continue
            _d = UsdPhysics.DriveAPI.Get(_p, "angular") or UsdPhysics.DriveAPI.Apply(_p, "angular")
            if _stiff > 0:
                _d.CreateTypeAttr().Set("force")
                _d.CreateStiffnessAttr().Set(_stiff)
                _d.CreateDampingAttr().Set(_damp)
            if _maxf > 0:
                # 구동력 상한. 지금까지 건드리지 않아 URDF effort(163 N·m)가 그대로 상한이었다.
                # 팔을 멀리 뻗는 자세에서 팔꿈치(joint_3)가 목표보다 0.267 rad 처진 채
                # 도달 판정을 못 받아 `404` 가 났다 (2026-09-21 실측, 주변에 닿는 물체는 없었다).
                _d.CreateMaxForceAttr().Set(_maxf)
            _t = _d.GetTargetPositionAttr().Get()
            if not _preserve_targets and _t not in (None, 0.0):
                _d.CreateTargetPositionAttr().Set(0.0)
                _tuned.append(f"{_p.GetName()} 목표 {_t}°→0°")
        say(("팔 구동 게인 설정: 강성 %.0e 감쇠 %.0e 힘상한 %s" % (_stiff, _damp, _maxf or "에셋값")) if _stiff > 0
            else "팔 구동 게인: **에셋 기본값 사용** (ARM_DRIVE_STIFFNESS=0)")
        if _tuned:
            say(f"  구동 목표 보정: {_tuned}")
        elif _preserve_targets:
            say("  구동 목표 보존: 레벨에 저장된 카메라 관측 자세 유지")

        if before_reset is not None:
            before_reset(st)      # ROS 그래프 설정 보완 등 — 재생(초기화) 전에 해야 반영된다

        # **출발 위치를 옮기는 시험용 스위치** — `SIM_ROBOT_OFFSET="dx,dy"` (m, 월드).
        # 트레이 앵커·책 배치·`home_arm` 은 **모두 이 뒤에** 잡히므로, 옮긴 자리가
        # 그대로 새 출발점이 된다. 즉 '출발점 오차' 가 아니라 **'출발점 변화'** 를 재는 것이다.
        # reset 전에 옮겨야 그 자세가 기본 상태로 저장된다.
        _off = os.environ.get("SIM_ROBOT_OFFSET", "").strip()
        if _off:
            try:
                _dx, _dy = (float(v) for v in _off.replace(" ", "").split(","))
                _rp, _rq = SingleXFormPrim(R).get_world_pose()
                _rp = np.asarray(_rp, float).copy()
                _rp[0] += _dx
                _rp[1] += _dy
                SingleXFormPrim(R).set_world_pose(_rp, _rq)
                say(f"출발 위치 이동: ({_dx:+.3f}, {_dy:+.3f}) m → "
                    f"({_rp[0]:+.4f}, {_rp[1]:+.4f}) [SIM_ROBOT_OFFSET]")
            except (TypeError, ValueError) as _exc:
                say(f"SIM_ROBOT_OFFSET 형식 오류 '{_off}' ({_exc}) — 옮기지 않는다")

        # **부유 베이스의 뿌리 링크(`<루트>/world`)를 루트 XForm 에 맞춘다** — `SIM_ALIGN_WORLD_LINK=1`.
        # 이 레벨(데스크탑 사본)은 뿌리 링크가 루트와 따로 놀아 (4.763, -3.469, yaw −15°) 에 있고
        # 루트는 (4.986, -5.607, yaw +90°) 다 (2026-09-22 야간 실측, 스텝 60 부터 끝까지 그대로).
        # 물리는 뿌리 링크 기준으로 풀리므로 팔 베이스가 2 m 떨어진 곳에 서고, IK 는 전부 401 이다.
        _alw = os.environ.get("SIM_ALIGN_WORLD_LINK", "0") != "0"
        if _alw and self.stage.GetPrimAtPath(R + "/world").IsValid():
            _rp, _rq = SingleXFormPrim(R).get_world_pose()
            _wl = SingleXFormPrim(R + "/world")
            _wp0, _wq0 = _wl.get_world_pose()
            _wl.set_local_pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
            _wp1, _wq1 = _wl.get_world_pose()
            say(f"[뿌리링크] {R}/world {np.round(np.asarray(_wp0, float), 3).tolist()} q "
                f"{np.round(np.asarray(_wq0, float), 3).tolist()} → {np.round(np.asarray(_wp1, float), 3).tolist()} q "
                f"{np.round(np.asarray(_wq1, float), 3).tolist()} (루트 {np.round(np.asarray(_rp, float), 3).tolist()}) "
                f"[SIM_ALIGN_WORLD_LINK]")
        self.world.reset(); self.robot.initialize()
        r = self.robot
        self.idx_arm = [r.get_dof_index(j) for j in ARM_JOINTS]
        self.idx_fing = [r.get_dof_index(j) for j in FINGERS]
        self.base_idx = [r.get_dof_index(j) for j in r.dof_names if j.startswith("dummy_base")]
        self.base_hold = r.get_joint_positions()[self.base_idx]
        for _ in range(120):
            self.world.step(render=False)

        # **책도 트레이에 묶는다.** 트레이만 로봇에 붙이면 주행할 때 책은 얹혀만 있어서
        # 뒤에 남는다 (2026-09-21 실측: 로봇 2m 이동에 책은 199cm 뒤처졌다).
        # 파지 직전에 푼다 — 그때부터는 손이 들고 간다. SIM_BOOK_LOCK=0 이면 안 묶는다.
        self._book_locks = {}
        if os.environ.get("SIM_BOOK_LOCK", "0") != "0" and getattr(self, "_tray_joint", None):
            for i, b in enumerate(self.books):
                self._book_locks[b] = self._lock_book_to_tray(b, i)
            self.say(f"책 {len(self._book_locks)}권을 트레이에 고정 (파지 직전에 푼다)")

        l0p, l0q = SingleXFormPrim(BASE_LINK).get_world_pose()
        self.l0p = np.asarray(l0p, float); self.Rl0 = R_from_quat(np.asarray(l0q, float))
        if BOT.lula[0] == "supported":
            cfg = interface_config_loader.load_supported_lula_kinematics_solver_config(BOT.lula[1])
        else:
            # Isaac 기본 지원 목록에 Doosan 이 없다 — descriptor·URDF 를 직접 준다
            cfg = {"robot_description_path": BOT.lula[1], "urdf_path": BOT.lula[2]}
        self.lula = LulaKinematicsSolver(**cfg)
        # 씨앗을 조금씩 흔들어 다른 해 가지를 찾아본다 (먼 해가 나왔을 때만 쓴다)
        self._ik_nudges = []
        for j in range(BOT.dof):
            for d in (0.3, -0.3, 0.8, -0.8):
                v = np.zeros(BOT.dof); v[j] = d
                self._ik_nudges.append(v)
        # **반대편 가지도 후보에 넣는다.** 같은 손끝 자세에 팔을 통째로 돌린 해가 있는데,
        # 작은 흔들기(±0.8)로는 거기까지 못 간다. 받침판에 막히지 않는 유일한 해가
        # 그쪽일 수 있다 — 실제로 어제까지 파지가 되던 자세가 그 가지였다 (2026-09-21).
        # 어느 쪽을 고를지는 **받침판 관통 검사(_above_deck)가 정한다.**
        for j in (0, 3, 5):          # 어깨 회전(joint_1)과 손목(joint_4, joint_6)
            if j < BOT.dof:
                for d in (math.pi, -math.pi):
                    v = np.zeros(BOT.dof); v[j] = d
                    self._ik_nudges.append(v)
        self.lula.set_robot_base_pose(l0p, l0q)
        self.ik = ArticulationKinematicsSolver(r, self.lula, BOT.ee_frame)
        # **받침판 관통 검사에 쓸 충돌 구.** 기술서(descriptor)에 링크별로 들어 있다.
        # 링크 원점 높이만 보면 부족하다 — 원점은 판 위인데 팔 몸통이 닿는 경우를 놓친다
        # (2026-09-21: link_3 원점은 판 위인데 joint_2 가 -1.43 에서 물리적으로 멈췄다).
        # base_link·link_1 은 **받침판에 붙어 있는 것이 정상**이라 검사에서 뺀다.
        self._deck_spheres = []
        try:
            # 지원 로봇의 BOT.lula[1]은 파일 경로가 아니라 이름("Franka")이다.
            # loader가 돌려준 실제 descriptor 경로를 써야 충돌 구 검사가 켜진다.
            _desc_path = cfg["robot_description_path"]
            with open(_desc_path, encoding="utf-8") as _desc_file:
                _desc = yaml.safe_load(_desc_file)
            for _entry in (_desc.get("collision_spheres") or []):
                for _lname, _sph in _entry.items():
                    if _lname in ("base_link", "link_1"):
                        continue
                    try:
                        self.lula.compute_forward_kinematics(_lname, np.zeros(BOT.dof))
                    except Exception:      # noqa: BLE001 — Lula 가 모르는 프레임은 건너뛴다
                        continue
                    for _s in _sph:
                        self._deck_spheres.append(
                            (_lname, np.asarray(_s["center"], float), float(_s["radius"])))
        except Exception as exc:      # noqa: BLE001
            say(f"[주의] 충돌 구를 못 읽었다 — 받침판 관통 검사를 하지 않는다: {exc}")
        say(f"받침판 관통 검사: 구 {len(self._deck_spheres)}개 "
            f"(판 z {DECK_Z:.3f}, 허용 관통 {DECK_SINK_M*100:.0f}cm)")

        ee_p, ee_R = self.ik.compute_end_effector_pose()
        hand_p = np.array(SingleXFormPrim(HAND_LINK).get_world_pose()[0])
        lf = np.array(SingleXFormPrim(R + "/" + BOT.finger_links[0]).get_world_pose()[0])
        rf = np.array(SingleXFormPrim(R + "/" + BOT.finger_links[1]).get_world_pose()[0])
        # ee_frame 과 hand_link 가 **같은 링크면 차이가 0** 이라 나누면 nan 이 된다. nan 은
        # quat_from_R 의 max(0.0, nan)=0.0 에 먹혀 **쿼터니언 [0,0,0,0]** 이 되고, 예외 하나 없이
        # 경로 전체가 망가진다. 지금은 Lula(URDF)와 USD prim 의 원점이 달라 우연히 0 이 아니지만
        # 로봇이 바뀌면 조용히 터진다. 손가락 쪽 `or 1.0` 도 같은 이유로 위험하다 (2026-09-21 점검).
        def _unit(v, what):
            n = float(np.linalg.norm(v))
            if n < 1e-6:
                raise RuntimeError(
                    f"{what} 를 못 구한다 (길이 {n:.2e}). "
                    f"ee_frame={BOT.ee_frame} hand_link={BOT.hand_link} "
                    f"finger_links={BOT.finger_links} 를 확인할 것")
            return v / n

        _a_raw = ee_R.T @ _unit(ee_p - hand_p, "접근축")
        _c_raw = ee_R.T @ _unit(rf - lf, "물림축")
        # **검증된 동작 그대로 둔다** (단순 반올림). 2026-09-21 에 직교화를 넣어 봤더니
        # 축이 90° 달라져 계획이 깨졌다 — 반올림이 뭉개는 것은 실재하는 문제지만,
        # 고치면 파지 자세 기준이 바뀌므로 시연 뒤에 충분히 시험하고 손댄다.
        # **물림축을 기준으로 접근축을 직교화한 뒤 반올림한다.**
        # 그냥 반올림하면 원시값 [0.93, -0.368, 0.003] 이 [1,0,0] 이 되어 물림축 [-1,0,0] 과
        # **평행**해진다 → 회전행렬이 망가진 채로 IK 를 푼다. 그 상태에서 approach 가
        # 90.4°(1.577 rad) 를 점프했다 (2026-09-21 무작위 배치 시험).
        # 두 축은 정의상 직교해야 한다 (하나는 손가락이 물리는 방향, 하나는 손이 나아가는 방향).
        self._c_loc = np.round(_c_raw)
        if GRIP_ORTHO:
            # 직교화는 **기본 꺼짐.** 켜 보니 접근축이 [1,0,0] → [0,-1,0] 로 90° 바뀌어
            # 파지 자세가 달라졌고, approach 가 `404 제한 시간 초과` 로 막혔다 (2026-09-21).
            # 원시 접근축이 물림축과 거의 나란한 것 자체가 **측정이 이상하다는 신호**다
            # (ee_frame 과 hand_link 가 같은 링크를 가리키는 문제) — 덮지 말고 드러낸다.
            _cu = self._c_loc / max(float(np.linalg.norm(self._c_loc)), 1e-9)
            _a_perp = _a_raw - float(np.dot(_a_raw, _cu)) * _cu
            if float(np.linalg.norm(_a_perp)) < 1e-6:
                raise RuntimeError(
                    f"접근축이 물림축과 완전히 겹친다 (접근 {np.round(_a_raw, 3).tolist()}, "
                    f"물림 {np.round(_c_raw, 3).tolist()})")
            self._a_loc = np.round(_a_perp / float(np.linalg.norm(_a_perp)))
        else:
            self._a_loc = np.round(_a_raw)
        if abs(float(np.dot(self._a_loc, self._c_loc))) > 1e-6:
            # 평행이면 회전행렬이 안 만들어진다. 멈추지는 않되 **반드시 눈에 띄게** 남긴다
            self.say(f"[경고] 접근축 {self._a_loc.tolist()} 와 물림축 {self._c_loc.tolist()} 가 "
                     f"직교하지 않는다 (원시값 {np.round(_a_raw,3).tolist()} / "
                     f"{np.round(_c_raw,3).tolist()}). 파지 자세가 틀어질 수 있다")
        self.say(f"그리퍼 축: 접근축(손 기준) {np.round(self._a_loc, 3).tolist()}, "
                 f"물림축 {np.round(self._c_loc, 3).tolist()}")
        # **파지·삽입 자세는 팔이 놓인 방향을 따라야 한다.**
        # 예전에는 월드 x/y 로 박혀 있었다. 로봇이 yaw 90° 로 서면 트레이 칸도 90° 돌아가
        # 책 두께가 월드 y 를 향하는데, 손가락은 월드 x 로 물려 해가 없다 (2026-09-20 실측).
        # 팔 기준 x(칸이 늘어선 방향)·y(서가 쪽)를 월드로 돌려 쓴다. yaw 0 이면 예전과 같다.
        _cy, _sy = math.cos(self.tray_yaw), math.sin(self.tray_yaw)
        _arm_x_w = [_cy, _sy, 0.0]          # 칸이 늘어선 방향
        _arm_y_w = [-_sy, _cy, 0.0]         # 서가를 향하는 방향
        _close_w = _arm_y_w if GRIP_ROT90 else _arm_x_w
        self.DOWN = self.orientation([0, 0, -1], _close_w)
        self.HORIZ = self.orientation(_arm_y_w, _arm_x_w)
        self.say(f"파지 자세 기준: 팔 yaw {math.degrees(self.tray_yaw):.1f}°, "
                 f"물림축(월드) {np.round(_close_w, 3).tolist()}"
                 f"{' [SIM_GRIP_ROT90 — 90도 돌림]' if GRIP_ROT90 else ''}")

        # 로봇마다 자세·그리퍼 값이 다르다. franka 는 arm.yaml(검증 완료), 나머지는 arm_<이름>.yaml
        _cfg_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
        _cfg = os.path.join(_cfg_dir, "arm.yaml" if BOT.name == "franka" else f"arm_{BOT.name}.yaml")
        conf = yaml.safe_load(open(_cfg))
        # (예전에는 여기서 tolerance 를 코드로 덮어썼다. yaml 이 무시돼 로봇별 조정이 안 됐다)
        self.conf = conf
        self.arm = ArmController(_Backend(self), conf)

        tray_top = DECK_Z + floor_top + self.W_max
        self._home_tip_z = tray_top + 0.30      # 받침판 기준 높이 — 주행해도 안 변한다
        # **홈은 IK 로 풀지 않는다.** 검증된 관절각을 그대로 쓰고 손끝 자리는 FK 로 얻는다.
        # 예전에는 트레이 위 한 점 + DOWN 자세로 IK 를 풀었는데, 씨앗의 joint_1 에
        # **월드 방위각**을 넣고 있어 뒤집힌 가지(joint_1 ≈ +3.26 rad)로 풀렸다.
        # 그 해는 가지 경계 위라 베이스가 1~2mm 흔들릴 때마다 갈아탔다 (402, 2.9 rad).
        self.q_home = np.asarray(conf["poses"]["home"], float)
        # **갈아 끼우기 전의 홈을 남겨 둔다.** 버리는 값이 아니다 — 스윕 스캔은
        # 이 자세에서 아래 판이 풀리고, `SIM_HOME_Q` 자세에서는 안 풀린다
        # (2026-09-24 실측: 같은 아래 판이 여유 0.166 으로 5/5 vs 0.118 로 3/5).
        # 파지는 바꾼 홈이 좋고 스캔은 원래 홈이 좋다 — **둘을 다 들고 있는다.**
        self.q_home_conf = np.asarray(conf["poses"]["home"], float)
        # **홈 관절각을 밖에서 갈아 끼울 수 있게 한다** (`SIM_HOME_Q`, 쉼표로 7개).
        # 왜: 전 구간 최소 여유 0.128 rad 의 뿌리가 이 자세다. yaml 의 홈은 관절1 이
        # 2.811 rad 이라 한계 ±2.9671 에서 0.156 밖에 안 떨어져 있고, 파지 해는 홈에서
        # 가장 가까운 것을 고르므로 그 좁은 가지에 묶인다 (파지 여유 0.162).
        # 같은 **손 자세**를 내면서 여유가 훨씬 큰 관절 조합이 따로 있다 (2026-09-23 실측:
        # 홈 0.707 · 접근 0.389 · 파지 0.282, 홈→파지 거리는 오히려 더 짧다).
        # 스윙 경로가 q_home[0] 을 기준으로 삼으므로 바꾸면 동선도 달라진다 —
        # 그래서 yaml 을 건드리지 않고 **스위치로** 두어 한 판씩 견줄 수 있게 한다.
        _hq = os.environ.get("SIM_HOME_Q", "").strip()
        if _hq:
            _v = [float(x) for x in _hq.replace(" ", "").split(",") if x]
            if len(_v) != len(self.q_home):
                raise RuntimeError(f"SIM_HOME_Q 는 관절 {len(self.q_home)}개여야 한다 "
                                   f"(받은 것 {len(_v)}개): {_hq}")
            say(f"**홈 자세를 바꿔 끼웠다** {np.round(self.q_home, 3).tolist()} → "
                f"{np.round(np.asarray(_v), 3).tolist()} [SIM_HOME_Q]")
            self.q_home = np.asarray(_v, float)
        # **책 관측 자세는 홈과 분리한다.** 손목 회전·카메라 올리기 스위치는 관측 자세(q_observe)만 바꾼다 — 홈(q_home)을
        # 바꾸면 검증된 파지 접근 계획(plan_job 이 홈에서 시작)이 손목 특이점에 걸린다 (2026-09-23 19:01 실측).
        self.q_observe = self.q_home.copy()
        # **책 관측(홈) 자세에서 그리퍼만 돌리는 스위치** — `SIM_HOME_J7_DEG` (도). 트레이가 로봇 기준 90° 틀어져
        # 실리는 레벨에서, 홈의 손목(관절 7)만 돌려 카메라·물림축을 트레이에 맞춘다 (2026-09-23 사용자 제안).
        _j7 = float(os.environ.get("SIM_HOME_J7_DEG", "0") or 0)
        if abs(_j7) > 1e-6:
            _lo, _hi = -2.9671, 2.9671
            _new = float(np.clip(self.q_observe[6] + math.radians(_j7), _lo, _hi))
            say(f"관측 자세 손목(관절 7) {math.degrees(self.q_observe[6]):+.1f}° → {math.degrees(_new):+.1f}° 로 돌림 [SIM_HOME_J7_DEG={_j7:+.0f}] (홈은 그대로)")
            self.q_observe[6] = _new
        # **홈 자세를 팔 기준으로 옮기는 스위치** — `SIM_HOME_SHIFT="dx,dy,dz"` (m). 손 자세는 그대로 두고 위치만
        # 옮겨 우리 IK(arm_kinematics)로 다시 푼다. 그리퍼가 트레이 책 윗부분을 가려 카메라를 10 cm 올릴 때 쓴다 (2026-09-23).
        _hs = os.environ.get("SIM_HOME_SHIFT", "").strip()
        if _hs:
            try:
                import arm_kinematics as _ak
                _d = np.array([float(v) for v in _hs.split(",")], float)
                _p, _R = _ak.fk(self.q_observe)
                _r = _ak.ik_best(_p + _d, _R, _ak.seeds_around(self.q_observe), prefer=self.q_observe)
                if _r.ok:
                    say(f"관측 자세 이동 {np.round(_d, 3).tolist()} m: 손 {np.round(_p, 3).tolist()} → {np.round(_p + _d, 3).tolist()}, "
                        f"관절 {np.round(self.q_observe, 3).tolist()} → {np.round(_r.q, 3).tolist()} (한계 여유 {_r.limit_margin:.3f}) [SIM_HOME_SHIFT] (홈은 그대로)")
                    self.q_observe = np.asarray(_r.q, float)
                else:
                    say(f"홈 자세 이동 실패 — IK 안 풀림 (위치 오차 {_r.pos_err * 1000:.1f} mm, 여유 {_r.limit_margin:.3f}). 홈 그대로 [SIM_HOME_SHIFT]")
            except Exception as _exc:      # noqa: BLE001 - 스위치 실패가 시뮬을 막지 않게
                say(f"SIM_HOME_SHIFT 실패 {type(_exc).__name__}: {_exc} — 홈 그대로")
        # **②의 첫 단계 — 모델이 같은지부터 본다** (숫자만 찍는다, 동작은 안 바뀐다).
        # `SIM_AK_CHECK=0` 으로 끌 수 있다.
        if os.environ.get("SIM_AK_CHECK", "1") != "0":
            try:
                self.check_ak_model()
            except Exception as _exc:      # noqa: BLE001 - 대조 실패가 기동을 막지 않게
                self.say(f"[모델대조] 못 쟀다 {type(_exc).__name__}: {_exc}")
        # ② — `arm_kinematics` 는 손(panda_hand)을 모형화하고 IK 목표는 ee_frame(right_gripper)
        # 으로 온다. 둘 사이 고정 변환을 **Lula 에서 한 번 재서** 둔다. 손으로 적지 않는다.
        self._ak_tool = None
        try:
            _ph, _Rh = self.lula.compute_forward_kinematics(BOT.hand_link, self.q_home)
            _pe, _Re = self.lula.compute_forward_kinematics(BOT.ee_frame, self.q_home)
            _Rh = np.asarray(_Rh, float); _Re = np.asarray(_Re, float)
            self._ak_tool = (_Rh.T @ (np.asarray(_pe, float) - np.asarray(_ph, float)), _Rh.T @ _Re)
            self.say(f"[IK] 손→{BOT.ee_frame} 고정 변환(손 기준) "
                     f"{np.round(self._ak_tool[0] * 1000, 1).tolist()} mm")
        except Exception as _exc:      # noqa: BLE001
            self.say(f"[IK] 손→ee 변환을 못 쟀다 — SIM_IK_AK 는 Lula 로 되돌아간다: {_exc}")
        self._ak_stats = {"n": 0, "둘다": 0, "Lula만": 0, "ak만": 0, "둘다못": 0, "dq": 0.0, "mm": 0.0}
        _hp, _hR = self.lula.compute_forward_kinematics(BOT.ee_frame, self.q_home)
        self.home_tip = np.asarray(_hp, float)
        self.HOME_ORI = quat_from_R(np.asarray(_hR, float))
        # **홈과 트레이 중앙을 팔 기준으로 굳혀 둔다.** 주행 뒤에는 이 값으로 월드를 다시 만든다.
        # 홈은 트레이가 어디에 앉았느냐가 아니라 **팔에서 본 자리**로 정의되어야 한다.
        self._home_tip_arm = self.to_arm(self.home_tip)
        self._tray_center_arm = self.to_arm(self.tray_center_w)
        _br = self.elbow_branch(self.q_home)
        self.say(f"홈: 관절각 {np.round(self.q_home, 4).tolist()} → "
                 f"손끝(팔기준) {np.round(self._home_tip_arm, 4).tolist()}  "
                 f"가지 **팔꿈치{'↑' if _br == 'up' else '↓' if _br == 'down' else '?'}**")
        if BRANCH_GUARD and _br == "down":
            self.say("[경고] 홈이 팔꿈치↓ 다 — 가지 가드가 모든 IK 를 거절한다. "
                     "arm_<로봇>.yaml 의 poses.home 을 확인할 것")
        elif BRANCH_GUARD and _br is None:
            # **이건 경고가 아니다.** `elbow_branch` 는 link_2/3/5 로 판정하는데 Franka 에는
            # 그 프레임이 없어 늘 None 이고, `_branch_ok` 는 None 이면 통과시킨다 — 가지
            # 가드는 6축용이지 이 팔에는 걸리지 않는다. 예전 문구가 "모든 IK 를 거절할 수
            # 있다" 였던 탓에 2026-09-24 밤 내내 401 의 용의자로 세 번 지목됐다.
            # 실제로 그 판들에서 가드는 한 해도 버리지 않았다 (`_arm_limits` 로그 0회).
            self.say("홈의 가지를 판정하지 않는다 (link_2/3/5 프레임이 없다) — "
                     "가지 가드는 이 팔에서 아무것도 거르지 않는다. 6축에서만 쓰인다")
        self.open_tray = self.T_max / 2 + GRIP_CLEAR
        # 트레이 추종은 **__init__ 맨 마지막**에 설정한다. 그리퍼 축 계산보다 앞에 두었더니
        # ee_frame 과 hand_link 가 같은 자리로 나와 '접근축 를 못 구한다' 로 죽었다 (2026-09-21 실측).
        # 재생·정착이 끝난 **실제 자세**로 로봇↔트레이 상대 변환을 잡는다.
        # 이후 매 스텝 follow_tray() 가 이 관계를 유지한다 → 주행해도 트레이가 따라온다.
        if self._tray_follow:
            # **팔 베이스 링크에 붙인다.** 트레이가 어디에 있어야 하는지는 "팔 기준" 으로
            # 정해져 있다 (칸 좌표가 전부 팔 기준이다). 차체(Cube/articulation_root)에
            # 붙이면 차체와 팔 베이스가 **서로 다른 몸체**라 주행 뒤 둘이 어긋나고,
            # 한쪽을 맞추면 다른 쪽이 틀어진다 — 2026-09-21 실측: 주행 복귀 보정으로
            # 팔 베이스를 제자리에 놓자 트레이가 월드에서 6.1 cm 끌려가 책이 칸 중심에서
            # 5.4 cm 밀렸다 (보정 **전에는** 칸 중심에 정확히 있었다).
            # **기본은 차체(Cube/articulation_root)** — 지금까지의 동작이다.
            # SIM_TRAY_ANCHOR=arm 으로 바꾸면 **팔 베이스 링크**에 붙는다.
            _body = f"{R}/Cube" if st.GetPrimAtPath(f"{R}/Cube").IsValid() else BOT.articulation_root
            _arm_base = f"{R}/{BOT.base_link}"
            if os.environ.get("SIM_TRAY_ANCHOR", "body") == "arm" \
                    and st.GetPrimAtPath(_arm_base).IsValid():
                _anchor = _arm_base
            else:
                _anchor = _body
            _ap, _aq = SingleXFormPrim(_anchor).get_world_pose()
            _tp, _tq = SingleXFormPrim(self.tray).get_world_pose()
            _Ra = R_from_quat(np.asarray(_aq, float))
            self._tray_anchor = _anchor
            self._tray_rel_p = _Ra.T @ (np.asarray(_tp, float) - np.asarray(_ap, float))
            self._tray_rel_R = _Ra.T @ R_from_quat(np.asarray(_tq, float))
            # **트레이 위 책은 self.books 전부다** — 이 장면이 트레이 칸에 직접 놓은 것들이다.
            # 기하로 판정해 봤더니 6권 중 1권만 걸렸다 (2026-09-21 실측). 판정할 것이 아니라
            # 아는 것이다. 서가에 꽂힌 책은 애초에 self.books 가 아니고,
            # 파지한 책은 attach() 에서 빠진다.
            _Rt = R_from_quat(np.asarray(_tq, float))
            self._tray_books = {}
            for _bk in self.books:
                _bp, _bq = SingleXFormPrim(_bk).get_world_pose()
                self._tray_books[_bk] = (
                    _Rt.T @ (np.asarray(_bp, float) - np.asarray(_tp, float)),
                    _Rt.T @ R_from_quat(np.asarray(_bq, float)))
            self.say(f"트레이 위 책 {len(self._tray_books)}/{len(self.books)}권을 같이 옮긴다")
            self.say(f"트레이 기준 링크: {_anchor.rsplit(chr(47), 1)[-1]} "
                     f"(SIM_TRAY_ANCHOR={os.environ.get(chr(83)+chr(73)+chr(77)+chr(95)+'TRAY_ANCHOR', 'body')})")
            self.say(f"트레이가 로봇을 따라간다: {os.path.basename(_anchor)} ↔ bs_tray (키네마틱)")
            # **기준선 모드.** `SIM_HANDS_OFF=1` 이면 트레이·책에 아무것도 하지 않는다 —
            # 이송도, 추종도, 계측도 없다. 레벨을 그냥 재생만 한 상태를 눈으로 보기 위한 것이다.
            # 내가 손대는 코드를 전부 끄고도 책이 칸에 안 들어가면 원인은 레벨·에셋에 있고,
            # 들어가면 원인은 내 코드에 있다. 이 구분이 안 돼서 며칠을 헤맸다 (2026-09-22).
            if HANDS_OFF:
                self.say("**기준선 모드**: 트레이·책에 아무것도 하지 않는다 "
                         "(이송·추종·고정 전부 꺼짐) [SIM_HANDS_OFF=1]")
            else:
                self.start_delivery()
                # world.step() 을 부르는 곳이 셋이다 (run_simulation, manipulation_executor 2군데).
                # 물리 콜백에 물리면 어디서 돌리든 한 번씩만 불린다.
                try:
                    self.world.add_physics_callback(
                        "bs_tray_follow",
                        lambda _dt: (self.hold_base_tick() if HOLD_BASE_TICK else None,
                                     self.follow_hand(),
                                     self.deliver_tick(), self.follow_tray(),
                                     self.tray_watch(),
                                     self.diag_attach_tick()))
                except Exception as exc:     # noqa: BLE001
                    self.say(f"[경고] 트레이 추종 콜백 등록 실패 — 주행하면 트레이가 뒤에 남는다: {exc}")

        self.lock_base_mass()
        say(f"장면 준비: 트레이 칸 {nslots}개, 책 {len(self.books)}권, 원본 {len(sources)}종")
        for b in self.books:
            t, ln, w = self.dims[b]
            say(f"  {b.rsplit('/', 1)[1]}: 두께 {t:.3f} 세운높이 {ln:.3f} 깊이 {w:.3f}")

    # ---------------------------------------------------------------- 도구
    def upright_quat(self, path, q0, pos):
        """책을 트레이 기준 자세로 돌리는 쿼터니언을 찾는다.

        에셋마다 원본 자세가 다르므로 90° 회전 조합(24가지)을 시험해
        x 가 가장 얇고 y 가 가장 긴 자세를 고른다. 물리 시작 전이라 자세만 바꿔 본다.
        """
        def mul(a, b):
            w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
            return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                             w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], float)
        s = math.sqrt(0.5)
        step = {"x": np.array([s, s, 0, 0]), "y": np.array([s, 0, s, 0]), "z": np.array([s, 0, 0, s])}
        cands = []
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    q = q0
                    for _ in range(i): q = mul(step["x"], q)
                    for _ in range(j): q = mul(step["y"], q)
                    for _ in range(k): q = mul(step["z"], q)
                    cands.append(q)
        xf = SingleXFormPrim(path)
        best, best_score = q0, None
        for q in cands:
            xf.set_world_pose(pos, q)
            b = self.aabb(path)
            e = np.array([b[3] - b[0], b[4] - b[1], b[5] - b[2]])
            # 원하는 순서: x 최소, y 최대 → 점수가 낮을수록 좋다
            score = (e[0] - e.min()) + (e.max() - e[1])
            if best_score is None or score < best_score:
                best, best_score = q, score
            if score < 1e-4:
                break
        return best

    def _prim_valid(self, p):
        # 확인할 수 없으면 **유효하지 않다**고 본다. 검사의 기본값이 '통과' 면 검사가 아니다
        return self.stage.GetPrimAtPath(p).IsValid() if getattr(self, "stage", None) else False

    def set_shelf(self, prim_path):
        """현재 서가를 바꾼다 — 서가 상자·앞면·판 목록 캐시가 그 서가 것이 된다. 성공이면 True.

        서가가 둘인 그림(2026-09-25): 팔은 "서가 앞에 서 있고 서가가 팔 기준 +Y" 만 가정하므로
        서가가 바뀌면 **재는 대상**만 바꾸면 된다. 같은 서가면 아무것도 안 한다.
        """
        prim_path = str(prim_path)
        if prim_path == getattr(self, "shelf_prim", SHELF):
            return True
        pr = self.stage.GetPrimAtPath(prim_path)
        if not (pr and pr.IsValid()):
            self.say(f"[서가] 없는 prim 이라 안 바꾼다: {prim_path} (지금 {self.shelf_prim})")
            return False
        bb = np.asarray(self.aabb(prim_path), float)
        if not np.all(np.isfinite(bb)) or np.any(bb[3:] - bb[:3] <= 0):
            self.say(f"[서가] AABB 를 못 재서 안 바꾼다: {prim_path}")
            return False
        self.shelf_prim = prim_path
        self.shelf_aabb_world = bb.copy()
        self.shelf_front_y = float(bb[1])
        self._boards_seen = set()
        self.say(f"[서가] 현재 서가 → {prim_path} · 상자(월드) {np.round(bb, 3).tolist()}")
        return True

    def level_boards(self):
        """레벨(장면)의 서가 메시에서 **판 윗면 z(월드) 목록**을 읽는다. 못 읽으면 [].

        **그림자 모드** (웹 클로드 v42 회신 §4): 읽어서 로그에만 찍고 판단은 상수(`SIM_SWEEP_BOARDS`·
        `SIM_SHELF_ROW_Z`)가 그대로 한다. 동작 변경이 구조적으로 0 이다. 그림자 로그가 여러 판
        연속 상수와 일치하면 그때 스위치로 실측을 쓰게 한다 — 상수를 지우는 게 아니라 **검증된
        캐시**로 강등하는 것이다. `reach_check.read_shelf` 와 같은 규칙(`boards_from_zs`)이다.
        """
        try:
            root = self.stage.GetPrimAtPath(getattr(self, "shelf_prim", SHELF))
            if not (root and root.IsValid()):
                return []
            zs = []
            for pr in Usd.PrimRange(root):
                if not pr.IsA(UsdGeom.Mesh):
                    continue
                pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
                if not pts:
                    continue
                M = UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                zs.extend(float(M.Transform(Gf.Vec3d(float(q[0]), float(q[1]), float(q[2])))[2]) for q in pts)
            return boards_from_zs(zs)
        except Exception as exc:      # noqa: BLE001 - 계측이 기동을 막으면 안 된다
            self.say(f"[판목록] 레벨에서 못 읽었다: {type(exc).__name__}: {exc}")
            return []

    def shadow_boards(self, used):
        """`used`(실제로 쓰는 판 z 목록, 월드)를 레벨 실측과 견줘 **로그만** 남긴다. 값은 안 바꾼다."""
        lv = self.level_boards()
        if not lv:
            self.say(f"[판목록] 레벨 실측 없음 · 쓰는 값 {sorted(round(float(u), 4) for u in used)} (상수)")
            return lv
        diffs = []
        for u in used:
            b = nearest_board(u, lv, 0.10)
            diffs.append("없음" if b is None else f"{(b - float(u)) * 1000:+.1f}")
        self.say(f"[판목록] 레벨 실측 {lv} · 쓰는 값 {sorted(round(float(u), 4) for u in used)} "
                 f"· 차이 [{', '.join(diffs)}] mm  (그림자 — 판단은 상수 그대로)")
        return lv

    def known_boards(self):
        """지금까지 아는 **선반 판 윗면 z(월드) 목록** — 설정/실측한 아래 판 + 꽂으면서 잰 판들."""
        out = {round(float(self.shelf_floor_z), 4)}
        out.update(round(float(b), 4) for b in getattr(self, "_boards_seen", ()))
        return sorted(out)

    def snap_floor(self, z_cmd, tol=0.08):
        """명령이 가리키는 칸 바닥 z(월드) → **실제 판 윗면**으로 맞춘다.

        칸 바닥은 명령의 칸 중심 z 에서 책 반높이를 뺀 **계산값**이다. 그 계산은 아래 판에서
        55 mm 위로 나오는데(칸 중심 0.3399 는 판+반높이가 아니다), 지금까지는 스칼라
        `shelf_floor_z` 로 스냅해 조용히 지워 왔다. **위 판에서는 60 cm 차이라 스냅이 안
        걸렸고** 계산값이 그대로 나가 손을 뗀 순간 책이 59 mm 떨어져 5.4° 틀어졌다
        (2026-09-25 01:02, 한 판에 두 선반을 쓴 첫날). 판은 목록이다:

          1. 아래 판(`shelf_floor_z`) 7 cm 안이면 그대로 — **지금까지의 동작, 안 바뀐다**
          2. 아니면 아는 판 목록에서 `tol` 안의 가장 가까운 판
          3. 없으면 서가 메시에서 `z_cmd` 근처 수평면을 잰다(`measure_shelf_board_z`) — 잰 값은
             목록에 넣어 둔다(조사·판정도 같은 판을 쓴다)
          4. 그래도 없으면 계산값 그대로 — 그리고 **떨어진다고** 경고한다
        """
        z_cmd = float(z_cmd)
        if abs(z_cmd - self.shelf_floor_z) < 0.07:
            return float(self.shelf_floor_z)
        b = nearest_board(z_cmd, self.known_boards(), tol)
        if b is None:
            try:
                b = self.measure_shelf_board_z(z_cmd, tol=tol)
            except Exception as exc:      # noqa: BLE001 - 계측이 작업을 막으면 안 된다
                self.say(f"[선반] 판을 못 쟀다: {type(exc).__name__}: {exc}")
                b = None
            if b is not None:
                self._boards_seen = set(getattr(self, "_boards_seen", set())) | {round(float(b), 4)}
        if b is None:
            self.say(f"[경고] 명령이 가리키는 칸 바닥 {z_cmd:.4f} 근처 {tol * 100:.0f} cm 안에 판이 없다 "
                     f"(아는 판 {self.known_boards()}). 계산값대로 놓으면 **책이 그 차이만큼 떨어져 틀어진다** "
                     f"— 판 z 를 확인할 것")
            return z_cmd
        self.say(f"[선반] 칸 바닥 계산값 {z_cmd:.4f} → 실측 판 윗면 {b:.4f} 로 맞춤 "
                 f"({(b - z_cmd) * 1000:+.1f} mm) [snap_floor]")
        return float(b)

    def measure_shelf_board_z(self, near_z, tol=0.08, min_pts=4):
        """서가 메시에서 `near_z` 에 가장 가까운 **수평면**의 z (월드). 못 찾으면 None.

        판 윗면은 같은 z 에 점이 여러 개 모인다. 그 뭉치들 중 지금 값에 가장 가까운
        것을 고른다 — 아랫면(같은 판의 반대쪽)과 헷갈리지 않도록 `tol` 안만 본다.
        """
        _shelf = getattr(self, "shelf_prim", SHELF)
        root = self.stage.GetPrimAtPath(_shelf)
        if not (root and root.IsValid()):
            root = self.stage.GetPrimAtPath(_shelf.rsplit("/", 1)[0])
        if not (root and root.IsValid()):
            return None
        from collections import Counter
        zs = Counter()
        for pr in Usd.PrimRange(root):
            if not pr.IsA(UsdGeom.Mesh):
                continue
            try:
                pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
                if not pts:
                    continue
                M = UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                for q in pts:
                    w = M.Transform(Gf.Vec3d(float(q[0]), float(q[1]), float(q[2])))
                    z = float(w[2])
                    if abs(z - near_z) <= tol:
                        zs[round(z, 4)] += 1
            except Exception:      # noqa: BLE001 - 계측이 작업을 막으면 안 된다
                continue
        cand = [(abs(z - near_z), z, n) for z, n in zs.items() if n >= min_pts]
        if not cand:
            return None
        return min(cand)[1]

    def aabb(self, p):
        self._cache.Clear()
        return np.array(compute_aabb(self._cache, p, include_children=True), float)

    def center(self, p):
        b = self.aabb(p)
        return (b[:3] + b[3:]) / 2

    def clash_report(self):
        """팔 관절 주변에 **실제 충돌체**가 무엇이 있는지 PhysX 로 묻는다.

        왜: AABB 겹침은 쓸모가 없었다 — compute_aabb 는 Xform 에서 하위 트리를 늘 포함해서
        "link_2 가 트레이와 겹친다(= 그 아래 달린 그리퍼가 트레이에 있다)" 는 당연한 답만 나왔다.
        PhysX overlap 은 콜라이더 단위라 **무엇이 팔을 막고 있는지** 바로 나온다 (2026-09-20).
        """
        try:
            from omni.physx import get_physx_scene_query_interface
            sq = get_physx_scene_query_interface()
        except Exception as e:
            return f"[진단] PhysX 조회 불가: {e}"
        # 로봇 이름을 박으면 다른 프로파일에서 전부 건너뛰고 **'없음' 이라는 거짓 안심**을 준다.
        # 이 진단은 "서가인 줄 알았는데 아니었다" 를 막으려고 만든 것이라 그게 제일 나쁘다.
        _arm_dir = os.path.dirname(BOT.hand_link)          # m0609 → 'm0609', franka → ''
        _pre = f"{R}/{_arm_dir}" if _arm_dir else R
        hits = []
        for i in range(1, BOT.dof + 1):
            lp = f"{_pre}/link_{i}"
            if not self._prim_valid(lp):
                continue
            pos = np.asarray(SingleXFormPrim(lp).get_world_pose()[0], float)
            found = set()

            def _cb(h, _f=found):
                _f.add(str(h.collision))
                return True

            try:
                sq.overlap_sphere(0.18, [float(v) for v in pos], _cb, False)
            except Exception as e:
                return f"[진단] overlap 실패: {e}"
            # **팔 자신만** 뺀다. 받침판·카터 몸체는 남긴다 —
            # 이것들을 걸러냈다가 "팔 주변에 아무것도 없다" 는 잘못된 결론을 냈다 (2026-09-20).
            _self = f"/{_arm_dir}/link_" if _arm_dir else "/link_"
            out = sorted({f.split("/World/")[-1] for f in found if _self not in f})
            if out:
                hits.append(f"link_{i} 반경18cm: " + ", ".join(x[:46] for x in out[:4]))
        return ("[진단] 팔 주변 물체 — " + " | ".join(hits)) if hits else "[진단] 팔 주변에 바깥 물체 없음"

    def refresh_base(self):
        """팔 베이스의 **지금** 월드 자세를 읽고, 거기 딸린 것을 **전부** 다시 잡는다.

        왜 필요한가: 예전에는 시작할 때 한 번 읽은 값을 끝까지 썼다. 그러면 AMR 이
        주행한 만큼 **IK 목표도 좌표 계약도 통째로 어긋난다** (2026-09-21 확인).
        주행을 멈춘 자리에서 파지·반납을 하려면 그 자리 기준으로 다시 세워야 한다.

        다시 잡는 것 (전부 베이스 자세에서 유도되는 값이다):
          - `l0p` / `Rl0`        좌표 계약 변환 (`to_world` / `to_arm`)
          - Lula 베이스 자세      IK·FK 가 월드 목표를 푸는 기준
          - `tray_yaw`           로봇이 보고 있는 방향
          - `DOWN` / `HORIZ`     파지·삽입 손 자세 (방향을 따라 돌아야 한다)
          - `tray_center_w`      트레이 월드 중앙 (팔 기준 값에서 다시 만든다)
          - `home_tip`           홈 위치 (트레이 위 고정 높이)

        `q_home` 은 **관절각**이라 다시 풀지 않는다 — 팔에서 본 홈은 그대로다.

        돌려주는 값: 지난번 기준점에서 팔 베이스가 움직인 거리 (m).
        """
        l0p, l0q = SingleXFormPrim(BASE_LINK).get_world_pose()
        l0p = np.asarray(l0p, float)
        l0q = np.asarray(l0q, float)
        moved = float(np.linalg.norm(l0p - self.l0p))
        self.l0p = l0p
        self.Rl0 = R_from_quat(l0q)
        self.lula.set_robot_base_pose(l0p, l0q)
        if os.environ.get("SIM_TRACE_BASE", "0") != "0" and getattr(self, "robot", None) is not None:
            # 비결정성 출처 (NIGHTLY F-6): 입력(비전)이 같아도 판마다 베이스·시작 관절이 다른지
            try:
                self.say(f"[refresh_base] 팔베이스 {np.round(l0p, 4).tolist()} q {np.round(l0q, 5).tolist()} "
                         f"시작관절 {np.round(np.asarray(self.robot.get_joint_positions()[self.idx_arm], float), 4).tolist()}")
            except Exception as _exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                self.say(f"[refresh_base] 기록 실패: {type(_exc).__name__}: {_exc}")

        # 방향은 yaw 만 쓴다. 베이스에 섞인 기울기를 손 자세에 넣으면 책이 기운다
        yaw = math.atan2(float(self.Rl0[1, 0]), float(self.Rl0[0, 0]))
        # **손 자세는 yaw 에만 의존한다 — yaw 가 그대로면 다시 만들지 않는다.**
        # 같은 값을 다시 계산해도 쿼터니언이 미세하게 달라지고, 그 차이로 IK 가
        # 다른 가지를 골라 `402 approach 2.9 rad` 로 죽는다. 제자리에서든 평행이동에서든
        # 마찬가지였다 (2026-09-21: 무작위 배치 시험에서 옮기기만 해도 3/4 가 실패).
        _yaw_changed = abs(yaw - self.tray_yaw) > 1e-6
        _yaw_before = float(self.tray_yaw)
        self.tray_yaw = float(yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        # **팔 기준으로 굳혀 둔 값에서 월드를 다시 만든다.** 로봇과 함께 통째로 따라오므로
        # 팔에서 본 자리는 언제나 그대로고, q_home 이 계속 유효하다.
        self.home_tip = self.to_world(self._home_tip_arm)
        self.tray_center_w = self.to_world(self._tray_center_arm)
        self.tray_y = float(self.tray_center_w[1])
        if _yaw_changed:
            _arm_x_w = [cy, sy, 0.0]        # 칸이 늘어선 방향
            _arm_y_w = [-sy, cy, 0.0]       # 서가를 향하는 방향
            self.DOWN = self.orientation([0, 0, -1], _arm_y_w if GRIP_ROT90 else _arm_x_w)
            self.HORIZ = self.orientation(_arm_y_w, _arm_x_w)
            # **홈 손자세도 같이 돌린다.** `HOME_ORI` 는 순기구학으로 잡은 **월드** 자세라
            # 로봇이 돌면 같이 돌아야 한다. 안 돌리면 approach 가 출발하자마자
            # 90°(1.558 rad) 점프해 `401 손목 특이점` 으로 죽는다
            # (2026-09-22 실측: 튐 지점이 경로 1312 스텝 중 **1번째**였다).
            _d = yaw - _yaw_before
            _Rz = np.array([[math.cos(_d), -math.sin(_d), 0.0],
                            [math.sin(_d), math.cos(_d), 0.0],
                            [0.0, 0.0, 1.0]])
            if getattr(self, "HOME_ORI", None) is not None:
                self.HOME_ORI = quat_from_R(_Rz @ R_from_quat(np.asarray(self.HOME_ORI, float)))
                self.say(f"홈 손자세도 {math.degrees(_d):+.1f}° 같이 돌렸다")
            self.say(f"로봇이 돌았다 — 손 자세 기준을 다시 잡았다 (yaw {math.degrees(yaw):+.1f}°)")
        return moved

    def yaw_to_arm(self, p_world):
        """월드 점 → **팔이 보는 방향** 기준 (x 칸 방향, y 서가 방향). z 는 월드 그대로.

        삽입 경로가 "칸은 월드 x 로 늘어서고 서가는 월드 +y 에 있다"고 가정하던 것을
        걷어내기 위한 변환이다. yaw 0 이면 원점만 옮기는 것이라 예전 값과 **완전히 같다**.
        높이는 돌리지 않는다 — 베이스에 섞인 기울기가 삽입 높이에 새면 안 된다.
        """
        p = np.asarray(p_world, float)
        cy, sy = math.cos(self.tray_yaw), math.sin(self.tray_yaw)
        dx, dy = p[0] - self.l0p[0], p[1] - self.l0p[1]
        return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p[2]])

    def yaw_to_world(self, p_yaw):
        """yaw_to_arm 의 역변환"""
        p = np.asarray(p_yaw, float)
        cy, sy = math.cos(self.tray_yaw), math.sin(self.tray_yaw)
        return np.array([self.l0p[0] + cy * p[0] - sy * p[1],
                         self.l0p[1] + sy * p[0] + cy * p[1],
                         p[2]])

    def plan_scan(self, table=None, home=None):
        """서가 스캔 자세들을 **관절 해**로 푼다 → [(이름, q, 메타), ...].

        **베이스를 움직이지 않는다.** 반납할 자리에서 팔만 접어 훑는다 (scan_planner 참조).
        푸는 순서가 중요하다: 앞 자세를 씨앗으로 여러 번 풀어 **가장 가까운 해**를 고른다.
        하나만 씨앗으로 쓰면 자세마다 다른 가지가 잡혀 관절이 254° 까지 돌았다 (9/22 실측).

        좌표는 팔 기준이므로 **지금의 팔 베이스**를 먼저 다시 읽는다 — 주행 뒤에는
        시작할 때 읽은 값이 낡아 있다.
        """
        import scan_planner as _sp

        self.refresh_base()
        _sp_home = np.asarray(self.q_home if home is None else home, float)
        q_prev = _sp_home
        out = []
        # 예전 맵에서 잰 0.749 m 를 쓰지 않는다. 지금 로드한 USD의 서가 앞면을
        # 현재 arm_base_link 좌표로 매번 바꿔 스캔 목표로 쓴다.
        # 월드 AABB의 여덟 꼭짓점을 팔 좌표로 바꾸고, 로봇 정면(+Y)에 있는
        # 가장 가까운 면을 앞면으로 고른다. 월드 y 최소값만 쓰면 로봇 yaw가
        # 바뀐 맵에서 옆면/뒷면을 앞면으로 오인한다.
        _bb = np.asarray(self.shelf_aabb_world, float)
        _corners = [np.array([x, y, z], float)
                    for x in (_bb[0], _bb[3])
                    for y in (_bb[1], _bb[4])
                    for z in (_bb[2], _bb[5])]
        _front_candidates = [float(self.to_arm(p)[1]) for p in _corners
                             if float(self.to_arm(p)[1]) > 0.10]
        if not _front_candidates:
            raise RuntimeError("선택한 서가가 arm_base_link 정면(+Y)에 없다")
        _shelf_face_y = min(_front_candidates)
        if not 0.35 <= _shelf_face_y <= 1.40:
            raise RuntimeError(
                f"서가 앞면 거리 {_shelf_face_y:.3f} m가 안전 범위 0.35~1.40 m 밖이다")
        self.say(f"[스캔] 현재 USD 서가 앞면(팔기준) y={_shelf_face_y:.3f} m")
        for name, hp, hR, meta in _sp.scan_poses(
                table or _sp.SCAN_TABLE, arm_base_z=float(self.l0p[2]),
                shelf_face_y=_shelf_face_y):
            wp = self.to_world(hp)
            wq = quat_from_R(self.Rl0 @ hR)
            # **가드를 거친다.** 여기서 Lula 를 직접 부르면 `_arm_limits`·
            # `_branch_ok`·`_above_deck` 가 전부 빠진다. 그 결과 도달 불가 해
            # (관절6 이 Lula 한계 3.75 인데 실제 한계 3.0)를 향해 3.03 rad 을 감으며
            # 데크와 트레이를 지나가다 404 로 죽었다 (2026-09-23 비전팀 실측).
            # 다중 씨앗은 비전 브랜치에서 온 개선이라 살린다 — **가드를 거쳐서** 쓴다.
            cands = []
            self._ik_phase = "scan"        # [IK대조] 단계별 집계용
            for seed in _sp.seeds_for(q_prev, _sp_home):
                q_c, ok_c = self.ik_joints(wp, wq, np.asarray(seed, float),
                                           frame=BOT.hand_link)
                if not ok_c:
                    continue
                if not any(float(np.max(np.abs(q_c - o))) < 1e-3 for o in cands):
                    cands.append(q_c)
            q = _sp.pick_closest(cands, q_prev)
            if q is None:
                self.say(f"[스캔] {name} 해 없음 — 건너뛴다")
                continue
            step = _sp.step_size(q_prev, q)
            if out and step > _sp.MAX_STEP_RAD:
                # 크게 도는 것 자체는 막지 않는다. **알아챌 수 있게** 남긴다
                self.say(f"[스캔] {name} 에서 관절이 크게 돈다 "
                         f"{math.degrees(step):.0f}° — 자세 표를 다시 볼 것")
            out.append((name, q, dict(meta, step_rad=step, candidates=len(cands))))
            q_prev = q
        self.say(f"[스캔] 자세 {len(out)}/{len(table or _sp.SCAN_TABLE)} 개 계획")
        return out

    def set_scan_tray_guard(self, enabled):
        """스캔 중 트레이만 키네마틱으로 고정한다.

        스캔에는 트레이를 움직일 이유가 없다. 팔 반작용이나 차체의 미세 이동으로
        별도 rigid body인 트레이가 데크에서 미끄러지는 사고를 막고, 스캔 종료 시
        원래 동적 상태로 되돌린다.
        """
        prim = self.stage.GetPrimAtPath(self.tray)
        if not prim.IsValid():
            raise RuntimeError(f"트레이 prim 없음: {self.tray}")
        attr = UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr()
        if enabled:
            if getattr(self, "_scan_tray_guard", None) is not None:
                return
            p, q = SingleXFormPrim(self.tray).get_world_pose()
            self._scan_tray_guard = (bool(attr.Get()), np.asarray(p, float).copy(),
                                     np.asarray(q, float).copy())
            attr.Set(True)
            SingleXFormPrim(self.tray).set_world_pose(
                self._scan_tray_guard[1], self._scan_tray_guard[2])
            self.say("[스캔안전] 트레이 위치를 키네마틱으로 고정했다")
            return
        guard = getattr(self, "_scan_tray_guard", None)
        if guard is None:
            return
        SingleXFormPrim(self.tray).set_world_pose(guard[1], guard[2])
        attr.Set(guard[0])
        self._scan_tray_guard = None
        self.say("[스캔안전] 트레이 고정을 풀고 원래 동적 상태로 복구했다")

    def check_ak_model(self, qs=None, tol_mm=1.0):
        """`arm_kinematics` 의 기구학 모델이 **Lula 와 같은 답을 내는가** — 한 번만 잰다.

        ②(로봇팔 구동을 `arm_kinematics` 호출로 옮기기)의 첫 단계다. IK 를 옮기기 전에
        **FK 가 같은지부터** 본다. 여기서 안 맞으면 IK 를 옮겨 봐야 엉뚱한 데로 간다.

        **동작을 바꾸지 않는다.** 숫자만 찍는다. 그래서 위험이 0 이고, 판이 돌 때
        로그로 확인만 하면 된다.

        `ak.fk` 는 **팔 기준**, Lula 는 **월드**다. `to_world` 로 맞춰 견준다.
        `ak` 는 Franka 전용 DH 표라, 다른 로봇에서는 안 맞는 것이 정상이다 —
        그때는 그렇게 말한다.
        """
        try:
            import arm_kinematics as _ak
        except Exception as exc:      # noqa: BLE001
            self.say(f"[모델대조] arm_kinematics 를 못 읽었다: {exc}")
            return None
        if qs is None:
            qs = [np.asarray(self.q_home, float),
                  np.asarray(getattr(self, "q_observe", self.q_home), float)]
            qs.append(qs[0] + np.array([0.2, -0.15, 0.1, 0.12, -0.2, 0.15, -0.1])[:len(qs[0])])
        worst, rows = 0.0, []
        for i, q in enumerate(qs):
            q = np.asarray(q, float)
            if len(q) != 7:
                rows.append(f"  q{i}: 관절 {len(q)}개 — ak 는 7축(Franka) 전용이라 건너뜀")
                continue
            try:
                # **손(hand_link)과 견준다.** `ak.fk` 는 panda_hand 다. ee_frame(right_gripper)과
                # 견주면 손끝 오프셋(약 10 cm)이 "모델이 다르다" 로 잘못 읽힌다.
                lp = np.asarray(self.lula.compute_forward_kinematics(BOT.hand_link, q)[0], float)
            except Exception as exc:      # noqa: BLE001
                rows.append(f"  q{i}: Lula FK 실패 {type(exc).__name__}")
                continue
            ap, _aR = _ak.fk(q)
            aw = self.to_world(ap)
            d = float(np.linalg.norm(aw - lp)) * 1000.0
            worst = max(worst, d)
            rows.append(f"  q{i}: Lula {np.round(lp, 4).tolist()} · ak→월드 "
                        f"{np.round(aw, 4).tolist()} · 차이 **{d:.2f} mm**")
        verdict = ("두 모델이 같다 — IK 를 옮겨도 된다" if worst <= tol_mm
                   else f"**{tol_mm:.1f} mm 를 넘는다 — IK 를 옮기기 전에 모델을 먼저 맞춰야 한다**")
        self.say(f"[모델대조] arm_kinematics vs Lula ({BOT.hand_link} 위치)\n" + "\n".join(rows)
                 + f"\n  최대 차이 {worst:.2f} mm → {verdict}")
        return worst

    def to_world(self, p_arm):
        return self.l0p + self.Rl0 @ np.asarray(p_arm, float)

    def to_arm(self, p_world):
        return self.Rl0.T @ (np.asarray(p_world, float) - self.l0p)

    def orientation(self, approach, closing):
        b_l = np.cross(self._a_loc, self._c_loc); a_w = np.array(approach, float); c_w = np.array(closing, float)
        Lm = np.stack([self._a_loc, self._c_loc, b_l], axis=1); Wm = np.stack([a_w, c_w, np.cross(a_w, c_w)], axis=1)
        return quat_from_R(Wm @ Lm.T)

    def elbow_branch(self, q):
        """팔꿈치가 어깨–손목 선보다 **위**인가. 관절 원점만 보므로 충돌 구 정확도와 무관.

        6축은 같은 손 자세에 해가 **정확히 8개**뿐이다 (7축 Franka 처럼 연속으로 비켜설 수
        없다). 그 8개는 팔꿈치↓ 4개와 팔꿈치↑ 4개로 갈리고, **팔꿈치↓ 는 윗팔이 수평 아래
        15.9° 로 내려가 받침판에 닿는다.** 그래서 가지를 고르는 것이 전부다.
        (근거: 웹 클로드 v21 회신 §1, 도구 simulation/isaac/tools/branch/m0609_kin.py)
        """
        try:
            sh = np.asarray(self.lula.compute_forward_kinematics("link_2", q)[0], float)
            el = np.asarray(self.lula.compute_forward_kinematics("link_3", q)[0], float)
            wr = np.asarray(self.lula.compute_forward_kinematics("link_5", q)[0], float)
        except Exception:      # noqa: BLE001 — 프레임을 모르면 판정하지 않는다
            return None
        line = wr - sh
        den = float(np.dot(line, line))
        if den < 1e-12:
            return None
        t = float(np.dot(el - sh, line)) / den
        return "up" if float((el - (sh + t * line))[2]) > 0 else "down"

    def _branch_ok(self, q):
        """팔꿈치↑ 이고, 어깨가 홈과 같은 쪽을 보는가.

        두 번째 조건은 어깨가 반대로 도는 팔꿈치↑ 해(j1 ≈ +2.87)를 막는다 —
        가지는 맞아도 팔 전체가 뒤로 돈다.
        """
        if not BRANCH_GUARD:
            return True
        br = self.elbow_branch(q)
        if br is None:
            # **판정할 수 없으면 통과시킨다.** Franka 는 link_2/3/5 라는 프레임이 없어서
            # 늘 None 이 나온다 — 여기서 False 를 주면 **모든 IK 해가 거절되어 Franka 가
            # 통째로 멈춘다.** 가지 문제는 6축(해가 8개뿐)에만 있는 것이다.
            return True
        if br != "up":
            return False
        d = float(q[0]) - float(self.q_home[0])
        d = (d + math.pi) % (2 * math.pi) - math.pi          # −π~π 로 감싼다
        return abs(d) < math.pi / 2

    def _above_deck(self, q):
        """이 관절각에서 **팔이 받침판을 뚫지 않는가** (기술서 충돌 구로 검사).

        Lula 는 기구학만 본다. 어깨를 받침판 속으로 밀어 넣는 해도 멀쩡히 돌려준다.
        그 해로 경로를 짜면 팔이 물리적으로 못 가고 `404 제한 시간 초과` 가 난다 —
        경유점을 절반쯤 보낸 채 joint_2 가 -1.43 에서 멈춘다 (2026-09-21 실측).

        링크 원점 높이만 보면 부족했다 (원점은 판 위인데 몸통이 닿는다). 기술서의
        링크별 충돌 구를 FK 로 월드에 놓고, **구의 아랫점이 판보다 아래면** 버린다.
        """
        if not getattr(self, "_deck_spheres", None):
            return True
        fk = {}
        for name, c, r in self._deck_spheres:
            if name not in fk:
                try:
                    fk[name] = self.lula.compute_forward_kinematics(name, np.asarray(q, float))
                except Exception:      # noqa: BLE001
                    fk[name] = None
            if fk[name] is None:
                continue
            p, R = fk[name]
            z = float((np.asarray(p, float) + np.asarray(R, float) @ c)[2]) - r
            if z < DECK_Z - DECK_SINK_M:
                if os.environ.get("SIM_TRACE_IK", "0") != "0":
                    self._deck_rej = getattr(self, "_deck_rej", {})
                    key = f"{name} 바닥 z={z:.3f}"
                    self._deck_rej[key] = self._deck_rej.get(key, 0) + 1
                    if self._deck_rej[key] in (1, 100):
                        self.say(f"[받침판] {name} 구 바닥 {z:.3f} < {DECK_Z - DECK_SINK_M:.3f} "
                                 f"— 해를 버린다 ({self._deck_rej[key]}번째)")
                return False
        return True

    def _unwind_plan(self, segs):
        """경로에서 **2π 감김을 풀고**, 전체를 관절 한계 안으로 평행이동한다.

        왜: 손목(joint_6)이 운반 중에 계속 같은 방향으로 돌다 한계(±2π)에 닿으면
        Lula 가 2π 반대편 값을 돌려준다. 그러면 인접점 변화가 6.03 rad 로 잡혀 `402` 가
        난다 — 실제로는 0.2 rad 만 움직이는데도 (2026-09-21 실측).

        푸는 방법: 관절별로 `np.unwrap` 으로 이어 붙인 뒤, **2π 의 정수배만큼 통째로**
        옮겨 한계 안에 들어가게 한다. 2π 평행이동은 **자세를 바꾸지 않는다.**
        한계 안에 넣을 수 없으면 그 관절은 그대로 둔다 (진짜로 못 도는 것이다).

        돌려주는 값: (고친 segs, 고친 관절 이름들)
        """
        order = ["approach", "down", "lift", "carry_rotate", "wedge", "back",
                 "touch", "push", "retreat", "return"]
        names = [n for n in order if n in segs]
        if not names:
            return segs, ""
        lens = [len(segs[n]) for n in names]
        flat = np.concatenate([np.asarray(segs[n], float) for n in names], axis=0)
        lo, hi = self._joint_limits()
        fixed = []
        for j in range(flat.shape[1]):
            col = np.unwrap(flat[:, j])
            if np.allclose(col, flat[:, j], atol=1e-9):
                continue                      # 감긴 곳이 없다
            if lo is not None:
                k = 0
                # 한계 안에 들어가는 2π 배수를 찾는다 (가운데에 가장 가깝게)
                for cand in range(-3, 4):
                    c = col + cand * 2.0 * math.pi
                    if c.min() >= lo[j] - 1e-6 and c.max() <= hi[j] + 1e-6:
                        if k == 0 or abs(c.mean()) < abs(col + k * 2 * math.pi).mean():
                            k = cand
                c = col + k * 2.0 * math.pi
                if c.min() < lo[j] - 1e-6 or c.max() > hi[j] + 1e-6:
                    continue                  # 어떻게 옮겨도 한계를 넘는다 — 손대지 않는다
                col = c
            flat[:, j] = col
            fixed.append(f"joint_{j+1}")
        if not fixed:
            return segs, ""
        out, i = {}, 0
        for n, ln in zip(names, lens):
            out[n] = [flat[i + k].copy() for k in range(ln)]
            i += ln
        for n in segs:
            out.setdefault(n, segs[n])
        return out, ", ".join(fixed)

    def _unwrap_to(self, q, seed):
        """관절각을 **씨앗에 가장 가까운 같은 각도**로 바꾼다 (2π 감김 풀기).

        Lula 는 같은 자세를 ±2π 다른 값으로 돌려주기도 한다. 그대로 두면 인접점
        관절변화가 6.03 rad (≈345°) 로 잡혀 `402` 가 난다 — 실제로는 안 움직이는데도.
        2π 를 더하고 빼는 것은 **자세를 바꾸지 않는다**. 다만 관절 한계를 넘으면 안 되므로,
        한계를 알 수 있으면 그 안에 있을 때만 바꾼다.
        """
        q = np.asarray(q, float).copy()
        seed = np.asarray(seed, float)
        lo, hi = self._joint_limits()
        n = min(len(q), len(seed))
        for i in range(n):
            k = round((seed[i] - q[i]) / (2.0 * math.pi))
            if k == 0:
                continue
            cand = q[i] + k * 2.0 * math.pi
            if lo is not None and not (lo[i] - 1e-6 <= cand <= hi[i] + 1e-6):
                continue          # 한계를 벗어나면 그대로 둔다
            q[i] = cand
        return q

    def _joint_limits(self):
        """(하한, 상한) 배열. 못 얻으면 (None, None) — 그때는 감김 풀기를 무조건 한다"""
        if hasattr(self, "_jl_cache"):
            return self._jl_cache
        lo = hi = None
        for name in ("get_cspace_position_limits", "get_joint_limits"):
            fn = getattr(self.lula, name, None)
            if fn is None:
                continue
            try:
                lo, hi = fn()
                lo = np.asarray(lo, float); hi = np.asarray(hi, float)
                break
            except Exception:      # noqa: BLE001  — API 는 버전마다 다르다
                lo = hi = None
        self._jl_cache = (lo, hi)
        if lo is not None:
            self.say(f"관절 한계 확인: {np.round(lo, 2).tolist()} ~ {np.round(hi, 2).tolist()}")
        else:
            self.say("[주의] 관절 한계를 못 얻었다 — 2π 감김 풀기를 한계 검사 없이 한다")
        return self._jl_cache

    def _arm_limits(self):
        """**레벨 아티큘레이션에 적힌 실제** 관절 한계 (rad). 못 읽으면 (None, None).

        왜 Lula 것을 안 쓰나: 둘이 다르다. 2026-09-22 실측 —
        Lula 는 `panda_joint6` 상한을 3.75 로 아는데 레벨의 조인트는 **3.0000** 이다.
        그래서 Lula 가 3.427 을 요구하는 해를 내놓고, 팔은 3.000 에서 영원히 멈춰
        `404 wedge: 제한 시간 초과` 가 났다. Lula 에는 한계를 **설정하는 API 가 없으므로**
        (읽기 전용) 여기서 해를 걸러야 한다.
        """
        if hasattr(self, "_al_cache"):
            return self._al_cache
        lo = np.full(len(BOT.arm_joints), -np.inf)
        hi = np.full(len(BOT.arm_joints), np.inf)
        found = 0
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(BOT.root)):
            if not prim.IsA(UsdPhysics.RevoluteJoint):
                continue
            name = prim.GetName()
            if name not in BOT.arm_joints:
                continue
            j = UsdPhysics.RevoluteJoint(prim)
            a, b = j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()
            if a is None or b is None:
                continue
            k = BOT.arm_joints.index(name)
            lo[k], hi[k] = math.radians(float(a)), math.radians(float(b))
            found += 1
        self._al_cache = (lo, hi) if found == len(BOT.arm_joints) else (None, None)
        if found == len(BOT.arm_joints):
            self.say(f"**실제** 관절 한계 (레벨): {np.round(lo, 3).tolist()} ~ "
                     f"{np.round(hi, 3).tolist()}")
            _llo, _lhi = self._joint_limits()
            if _lhi is not None:
                _d = np.max(np.abs(np.asarray(_lhi, float)[:len(hi)] - hi))
                if _d > 0.05:
                    self.say(f"[주의] Lula 가 아는 한계와 **{_d:.3f} rad 까지 다르다** — "
                             f"실제 한계로 IK 해를 거른다")
        else:
            self.say(f"[주의] 실제 관절 한계를 {found}/{len(BOT.arm_joints)} 개만 읽었다 "
                     f"— 거르지 않는다")
        return self._al_cache

    def _ak_capable(self, frame, seed):
        """`arm_kinematics` 가 이 프레임·이 관절 수를 풀 수 있는가 (손·손끝, 7축)."""
        return (len(np.asarray(seed, float)) == 7 and frame in (BOT.ee_frame, BOT.hand_link)
                and (frame == BOT.hand_link or getattr(self, "_ak_tool", None) is not None))

    def _ak_target(self, frame, target_w, ori_w):
        """월드 목표 → `arm_kinematics` 가 받는 팔 기준 목표 `(p, R, tool)`.

        손끝(ee_frame) 목표는 손 자세로 바꾸지 않고 **tool 로 넘겨 오차를 손끝에서 재게** 한다.
        손 자세로 바꿔 손에서 0.05 rad 를 허용하면 손끝은 0.1 m 지렛대로 5 mm 어긋난다 —
        compare 판(2026-09-25 02:20)에서 `되돌린 손끝 오차 6.97 mm` 가 그것이었다.
        """
        p = self.to_arm(np.asarray(target_w, float))
        Rm = self.Rl0.T @ R_from_quat(np.asarray(ori_w, float))
        return p, Rm, (self._ak_tool if frame == BOT.ee_frame else None)

    def _ak_rescue_seeds(self, frame, target_w, ori_w, seed):
        """Lula 가 씨앗 전부에서 못 풀었을 때 **씨앗으로 다시 넣을 해**를 `arm_kinematics` 로.

        2026-09-25 00:40 실측: 스윙 끝(p_sw)에서 DOWN→HORIZ 뒤집기가 `IK 실패` 였는데, 같은 점을
        오프라인에서 풀면 한계 여유 0.76 이었다(위 판 pre_ins 0.22 의 세 배). 도달 한계가 아니라
        **Lula 가 국소 풀이라 90° 뒤집기를 그 씨앗들에서 수렴 못 한 것**이다. 그래서 ak 해를 씨앗으로
        준다 — 결과는 여전히 Lula 해이고 가드(팔꿈치·한계·받침판)를 똑같이 거친다. Lula 가 푸는
        판에는 한 번도 불리지 않는다.
        """
        if not self._ak_capable(frame, seed):
            return []
        import arm_kinematics as _ak
        p, Rm, tool = self._ak_target(frame, target_w, ori_w)
        out = _ak.rescue_seeds(p, Rm, np.asarray(seed, float), pos_tol=0.004, rot_tol=0.05,
                               min_margin=0.0, tool=tool)
        self._ak_rescued = getattr(self, "_ak_rescued", 0) + 1
        if self._ak_rescued in (1, 10, 100):
            self.say(f"[IK] 씨앗 구조 ({self._ak_rescued}번째): Lula 가 씨앗 전부에서 못 풀어 "
                     f"arm_kinematics 해 {len(out)}개를 씨앗으로 다시 푼다 — "
                     f"목표(팔기준) {np.round(p, 4).tolist()} {frame}")
        return out

    def _dump_ik(self, frame, target_w, ori_w, seed, q_l, ok_l):
        """IK 문제 하나를 **그대로** 파일에 남긴다 (`SIM_IK_DUMP=<jsonl>`) — 오프라인 대조용.

        웹 클로드 v42 회신 §3: 대조에 필요한 건 "구간 · 목표 자세(프레임 명시) · 씨앗 · Lula 해" 네
        필드다. 그러면 Isaac 없이 `tools/ik_compare.py` 가 수천 개를 몇 초에 돌린다 — 시뮬 안에서
        두 번 풀어 heartbeat 를 늘리던 길(문턱 올리기 냄새)은 버린다. 목표는 **팔 기준·손끝 프레임**
        으로 적고 손→손끝 변환(tool)도 같이 적는다. 동작은 안 바뀐다 — 쓰기만 한다.
        """
        import json
        try:
            p, Rm, tool = self._ak_target(frame, target_w, ori_w)
            rec = {"phase": getattr(self, "_ik_phase", "?"), "frame": str(frame),
                   "p_arm": [round(float(v), 6) for v in p],
                   "R_arm": [round(float(v), 6) for v in np.asarray(Rm, float).ravel()],
                   "tool": None if tool is None else
                   {"T": [round(float(v), 6) for v in tool[0]],
                    "R": [round(float(v), 6) for v in np.asarray(tool[1], float).ravel()]},
                   "seed": [round(float(v), 6) for v in seed],
                   "lula_ok": bool(ok_l),
                   "lula_q": None if not ok_l else [round(float(v), 6) for v in np.asarray(q_l, float)]}
            with open(os.environ["SIM_IK_DUMP"], "a") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception as exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
            self._dump_err = getattr(self, "_dump_err", 0) + 1
            if self._dump_err == 1:
                self.say(f"[IK덤프] 못 썼다: {type(exc).__name__}: {exc}")

    def _solve_ik(self, frame, target_w, ori_w, seed):
        """IK 한 번 — **어느 풀이기로 풀지 여기서만 고른다.** 돌려주는 값은 Lula 와 같은 `(q, ok)`.

        `SIM_IK_AK` (②, 로봇팔 구동을 `arm_kinematics` 호출로):
          0        Lula 만 (기본, 지금까지의 동작)
          compare  Lula 로 풀어 **그대로 쓰고**, 같은 목표를 ak 로도 풀어 차이만 센다 — 위험 0
          1        ak 로 풀어 쓴다. ak 가 못 푸는 프레임(손·손끝 밖)은 Lula 로 되돌아간다

        ak 는 팔 기준·손(panda_hand) 기준이다. 월드 목표를 `to_arm` 으로, 손끝(ee_frame) 목표를
        기동 때 잰 `_ak_tool` 로 손 자세로 바꿔 넣는다. 허용 오차는 Lula 호출과 같은 값
        (0.004 m · 0.05 rad)이고 한계 여유 필터는 0 이다 — 한계 검사는 `ik_joints` 의 가드
        `_arm_limits` 가 두 풀이기에 똑같이 건다. 그래서 compare 에서 나는 차이는 풀이기 차이
        이지 기준 차이가 아니다.

        **원시 Lula IK 호출은 저장소 안에서 여기 한 곳뿐이어야 한다.** 회귀 점검:
          grep -rc 'self[.]lula[.]compute_inverse_kinematics(' controllers/ → **1**
        """
        mode = os.environ.get("SIM_IK_AK", "0").strip()
        target_w = np.asarray(target_w, float); ori_w = np.asarray(ori_w, float)
        seed = np.asarray(seed, float)
        capable = self._ak_capable(frame, seed)
        if mode == "1" and not capable:
            self._ak_fallback = getattr(self, "_ak_fallback", 0) + 1
            if self._ak_fallback in (1, 50):
                self.say(f"[IK] ak 가 못 푸는 프레임 {frame} — Lula 로 되돌아간다 ({self._ak_fallback}번째)")
            mode = "0"
        q_l = ok_l = None
        if mode != "1":
            q_l, ok_l = self.lula.compute_inverse_kinematics(frame, target_w, ori_w, seed, 0.004, 0.05)
            if capable and os.environ.get("SIM_IK_DUMP"):
                self._dump_ik(frame, target_w, ori_w, seed, q_l, ok_l)
            if mode != "compare" or not capable:
                return q_l, ok_l
        import arm_kinematics as _ak
        p, Rm, tool = self._ak_target(frame, target_w, ori_w)
        r = _ak.ik(p, Rm, seed, pos_tol=0.004, rot_tol=0.05, min_margin=0.0, tool=tool)
        if mode == "1":
            return (np.asarray(r.q, float), True) if r.ok else (seed, False)
        # compare — 세기만 한다. 쓰는 해는 Lula 다. **단계별로** 센다 (2026-09-25 02:20 판에서
        # Lula만 105 가 100~500번 한 구간에 몰렸는데 어느 단계인지 못 셌다).
        ph = getattr(self, "_ik_phase", "?")
        st = self._ak_stats
        ps = st.setdefault("단계", {}).setdefault(ph, {"n": 0, "Lula만": 0, "다중도못": 0})
        st["n"] += 1; ps["n"] += 1
        if ok_l and r.ok:
            st["둘다"] += 1
            st["dq"] = max(st["dq"], float(np.max(np.abs(np.asarray(r.q, float) - np.asarray(q_l, float)))))
            try:
                pa = np.asarray(self.lula.compute_forward_kinematics(frame, np.asarray(r.q, float))[0], float)
                st["mm"] = max(st["mm"], float(np.linalg.norm(pa - target_w)) * 1000.0)
            except Exception:      # noqa: BLE001
                pass
        elif ok_l:
            # 이 씨앗 하나에서 ak 가 못 푼 것이다. 실제 `SIM_IK_AK=1` 은 ik_joints 가 씨앗을
            # 여럿 돌리므로, **여러 씨앗에서도 못 푸는지**를 따로 센다 — 그게 진짜 차이다
            st["Lula만"] += 1; ps["Lula만"] += 1
            if not _ak.rescue_seeds(p, Rm, seed, k=1, pos_tol=0.004, rot_tol=0.05, min_margin=0.0, tool=tool):
                st["다중도못"] = st.get("다중도못", 0) + 1; ps["다중도못"] += 1
        elif r.ok:
            st["ak만"] += 1
        else:
            st["둘다못"] += 1
        if st["n"] in (1, 20, 100, 500, 2000, 5000):
            by = " · ".join(f"{k} {v['n']}(Lula만 {v['Lula만']}, 다중도못 {v['다중도못']})"
                            for k, v in st["단계"].items())
            self.say(f"[IK대조] {st['n']}번: 둘다 {st['둘다']} · Lula만 {st['Lula만']} "
                     f"(다중 씨앗도 못 품 {st.get('다중도못', 0)}) · ak만 {st['ak만']} "
                     f"· 둘다못 {st['둘다못']} · 해 차이 최대 {st['dq']:.3f} rad · "
                     f"ak 해를 Lula FK 로 되돌린 손끝 오차 최대 {st['mm']:.2f} mm\n"
                     f"[IK대조]   단계별: {by}")
        return q_l, ok_l

    def ik_joints(self, target, ori, seed, frame=None):
        """IK 해를 구하되 **씨앗 자세에서 너무 먼 해는 버린다.**

        6축에서는 같은 손끝 자세에 팔을 통째로 뒤로 돌린 해가 같이 존재한다. 그 해를
        고르면 경로가 로봇 자신(받침판·카터)을 통과해 어깨가 막힌다
        (2026-09-20 M0609: joint_1 이 3.22 rad = 185° 로 나와 approach 에서 멈췄다).
        Lula 가 씨앗을 주어도 먼 해를 돌려주므로, 여기서 걸러 다시 시도한다.
        """
        seed = np.asarray(seed, float)
        lim = BOT.ik_seed_limit
        best = None
        # 홈을 마지막 씨앗으로 둔다 — 흔들기로 못 찾으면 팔꿈치↑ 홈에서 다시 푼다
        _seeds = [seed] + [seed + d for d in self._ik_nudges] + [np.asarray(self.q_home, float)]
        k = 0
        rescued = False
        while True:
            if k >= len(_seeds):
                # 씨앗을 다 썼는데 해가 없다 → **딱 한 번** arm_kinematics 해를 씨앗으로 붙인다
                if best is not None or rescued:
                    break
                rescued = True
                _seeds.extend(self._ak_rescue_seeds(frame or BOT.ee_frame, target, ori, seed))
                continue
            s0 = _seeds[k]
            k += 1
            # 풀이기는 `_solve_ik` 한 곳이 고른다 (Lula / arm_kinematics). 가드는 **아래에서
            # 풀이기와 무관하게** 걸린다 — 가드를 우회하는 경로가 생기면 스캔이 겪은 일이
            # 그대로 재발한다 (Lula 거짓 한계 j6 3.75 로 팔이 한계에 붙은 채 404, 2026-09-23).
            q, ok = self._solve_ik(frame or BOT.ee_frame, target, ori, s0)
            if not ok:
                continue
            q = np.asarray(q, float)
            q = self._unwrap_to(q, seed)
            if not self._branch_ok(q):
                continue          # 팔꿈치↓ 해는 받침판에 닿는다 — 조용히 쓰지 않는다
            # **실제 관절 한계를 벗어나는 해는 버린다.** Lula 의 한계가 더 넓어서
            # 도달 못 하는 해가 나온다 (`_arm_limits` 참조). 그대로 쓰면 팔이 한계에
            # 붙은 채 목표를 못 맞춰 `404 제한 시간 초과` 로 죽는다.
            _alo, _ahi = self._arm_limits()
            if _alo is not None:
                _n = min(len(q), len(_alo))
                if np.any(q[:_n] < _alo[:_n] - 1e-3) or np.any(q[:_n] > _ahi[:_n] + 1e-3):
                    self._lim_skip = getattr(self, "_lim_skip", 0) + 1
                    if self._lim_skip in (1, 50, 500):
                        _bad = [i for i in range(_n)
                                if q[i] < _alo[i] - 1e-3 or q[i] > _ahi[i] + 1e-3]
                        self.say(f"[IK] 관절 한계를 넘는 해를 버렸다 ({self._lim_skip}번째) "
                                 f"관절 {[i + 1 for i in _bad]} q="
                                 f"{np.round(q[:_n], 3).tolist()}")
                    continue
            # 받침판 검사는 **가드로만** 남긴다. 해를 버리면 IK 가 다른 가지로 튀고,
            # 그게 `down` 1.96 rad 점프의 정체였다 (웹 클로드 v21 회신 §Q2).
            # 팔꿈치↑ 에서는 발동하지 않아야 정상이다 — 발동하면 그 자체가 신호다.
            if not self._above_deck(q):
                self._deck_warn = getattr(self, "_deck_warn", 0) + 1
                if self._deck_warn in (1, 20, 200):
                    self.say(f"[받침판 경고] 팔꿈치↑ 인데 받침판에 닿는 해가 나왔다 "
                             f"({self._deck_warn}번째) q={np.round(q, 3).tolist()}")
            d = float(np.max(np.abs(q[:len(seed)] - seed)))
            if best is None or d < best[1]:
                best = (q, d)
            # **충분히 가까우면 거기서 멈춘다.** 아니면 남은 씨앗도 전부 풀어 본다.
            if d <= MAX_STEP:
                break
        if best is None:
            return np.asarray(seed, float), False
        # **가장 가까운 해를 고른다.** 예전에는 `lim` 안에 드는 **첫** 해를 그냥 썼는데,
        # `ik_seed_limit` 이 0 인 로봇(Franka)에서는 `lim <= 0` 이 참이라 **첫 씨앗의 해를
        # 무조건 반환**했다. 흔들기 씨앗을 시도조차 안 하니 가지가 바뀐 해가 그대로 나가고,
        # 접근 경로에 π/2(1.571 rad) 계단이 생겨 `401` 로 죽었다 (2026-09-22 실측).
        # 스캔 자세 계획에서 쓰는 방식과 같다 — 여러 씨앗을 풀고 **앞 자세와 가장 가까운 해**.
        if lim > 0 and best[1] > lim and os.environ.get("SIM_TRACE_IK", "0") != "0":
            self.say(f"[IK] 가장 가까운 해도 {best[1]:.3f} rad 떨어져 있다 (한계 {lim})")
        return best[0], True

    def plan_joint_path(self, waypoints, seed):
        """경유점의 **자세만 IK 로 풀고, 사이는 관절 공간에서 잇는다** (두산 `movej` 방식).

        왜 필요한가: 손목을 크게 돌리는 구간(`carry_rotate`)을 직교 보간하면 손끝이
        특이점을 지나며 IK 해가 튄다. 32배까지 잘게 나눠도 2.96 rad(180°) 점프가 남았다
        (2026-09-21 실측). 자유 공간 이동은 **손끝이 어떤 곡선을 그리든 상관없다** —
        관절 공간에서 이으면 특이점이 아예 문제가 되지 않는다.

        직선이 필요한 구간(꽂아 넣기·밀기)은 그대로 `plan_path` 를 쓴다.
        """
        qs = [np.asarray(seed, float)]
        for p_, q_ in waypoints[1:]:
            sol, ok = self.ik_joints(np.asarray(p_, float), np.asarray(q_, float), qs[-1])
            if not ok:
                return None, 0.0, f"IK 실패 {np.round(p_, 3).tolist()}"
            d = np.abs(sol - qs[-1])
            n = max(1, int(math.ceil(float(d.max()) / (MAX_STEP * 0.7))))
            base = qs[-1].copy()
            for i in range(1, n + 1):
                qs.append(base + (sol - base) * (i / n))
        worst = float(np.max(np.abs(np.diff(np.asarray(qs, float), axis=0)))) if len(qs) > 1 else 0.0
        return qs, worst, ""

    def plan_path(self, waypoints, seed, step_m=0.005, step_rad=0.035):
        """경유점들을 잇는 관절 경로. **한 걸음이 너무 크면 그 구간만 잘게 쪼갠다.**

        고정 간격(2°)으로 나누면 손목이 특이점 근처를 지날 때 한 걸음에 0.25 rad 씩
        움직여 `402` 가 났다 (2026-09-21 실측). 간격을 전부 줄이면 쉬운 구간까지 느려지므로,
        걸리는 구간만 두 배씩 잘게 나눠 다시 푼다.
        """
        MAX_SPLIT = 5                      # 2^5 = 32배까지 잘게 (그래도 안 되면 진짜 못 가는 것)
        qs = [np.asarray(seed, float)]
        prev_p = prev_q = None
        worst = 0.0
        for p_, q_ in waypoints:
            p_ = np.asarray(p_, float); q_ = np.asarray(q_, float)
            if prev_p is None:
                prev_p, prev_q = p_, q_
                continue
            n0 = max(1, int(math.ceil(np.linalg.norm(p_ - prev_p) / step_m)),
                     int(math.ceil(quat_angle(prev_q, q_) / step_rad)))
            for split in range(MAX_SPLIT + 1):
                n = n0 * (2 ** split)
                trial = [qs[-1]]
                ok_all = True
                big = 0.0
                for i in range(1, n + 1):
                    t = i / n
                    tp = prev_p + (p_ - prev_p) * t
                    tq = slerp(prev_q, q_, t)
                    sol, ok = self.ik_joints(tp, tq, trial[-1])
                    if not ok:
                        return None, worst, f"IK 실패 {np.round(tp, 3).tolist()}"
                    step = float(np.max(np.abs(sol - trial[-1])))
                    big = max(big, step)
                    trial.append(sol)
                    if step > MAX_STEP:
                        ok_all = False
                        # **어디서 어느 관절이 튀는지 남긴다.** "한 걸음이 1.56 rad" 만으로는
                        # 보간을 더 잘게 할지, 자세를 바꿔야 할지 가를 수 없다 (2026-09-22).
                        if split == MAX_SPLIT:
                            _j = int(np.argmax(np.abs(sol - trial[-1])))
                            self.say(f"[IK] 튐 지점: t={t:.3f} (경유 {i}/{n}) "
                                     f"**관절 {_j + 1}** {trial[-1][_j]:+.3f} → {sol[_j]:+.3f} "
                                     f"({sol[_j] - trial[-1][_j]:+.3f} rad)")
                            self.say(f"[IK]   앞 {np.round(trial[-1], 3).tolist()}")
                            self.say(f"[IK]   뒤 {np.round(sol, 3).tolist()}")
                            self.say(f"[IK]   목표점 {np.round(tp, 4).tolist()}")
                        break              # 더 잘게 나눠 다시 푼다 (마지막 단계면 실패한다)
                if ok_all:
                    if split and os.environ.get("SIM_TRACE_IK", "0") != "0":
                        self.say(f"[보간] 이 구간은 {2 ** split}배 잘게 나눠 풀었다 "
                                 f"(최대 걸음 {big:.3f} rad)")
                    qs.extend(trial[1:])
                    worst = max(worst, big)
                    break
            else:
                return None, worst, (f"{2 ** MAX_SPLIT}배까지 잘게 나눠도 한 걸음이 "
                                     f"{big:.3f} rad 이다 (손목 특이점으로 본다)")
            prev_p, prev_q = p_, q_
        return qs, worst, ""

    def book_on_tray_near(self, p_world, tol=None):
        """트레이 위에 있는 책 중 AABB 중심이 p_world 에서 tol 안인 것"""
        tol = BOT.tray_match_tol if tol is None else tol
        best, dist = None, tol
        for b in self.books:
            c = self.center(b)
            d = float(np.linalg.norm(c - p_world))
            if d <= dist and c[2] < DECK_Z + 0.25:
                best, dist = b, d
        return best, dist

    def _plan_carry_joint(self, lift, transfer, pre_ins, DOWN, HORIZ, q_lift):
        """관절 모드 `carry_rotate` — 경유점 `transfer` 가 안 풀리면 자세를 바꿔, 그래도 안 되면
        빼고 잇는다. 후보 순서와 까닭은 `arm_planning.carry_ladder` 에 있다. 첫 후보가 지금까지의
        경로라 아래 판은 한 점도 안 바뀐다.
        """
        ladder = carry_ladder(lift, transfer, pre_ins, DOWN, HORIZ)
        qs, worst, err, i, errs = plan_first(ladder, lambda w: self.plan_joint_path(w, q_lift))
        if qs is not None and i:
            self.say(f"[운반] transfer@DOWN 이 안 풀려 **{ladder[i][0]}** 로 잇는다 "
                     f"({i + 1}번째 후보) — 앞 후보: {' / '.join(errs)}")
        return qs, worst, err

    # ---------------------------------------------------------------- 스윙 운반 (SIM_CARRY_MODE=swing)
    # carry_rotate 를 관절공간 한 호로 잇으면 양 끝 자세가 멀어 손끝이 1 m 넘게 솟았다
    # (2026-09-22 저녁 영상). 구간을 "관절 1개 순수 회전" 과 "직교 직선" 으로 나눠
    # 손끝 궤적을 예측할 수 있게 한다: lift → (a) 수직 clear → (b) j1 만 스윙 →
    # (c) 직교 reach → (d) 짧은 reorient. 기본 꺼짐 — 켜지 않으면 계획이 한 점도 안 바뀐다.
    # **2026-09-22 야간: 권한 문제로 시뮬에서 한 번도 돌려 보지 못했다.** 아침에 시험할 것.
    def _joint_interp(self, q0, q1):
        """관절공간 직선 (q0 제외, q1 포함). 한 걸음은 plan_joint_path 와 같은 MAX_STEP×0.7"""
        q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
        n = max(1, int(math.ceil(float(np.max(np.abs(q1 - q0))) / (MAX_STEP * 0.7))))
        return [q0 + (q1 - q0) * (i / n) for i in range(1, n + 1)]

    def _swing_angle(self, p_from_w, p_to_w, q1_now):
        """j1 만 돌려 p_from 의 방위각을 p_to 의 방위각으로 맞추는 Δφ (rad).

        방위각은 팔 베이스(link0 원점, j1 축이 지나는 곳) 기준이다. 짧은 쪽을 먼저,
        한계(실제 한계 − 0.05 rad)에 걸리면 반대쪽을 본다. 둘 다 안 되면 (None, 이유).
        """
        a = self.yaw_to_arm(p_from_w); b = self.yaw_to_arm(p_to_w)
        d = math.atan2(float(b[1]), float(b[0])) - math.atan2(float(a[1]), float(a[0]))
        d = (d + math.pi) % (2 * math.pi) - math.pi
        cands = [d] if abs(d) < 1e-9 else [d, d - math.copysign(2 * math.pi, d)]
        lo, hi = self._arm_limits()
        for c in cands:
            if lo is None or (lo[0] + 0.05 <= q1_now + c <= hi[0] - 0.05):
                return c, ""
        return None, (f"j1 한계: q1={q1_now:+.3f} Δφ 후보 {[round(c, 3) for c in cands]} "
                      f"한계 {lo[0]:+.3f}~{hi[0]:+.3f}")

    def _plan_swing_carry(self, lift, transfer, pre_ins, DOWN, HORIZ, q_lift):
        """carry_rotate 를 네 조각으로 (NIGHTLY_PLAN 2-3). 돌려주는 값은 plan_path 와 같은 꼴."""
        q_lift = np.asarray(q_lift, float)
        qs = [q_lift.copy()]
        # a. clear(수직으로 먼저 올리기)는 **뺐다** (2026-09-25). SIM_SWING_CLEAR_M 0.15 는 c 를
        # 악화시켰고(0.126 → 0.168 rad) 0.60 은 a 자체가 아래 판부터 실패했다. 위 판 401 의
        # 정체는 높이가 아니라 경유점 자세였고(아래 c 주석), 스윙 끝의 문제도 높이가 아니라
        # 반지름이었다(되돌림 주석). 표도 높이와 무관하다고 했다 — 손잡이를 남길 이유가 없다.
        p_clear = np.asarray(lift, float)
        n_a = 0
        # b. swing — j1 만. 나머지 관절은 얼린다 → 손끝이 같은 높이·같은 반지름의 수평 호
        dphi, err = self._swing_angle(p_clear, transfer, float(qs[-1][0]))
        if dphi is None:
            return None, 0.0, f"스윙 b(j1): {err}"
        q_sw = qs[-1].copy(); q_sw[0] += dphi
        qs.extend(self._joint_interp(qs[-1], q_sw))
        n_b = len(qs) - 1 - n_a
        _p, _R = self.lula.compute_forward_kinematics(BOT.ee_frame, q_sw)
        p_sw = np.asarray(_p, float); O_sw = quat_from_R(np.asarray(_R, float))
        # c. reach — 스윙 끝의 손 자세(=DOWN 을 Δφ 만큼 돌린 것)를 유지한 채 transfer 로 직선.
        #
        # **위 판에서는 여기서 막힌다** (2026-09-24 실측). 관절공간 되돌림을 붙여 봤더니
        # 그쪽도 `IK 실패` 였다 — 불연속이 아니라 **`transfer` 자체가 안 풀린다.**
        # transfer 는 grip_z+0.06 으로 **경로에서 제일 높은 점**이라, 꽂는 자리(여유
        # 0.217 로 풀린다)보다 5 cm 더 위다. 못 가는 건 꽂는 자리가 아니라 경유점이다.
        # clear 로 올려도(0.15 → 걸음 0.168 로 더 나빠짐) 차체를 물려도(standoff 0.52)
        # 그대로였다.
        #
        # **c 와 d 를 합쳐 한 번에 가는 길(p_sw → pre_ins, 자세는 slerp)도 닫혔다.**
        # 2026-09-24 실측: 걸음 0.243 rad 로 **아래 판부터** 깨졌다 — d 단독의 0.249 와
        # 같은 값이다. 합치면 d 의 관절공간 되돌림이 사라져 손목 특이점을 넘길 수단이
        # 없어진다. 되던 것까지 망가뜨리므로 쓰지 않는다.
        part, _w, err = self.plan_path([(p_sw, O_sw), (np.asarray(transfer, float), O_sw)], qs[-1])
        q_b_end = np.asarray(q_sw, float).copy()
        pre_ins_arm = self.yaw_to_arm(np.asarray(pre_ins, float))
        if part is not None:
            qs.extend(part[1:])
            n_c = len(part) - 1
            how_c = "직교"
            q_c_path = [np.asarray(v, float).copy() for v in part]      # q_sw … q_transfer
            # d. reorient — transfer → pre_ins (손 자세 → HORIZ). 직교가 안 되면 관절공간
            how = "직교"
            wps_d = [(np.asarray(transfer, float), O_sw), (np.asarray(pre_ins, float), HORIZ)]
            part, _w, err = self.plan_path(wps_d, qs[-1])
            if part is None:
                part, _w, err2 = self.plan_joint_path(wps_d, qs[-1])
                if part is None:
                    return None, 0.0, f"스윙 d(reorient): 직교 {err} / 관절 {err2}"
                how = f"관절공간(직교 실패: {err})"
                self.say(f"[스윙] d(reorient) 직교 실패 → 관절공간으로 — {err}")
            qs.extend(part[1:])
            n_d = len(part) - 1
        else:
            # **되돌림 — 손목을 먼저 돌리고 HORIZ 로 올라간다. c 가 안 풀릴 때만.**
            #
            # 위 판에서 `transfer@O_sw` 는 존재하지 않는다: DOWN 계열 손 자세는 팔기준 z
            # 0.84~0.95 띠에 해가 없고 HORIZ 만 풀린다 (2026-09-25 00:00 오프라인 표,
            # `arm_planning.carry_ladder` 주석). 그래서 낮은 데(p_sw, 스윙 끝)에서 손목을
            # O_sw → HORIZ 로 **관절공간**으로 돌리고 — d 가 매 판 그렇게 손목 특이점을
            # 넘기고 있었다 — 거기서 pre_ins 까지 HORIZ 를 유지한 채 직선으로 간다.
            # transfer 는 안 들르고 d 는 필요 없다(이미 HORIZ 로 도착한다).
            # c 가 풀리는 판(아래 판)은 이 분기에 오지 않는다 — 한 점도 안 바뀐다.
            # **돌리는 자리는 p_sw 가 아니다 — 먼저 바깥으로 뻗는다.** 2026-09-25 00:50 실측:
            # p_sw 가 팔 기준 반지름 0.284(트레이 위 lift 의 반지름이 그대로다)라 거기엔
            # HORIZ 해가 없다 — 손을 수평으로 하면 플랜지가 손끝보다 10 cm 뒤라 베이스
            # 기둥 위에 온다. 오프라인 표(방위각 157.6°, z 0.339, @HORIZ): r 0.284 ✗ ·
            # 0.34 → 0.143 · 0.40 → 0.320 · 0.46 → 0.504 · 0.52 → 0.695. 그리고 돌리는
            # 자리의 y 가 0.25 를 넘으면 매달린 책(0.19 m)이 서가 앞면(0.444)을 쓸고
            # 들어간다 — 그래서 pre_ins 자리에서 돌리지 않고, 같은 방위각으로 r 0.50
            # (y 0.19, 책 끝 0.38)까지만 뻗는다.
            p_sw_arm = self.to_arm(p_sw)
            r_sw = float(math.hypot(p_sw_arm[0], p_sw_arm[1]))
            phi = math.atan2(float(p_sw_arm[1]), float(p_sw_arm[0]))
            q_c_path = []
            n_o = 0
            p_out = p_sw
            if r_sw < SWING_TURN_R - 1e-3:
                p_out = self.to_world([SWING_TURN_R * math.cos(phi), SWING_TURN_R * math.sin(phi),
                                       float(p_sw_arm[2])])
                part, _w, err_o = self.plan_path([(p_sw, O_sw), (p_out, O_sw)], qs[-1])
                if part is None:
                    return None, 0.0, f"스윙 c(reach): 직교 {err} / 바깥으로 r {r_sw:.3f}→{SWING_TURN_R}: {err_o}"
                n_o = len(part) - 1
                q_c_path.extend(np.asarray(v, float).copy() for v in part)
                qs.extend(part[1:])
            # 손목 돌리기 — d 와 같은 순서: 직교(자세만 slerp) → 안 되면 관절공간
            wps_r = [(p_out, O_sw), (p_out, HORIZ)]
            part, _w, err_r = self.plan_path(wps_r, qs[-1])
            how_r = "직교"
            if part is None:
                part, _w, err_r2 = self.plan_joint_path(wps_r, qs[-1])
                if part is None:
                    return None, 0.0, (f"스윙 c(reach): 직교 {err} / 손목(r {SWING_TURN_R}): "
                                       f"직교 {err_r} / 관절 {err_r2}")
                how_r = f"관절(직교 실패: {err_r})"
            n_r = len(part) - 1
            q_c_path.extend(np.asarray(v, float).copy() for v in (part if not q_c_path else part[1:]))
            qs.extend(part[1:])
            part, _w, err_l = self.plan_path([(p_out, HORIZ), (np.asarray(pre_ins, float), HORIZ)], qs[-1])
            if part is None:
                return None, 0.0, f"스윙 c(reach): 직교 {err} / 손목 뒤 HORIZ 직선: {err_l}"
            q_c_path.extend(np.asarray(v, float).copy() for v in part[1:])   # … q_pre_ins
            qs.extend(part[1:])
            n_c = len(part) - 1
            n_d = 0
            how = "없음"
            how_c = (f"되돌림: 바깥 r {r_sw:.3f}→{SWING_TURN_R} ({n_o}점) → 손목 {n_r}점[{how_r}] "
                     f"→ HORIZ 직선 (직교 실패: {err})")
            self.say(f"[스윙] c(reach) 직교 실패 → 바깥으로 뻗어 손목 돌리고 HORIZ 로 올라간다 — {err}")
        self._swing = {"dphi": dphi, "p_sw": p_sw, "O_sw": O_sw,
                       "q_b_start": np.asarray(qs[n_a], float).copy(), "q_b_end": q_b_end,
                       "q_c_path": q_c_path}       # q_sw … 운반 끝. return 이 거꾸로 되짚는다
        arr = np.asarray(qs, float)
        worst = float(np.max(np.abs(np.diff(arr, axis=0)))) if len(arr) > 1 else 0.0
        self.say(f"[스윙] carry_rotate: b j1 Δφ "
                 f"{math.degrees(dphi):+.1f}° ({n_b}점) · c reach {n_c}점 [{how_c}] · d reorient "
                 f"{n_d}점 [{how}] · 최대걸음 {worst:.3f} rad · 스윙 끝 손끝(월드) "
                 f"{np.round(p_sw, 3).tolist()} · pre_ins(팔기준 xy·월드 z) {np.round(pre_ins_arm, 4).tolist()}")
        return qs, worst, ""

    def _plan_swing_return(self, retreat, HORIZ, q_retreat, transfer, lift, DOWN, q_lift):
        """return 을 스윙의 거울로: retreat → 역 reorient → 역 reach → 역 swing → home"""
        sw = getattr(self, "_swing", None)
        if sw is None:
            # carry 가 스윙이 아니었다 — 같은 값을 여기서 만든다 (j1 축 둘레로 lift 를 돌린 점)
            dphi, err = self._swing_angle(lift, transfer, float(np.asarray(q_lift, float)[0]))
            if dphi is None:
                return None, 0.0, f"역스윙 b(j1): {err}"
            q_sw = np.asarray(q_lift, float).copy(); q_sw[0] += dphi
            _p, _R = self.lula.compute_forward_kinematics(BOT.ee_frame, q_sw)
            sw = {"dphi": dphi, "p_sw": np.asarray(_p, float),
                  "O_sw": quat_from_R(np.asarray(_R, float))}
        dphi, p_sw, O_sw = sw["dphi"], sw["p_sw"], sw["O_sw"]
        if "q_c_path" in sw and os.environ.get("SIM_SWING_MIRROR", "1") != "0":
            # 운반 때 지나간 관절값을 그대로 거슬러 간다. 역 d 를 IK 로 다시 풀면 다른 가지로
            # 떨어져 q1 이 +0.93 rad 어긋나고 역 swing 이 j1 한계를 넘었다 (v08, 2026-09-23 00:4x)
            q_tr = sw["q_c_path"][-1]
            qs = [np.asarray(q_retreat, float).copy()]
            qs.extend(self._joint_interp(qs[-1], q_tr))          # 역 d (관절, retreat → transfer)
            d_d = float(np.max(np.abs(q_tr - qs[0])))
            qs.extend([q.copy() for q in reversed(sw["q_c_path"][:-1])])   # 역 c (운반 c 그대로)
            qs.extend(self._joint_interp(qs[-1], sw["q_b_start"]))         # 역 b (j1 만)
            d_home = np.abs(np.asarray(self.q_home, float) - sw["q_b_start"])
            qs.extend(self._joint_interp(sw["q_b_start"], self.q_home))
            arr = np.asarray(qs, float)
            worst = float(np.max(np.abs(np.diff(arr, axis=0)))) if len(arr) > 1 else 0.0
            self.say(f"[스윙] return(거울): 역 d 관절 Δ최대 {d_d:.3f} rad · 역 c {len(sw['q_c_path']) - 1}점 · "
                     f"역 b j1 {math.degrees(-dphi):+.1f}° · 홈까지 관절 Δ 최대 {float(d_home.max()):.3f} rad "
                     f"(관절 {int(np.argmax(d_home)) + 1}) · 최대걸음 {worst:.3f} rad")
            return qs, worst, ""
        qs = [np.asarray(q_retreat, float).copy()]
        # 역 d — retreat(HORIZ) → transfer(스윙 손 자세)
        how = "직교"
        wps_d = [(np.asarray(retreat, float), HORIZ), (np.asarray(transfer, float), O_sw)]
        part, _w, err = self.plan_path(wps_d, qs[-1])
        if part is None:
            part, _w, err2 = self.plan_joint_path(wps_d, qs[-1])
            if part is None:
                return None, 0.0, f"역스윙 d(reorient): 직교 {err} / 관절 {err2}"
            how = f"관절공간(직교 실패: {err})"
            self.say(f"[스윙] 역 d(reorient) 직교 실패 → 관절공간으로 — {err}")
        qs.extend(part[1:])
        # 역 c — transfer → 스윙 끝점 (자세 유지, 직선)
        part, _w, err = self.plan_path([(np.asarray(transfer, float), O_sw), (p_sw, O_sw)], qs[-1])
        if part is None:
            return None, 0.0, f"역스윙 c(reach): {err}"
        qs.extend(part[1:])
        # 역 b — j1 만 −Δφ
        q_b = qs[-1].copy(); q_b[0] -= dphi
        lo, hi = self._arm_limits()
        if lo is not None and not (lo[0] + 0.05 <= q_b[0] <= hi[0] - 0.05):
            return None, 0.0, f"역스윙 b(j1): q1 {q_b[0]:+.3f} 가 한계 {lo[0]:+.3f}~{hi[0]:+.3f} 밖"
        qs.extend(self._joint_interp(qs[-1], q_b))
        # 마지막 — 홈까지 관절공간 (트레이 위에서 트레이 위로: 짧아야 한다. 크면 로그로 보인다)
        d_home = np.abs(np.asarray(self.q_home, float) - q_b)
        qs.extend(self._joint_interp(q_b, self.q_home))
        arr = np.asarray(qs, float)
        worst = float(np.max(np.abs(np.diff(arr, axis=0)))) if len(arr) > 1 else 0.0
        self.say(f"[스윙] return: 역 d [{how}] · 역 b j1 {math.degrees(-dphi):+.1f}° · 홈까지 관절 Δ "
                 f"최대 {float(d_home.max()):.3f} rad (관절 {int(np.argmax(d_home)) + 1}) · "
                 f"최대걸음 {worst:.3f} rad")
        return qs, worst, ""

    def _audit_plan(self, segs, joint_segs):
        """계획의 모든 구간을 FK 표본으로 본다 (SIM_TRACE_CARRY / SIM_PATH_AUDIT).

        구간마다 한 줄: z최대(월드·팔기준, 위치 t) · 한계 최소여유(관절) · 최대걸음(위치).
        SIM_PATH_AUDIT=1 이면 "z최대 > 양 끝 z 최대 + SIM_AUDIT_DZ(0.15)" 또는 "한계 여유 < 0.05"
        를 위반으로 돌려준다. 트레이·서가 여유 검사는 **아직 없다** (AABB 가 필요, 2-2 남은 일).
        돌려주는 값: 위반 메시지 (없으면 "")
        """
        dz_lim = float(os.environ.get("SIM_AUDIT_DZ", "0.15"))
        lo, hi = self._arm_limits()
        viol = ""
        for name in ["approach", "down", "lift", "carry_rotate", "wedge", "back",
                     "touch", "push", "retreat", "return"]:
            qs = segs.get(name)
            if not qs or len(qs) < 2:
                continue
            arr = np.asarray(qs, float)
            idx = sorted(set(np.linspace(0, len(arr) - 1, min(len(arr), 41)).astype(int).tolist()))
            zs = []
            for i in idx:
                try:
                    zs.append(float(np.asarray(
                        self.lula.compute_forward_kinematics(BOT.ee_frame, arr[i])[0], float)[2]))
                except Exception:      # noqa: BLE001 — 계측이 계획을 막으면 안 된다
                    zs.append(float("nan"))
            zs = np.asarray(zs, float)
            k = int(np.nanargmax(zs)); z_end = max(zs[0], zs[-1])
            steps = np.max(np.abs(np.diff(arr, axis=0)), axis=1)
            ks = int(np.argmax(steps))
            js = int(np.argmax(np.abs(arr[ks + 1] - arr[ks])))
            if lo is not None:
                n = min(arr.shape[1], len(lo))
                marg = np.minimum(arr[:, :n] - lo[:n], hi[:n] - arr[:, :n])
                mi = np.unravel_index(int(np.argmin(marg)), marg.shape)
                m_txt = f"{float(marg[mi]):.3f} rad (관절 {mi[1] + 1}, t={mi[0] / (len(arr) - 1):.2f})"
                m_val = float(marg[mi])
            else:
                m_txt, m_val = "한계 모름", float("inf")
            self.say(f"[경로검사] {name}{'(관절)' if name in joint_segs else ''} · z최대 "
                     f"{zs[k]:.3f} 월드 / {zs[k] - float(self.l0p[2]):.3f} 팔기준 (t={idx[k] / (len(arr) - 1):.2f}, "
                     f"양끝 최대 {z_end:.3f}, 초과 {zs[k] - z_end:+.3f}) · 한계 최소여유 {m_txt} · "
                     f"최대걸음 {float(steps[ks]):.3f} rad (관절 {js + 1}, 점 {ks + 1}/{len(arr) - 1})")
            if not viol and zs[k] > z_end + dz_lim:
                viol = (f"경로검사 손끝 높이: {name} 점 {idx[k]}/{len(arr) - 1} z {zs[k]:.3f} > "
                        f"양끝 {z_end:.3f} + {dz_lim:.2f}")
            if not viol and m_val < 0.05:
                viol = f"경로검사 한계 여유: {name} {m_txt} < 0.05"
        if os.environ.get("SIM_TRACE_CARRY", "0") != "0" and segs.get("carry_rotate"):
            qa = np.asarray(segs["carry_rotate"][0], float); qb = np.asarray(segs["carry_rotate"][-1], float)
            self.say(f"[운반] q_lift {np.round(qa, 3).tolist()} → q_pre_ins {np.round(qb, 3).tolist()} "
                     f"Δ {np.round(qb - qa, 3).tolist()}")
        return viol

    # ---------------------------------------------------------------- 계획
    def _arm_pt(self, p_world):
        """`yaw_to_arm` 과 같되 **z 도 베이스 기준으로 내린다** — 덤프 전용.

        `yaw_to_arm` 이 z 를 월드로 두는 데는 이유가 있다(삽입 높이에 베이스 기울기가
        새지 않게). 다만 그 값을 `_arm` 이라 부르면 `to_arm` 과 규약이 갈려서, 두 값을
        그대로 빼는 사람이 나온다. 덤프에서는 이름과 내용을 맞춘다.
        """
        q = self.yaw_to_arm(p_world)
        return np.array([q[0], q[1], q[2] - float(self.l0p[2])])

    def plan_inputs(self, book, place_center_world):
        """`plan_job` 의 결과를 **완전히 결정하는 것 전부**. 두 판을 견줄 때 쓴다.

        왜: 2026-09-24 에 "같은 조합" 이라고 믿은 스윕과 실제 판이 다른 답을 냈고,
        무엇이 달랐는지 찾는 데 몇 시간을 썼다. `plan_job` 은 결정적이다 — 아래 값이
        같으면 결과가 같아야 한다. 그러니 **다르면 아래 어딘가가 다르다.**
        추측하지 말고 두 벌을 찍어 한 줄씩 견준다.
        """
        bb = self.aabb(book)
        bp, bq = SingleXFormPrim(book).get_world_pose()
        out = {
            "book": book,
            "book_pos_world": np.round(np.asarray(bp, float), 6).tolist(),
            "book_quat_world": np.round(np.asarray(bq, float), 6).tolist(),
            # **z 도 베이스 기준으로 내린다.** `yaw_to_arm` 은 x·y 만 원점을 옮기고
            # z 는 월드 그대로 돌려준다(높이에 베이스 기울기가 새지 않게 한 것이다).
            # 그런데 그 값을 `_arm` 이라 부르면 `to_arm`(z 까지 내린다)과 규약이 갈리고,
            # 2026-09-24 에 그것 때문에 양쪽이 각각 한 번씩 틀린 비교를 했다.
            # **이름이 `_arm` 이면 z 도 팔 기준이어야 한다.**
            "book_center_arm": np.round(self._arm_pt((bb[:3] + bb[3:]) / 2), 6).tolist(),
            "book_top_arm": np.round(self._arm_pt(
                [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, bb[5]]), 6).tolist(),
            "arm_base_world": np.round(np.asarray(self.l0p, float), 6).tolist(),
            "arm_yaw_deg": round(math.degrees(self.tray_yaw), 4),
            "q_home": np.round(np.asarray(self.q_home, float), 6).tolist(),
            "place_center_world": np.round(np.asarray(place_center_world, float), 6).tolist(),
            "place_center_arm": np.round(self._arm_pt(place_center_world), 6).tolist(),
            "shelf_floor_z": round(float(self.shelf_floor_z), 6),
            "dims": [round(float(v), 6) for v in self.dims.get(book, (self.T, self.L, self.W))],
            "pre_lift_m": float(self.conf["grasp"].get("pre_lift_m", 0.13)),
            "carry_lift_m": float(self.conf["grasp"].get("carry_lift_m", 0.17)),
            "env": {k: os.environ.get(k, "") for k in (
                "SIM_CARRY_MODE", "SIM_RETURN_MODE", "SIM_JOINT_SEGS",
                "SIM_MAX_STEP", "SIM_HOME_Q", "SIM_GRIP_ROT90", "SIM_GRASP_KINEMATIC")},
        }
        if book in self.grasp_local:
            c_loc, up_loc, hz = self.grasp_local[book]
            out["grasp_local_c"] = np.round(np.asarray(c_loc, float), 6).tolist()
            out["grasp_local_up"] = np.round(np.asarray(up_loc, float), 6).tolist()
            out["grasp_local_hz"] = round(float(hz), 6)
        # **책이 트레이에서 이미 돌아 있는가.** 파지 자세는 상수(DOWN)라 책의 yaw 를
        # 맞춰 주지 않는다 — 트레이에서 돌아 있으면 그대로 들려가 그대로 꽂힌다.
        # 2026-09-24: 통과한 데모 판도 꽂힌 x 폭이 43.0 mm(기울기 2.8°)였고, 운반 중
        # 회전은 0.2° 이하였다. 즉 **운반에서 생기는 각이 아니다.**
        # span_pred 가 실제로 꽂힌 폭과 맞으면 원인이 여기로 확정된다.
        _q = np.asarray(bq, float)
        _byaw = math.degrees(math.atan2(2 * (_q[0] * _q[3] + _q[1] * _q[2]),
                                        1 - 2 * (_q[2] ** 2 + _q[3] ** 2)))
        _rel = _byaw - math.degrees(self.tray_yaw)
        # **직각 배수에서 얼마나 벗어났는가**로 잰다. 트레이에 선 책과 서가에 꽂힌 책은
        # 90° 돌아 있어서, 그냥 빼면 89.68° 같은 값이 나온다 (2026-09-24 실측).
        # 우리가 알고 싶은 것은 그 89.68 이 아니라 **−0.32** 다.
        _skew = (_rel + 45.0) % 90.0 - 45.0
        out["book_yaw_world_deg"] = round(float(_byaw), 4)
        out["book_yaw_arm_deg"] = round(float(_rel), 4)
        out["book_skew_deg"] = round(float(_skew), 4)      # ← 이 값을 본다
        try:
            from shelf_gap import span_for
            _T, _Lb, _W = out["dims"]
            out["span_pred_mm"] = round(span_for(_T, _W, abs(_skew)) * 1000, 2)
            out["span_flat_mm"] = round(float(_T) * 1000, 2)
        except Exception:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
            pass
        # **트레이 책에 콜리전이 붙어 있는가.** 레벨 트레이를 쓰면 위 561~564 의
        # 콜리전 부여 루프가 통째로 건너뛴다 (`range(0 if self.use_level_tray ...)`).
        # 그러면 손가락이 책을 뚫고 지나가고, 마찰 파지는 쥘 대상이 없다.
        try:
            _nc = sum(1 for _m in Usd.PrimRange(self.stage.GetPrimAtPath(book))
                      if _m.HasAPI(UsdPhysics.CollisionAPI))
            _nr = sum(1 for _m in Usd.PrimRange(self.stage.GetPrimAtPath(book))
                      if _m.HasAPI(UsdPhysics.RigidBodyAPI))
            out["book_collision_prims"] = _nc
            out["book_rigidbody_prims"] = _nr
            if _nc == 0:
                out["book_collision_warning"] = (
                    "콜리전 없음 — 손가락이 뚫고 지나간다. 마찰 파지는 성립하지 않고 "
                    "헛쥠 판정도 무의미하다 (레벨 트레이를 쓰면 콜리전 부여를 건너뛴다)")
        except Exception:      # noqa: BLE001
            pass
        return out

    def say_plan_inputs(self, book, place_center_world, tag=""):
        """`plan_inputs` 를 한 줄씩 찍는다. `SIM_PLAN_INPUTS=<경로>` 면 JSON 으로도 쓴다."""
        d = self.plan_inputs(book, place_center_world)
        self.say(f"[계획입력]{(' ' + tag) if tag else ''} — 두 판을 견줄 때 이 블록을 통째로 비교할 것")
        for k, v in d.items():
            self.say(f"    {k:<20} {v}")
        _path = os.environ.get("SIM_PLAN_INPUTS", "").strip()
        if _path:
            try:
                import json
                with open(_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps({"tag": tag, **d}, ensure_ascii=False) + "\n")
            except Exception as exc:      # noqa: BLE001 - 기록이 작업을 막으면 안 된다
                self.say(f"[계획입력] 파일로 못 썼다 {_path}: {type(exc).__name__}: {exc}")
        return d

    def plan_job(self, book, place_center_world):
        """책 하나를 집어, 꽂힌 뒤 AABB 중심이 place_center_world 가 되도록 꽂는 경로. 홈 → 홈"""
        moved = self.refresh_base()
        if moved > 0.01:
            self.say(f"팔 베이스가 {moved*100:.1f}cm 움직였다 — IK 기준을 다시 잡았다")
        bb = self.aabb(book); bc = (bb[:3] + bb[3:]) / 2
        T, Lb, W = self.dims.get(book, (self.T, self.L, self.W))
        DOWN, HORIZ = self.DOWN, self.HORIZ
        # **손을 책에 맞춰 돌려서 잡는다** (`SIM_GRASP_ALIGN`, 기본 꺼짐).
        #
        # 왜: 트레이의 책은 2.4° 쯤 기운 채 서 있고, 파지 자세가 상수 DOWN 이라
        # 기운 채로 들려 기운 채로 꽂힌다. 그러면 칸 방향 AABB 가 두께 35.2 대신
        # 43.0 mm 가 되어(지렛대가 책 높이 227 mm 다) 이웃과의 여유를 한쪽당 3.9 mm
        # 먹는다. 2026-09-24 실측에서 통과한 판의 남은 여유가 5.4 mm 였다.
        #
        # **책을 돌려 고치는 길은 이미 실패했다.** 물고 있는 책을 돌리면 그 회전이
        # 손 안의 어긋남이 되어 406 이 났다 (2026-09-22, 3.9 cm). 그래서 `attach()` 는
        # 25° 미만이면 그냥 잡는다. 여기서는 **잡기 전에 손을 돌린다** — 잡은 뒤가
        # 아니므로 406 을 내지 않는다. 잡고 나면 책은 손 기준으로 반듯하고,
        # carry_rotate 가 손을 HORIZ(서가 방향)로 가져가므로 반듯하게 꽂힌다.
        #
        # 기본을 끈 이유: 파지 자세가 2.4° 바뀌면 IK 가지가 갈릴 수 있다. 이 팔은
        # 1 mm 에도 갈린다. 5회 통과한 기준선을 흔들지 않고 A/B 로 견준다.
        _align = 0.0
        if os.environ.get("SIM_GRASP_ALIGN", "0") != "0" and book in self.upright_q:
            _bq = SingleXFormPrim(book).get_world_pose()[1]
            # **기준 자세는 `attach()` 와 같은 자로 잰다.** `upright_q` 는 트레이가
            # 돌기 전의 월드 자세라, 레벨 트레이처럼 돌아 있으면 90° 가 그대로 남는다
            # (2026-09-24 A/B 1차: 2.0° 짜리 기울기가 90.1° 로 읽혀 스위치가 안 걸렸다).
            # `attach()` 는 트레이 자세로 다시 만들어 쓴다 — 여기서도 그래야 같은 값이 나온다.
            _up_w = np.asarray(self.upright_q[book], float)
            if book in getattr(self, "upright_rel", {}):
                _Rt = R_from_quat(np.asarray(
                    SingleXFormPrim(self.tray).get_world_pose()[1], float))
                _up_w = quat_from_R(_Rt @ self.upright_rel[book])
            _tilt = quat_angle(np.asarray(_bq, float), _up_w)
            if _tilt >= UPRIGHT_SNAP_RAD:
                self.say(f"[파지정렬] 책이 {math.degrees(_tilt):.1f}° 기울어 문턱"
                         f"({math.degrees(UPRIGHT_SNAP_RAD):.0f}°)을 넘는다 — 손을 맞추지 않는다. "
                         f"attach() 의 세우기에 맡긴다")
            elif _tilt > math.radians(0.2):
                _dR = (R_from_quat(np.asarray(_bq, float))
                       @ R_from_quat(_up_w).T)
                DOWN = quat_from_R(_dR @ R_from_quat(DOWN))
                _align = float(_tilt)
                self.say(f"[파지정렬] 책이 {math.degrees(_tilt):.2f}° 기울어 있다 — "
                         f"**손을 그만큼 돌려서 잡는다** [SIM_GRASP_ALIGN]. "
                         f"잡은 뒤가 아니라 잡기 전이라 406 을 내지 않는다")
        self.fit_bookends(float(place_center_world[0]), T)
        # **여기서부터 경유점은 팔이 보는 방향 기준으로 조립한다** (x 칸 방향, y 서가 방향).
        # 예전에는 월드 x/y 를 직접 썼고, 그래서 로봇이 돌아서면 경로가 통째로 깨졌다.
        # yaw 0 이면 예전 값과 완전히 같다 (원점만 옮긴 것이라 왕복하면 그대로 돌아온다).
        _pc = self.yaw_to_arm(place_center_world)
        place_x = float(_pc[0])
        y_front = float(_pc[1]) - W / 2 - MEASURED_INSET       # 꽂힌 책 중심 → 서가 앞면
        # 꽂힌 책 중심 → 칸 바닥. 목표 z 는 표준 책 높이를 가정하고 오므로, 책 높이가 다르면
        # 그대로 쓰면 책이 칸 바닥에서 뜨거나 파묻힌다. 같은 칸으로 볼 수 있으면 실제 칸 바닥에 맞춘다.
        floor_z = self.snap_floor(float(_pc[2]) - Lb / 2)
        spine_final = y_front + SPINE_INSET
        # **그림자 — 이웃 책 앞끝과 견준다.** 서가 B 는 책이 앞면에서 34 mm 뒤(A 는 5.3 mm)라 같은
        # SPINE_INSET 으로 꽂으면 이웃보다 29 mm 튀어나온다 (2026-09-25 데스크탑 실측). 아직 값은 안
        # 바꾼다 — 그 판의 이웃 앞끝 중앙값과 계획값의 차이만 찍어, 서가 A 회귀 판에서 이 차이가
        # A 실측(≈ +14.7 mm, 계획이 이웃보다 안쪽)과 같은지 먼저 본다. 바꾸는 건 그다음이다.
        try:
            _fronts = []
            for _path, _bb in self.shelf_book_boxes(board_z=floor_z):
                _fa = self.yaw_to_arm([float(_bb[0]), float(_bb[1]), float(_bb[2])])
                _fb = self.yaw_to_arm([float(_bb[3]), float(_bb[4]), float(_bb[5])])
                _fronts.append(min(float(_fa[1]), float(_fb[1])))      # 팔 기준 y 가 작은 쪽 = 앞끝
            if _fronts:
                _med = float(np.median(_fronts))
                self.say(f"[책등] 계획 spine_final(팔기준 y) {spine_final:+.4f} · 이웃 {len(_fronts)}권 앞끝 중앙값 "
                         f"{_med:+.4f} · 계획이 이웃보다 {(spine_final - _med) * 1000:+.1f} mm 안쪽  (그림자 — 값은 그대로)")
        except Exception as _exc:      # noqa: BLE001 - 계측이 계획을 막으면 안 된다
            self.say(f"[책등] 이웃 앞끝을 못 쟀다: {type(_exc).__name__}: {_exc}")
        grasp = self.yaw_to_arm([bc[0], bc[1], bb[5] - TIP_DOWN])
        if book in self.grasp_local:
            c_loc, up_loc, hz = self.grasp_local[book]
            p_w, q_w = SingleXFormPrim(book).get_world_pose()
            top = np.asarray(p_w, float) + R_from_quat(np.asarray(q_w, float)) @ (c_loc + up_loc * hz)
            # 책 자신의 윗면 중심에서 내려간다 (월드로 구한 뒤 팔 기준으로)
            grasp = self.yaw_to_arm([top[0], top[1], top[2] - TIP_DOWN])
        # 파지 전 대기 높이와 들어올림 높이. 예전에는 0.13/0.17 이 코드에 박혀 있었는데,
        # 6축은 그 높이에서 IK 가 안 풀린다 (2026-09-20 M0609 에서 접근 IK 실패) → 설정으로 뺀다.
        _pre_h = float(self.conf["grasp"].get("pre_lift_m", 0.13))
        _lift_h = float(self.conf["grasp"].get("carry_lift_m", 0.17))
        pre = grasp + np.array([0, 0, _pre_h]); lift = grasp + np.array([0, 0, _lift_h])
        grip_z = floor_z + Lb / 2 + 0.004
        y_pre = y_front - (W - TIP_DOWN) - 0.03
        transfer = np.array([place_x, y_pre - 0.02, grip_z + 0.06]); pre_ins = np.array([place_x, y_pre, grip_z + 0.01])
        wedge = np.array([place_x, y_front + 0.10 - (W - TIP_DOWN), grip_z])
        back = wedge - np.array([0, TIP_DOWN + 0.025, 0])
        push_z = floor_z + Lb / 2
        touch = np.array([place_x, wedge[1] - TIP_DOWN - 0.008, push_z])
        push = np.array([place_x, spine_final + 0.002 - 0.010, push_z])
        retreat = np.array([place_x, y_front - 0.13, push_z])
        # **여기서 한 번에 월드로 바꾼다.** IK 는 월드 목표를 받는다
        _w = self.yaw_to_world
        pre, grasp, lift = _w(pre), _w(grasp), _w(lift)
        transfer, pre_ins, wedge = _w(transfer), _w(pre_ins), _w(wedge)
        back, touch, push, retreat = _w(back), _w(touch), _w(push), _w(retreat)
        spine_final_w = float(_w([place_x, spine_final, 0.0])[1])
        place_x_w = float(place_center_world[0])
        # 홈 경유점은 **홈 자신의 손 자세**를 쓴다. DOWN 을 쓰면 관절각과 모순이라 큰 점프가 난다
        HOME_O = getattr(self, "HOME_ORI", DOWN)
        order = [("approach", [(self.home_tip, HOME_O), (pre, DOWN)]), ("down", [(pre, DOWN), (grasp, DOWN)]),
                 ("lift", [(grasp, DOWN), (lift, DOWN)]),
                 ("carry_rotate", [(lift, DOWN), (transfer, DOWN), (pre_ins, HORIZ)]),
                 ("wedge", [(pre_ins, HORIZ), (wedge, HORIZ)]), ("back", [(wedge, HORIZ), (back, HORIZ)]),
                 ("touch", [(back, HORIZ), (touch, HORIZ)]), ("push", [(touch, HORIZ), (push, HORIZ)]),
                 ("retreat", [(push, HORIZ), (retreat, HORIZ)]), ("return", [(retreat, HORIZ), (self.home_tip, HOME_O)])]
        # **자유 공간 이동은 관절 공간으로 잇는다** (movej). 직선이 필요한 구간만 직교 보간(movel).
        # 꽂아 넣는 동작은 책이 칸 벽을 따라 들어가야 하므로 직선이어야 한다.
        # approach 는 직교 보간이 검증돼 있다. 관절 공간으로 바꿨더니 팔이 넓게 휘둘러
        # 실행에서 `404 제한 시간 초과` 가 났다 (2026-09-21 실측). 문제였던 구간만 바꾼다.
        JOINT_SEGS = set(os.environ.get(
            "SIM_JOINT_SEGS", "carry_rotate,return").replace(" ", "").split(","))
        # 운반·복귀를 스윙으로 (SIM_CARRY_MODE / SIM_RETURN_MODE = swing). 기본은 지금 동작
        CARRY_MODE = os.environ.get("SIM_CARRY_MODE", "joint").strip()
        RETURN_MODE = os.environ.get("SIM_RETURN_MODE", "joint").strip()
        self._swing = None
        segs = {}; q = self.q_home
        for name, wps in order:
            self._ik_phase = name          # [IK대조] 단계별 집계용
            if name == "carry_rotate" and CARRY_MODE == "swing":
                qs, worst, err = self._plan_swing_carry(lift, transfer, pre_ins, DOWN, HORIZ, q)
            elif name == "return" and RETURN_MODE == "swing":
                qs, worst, err = self._plan_swing_return(retreat, HORIZ, q, transfer, lift, DOWN,
                                                         segs["lift"][-1])
            elif name == "carry_rotate" and name in JOINT_SEGS:
                qs, worst, err = self._plan_carry_joint(lift, transfer, pre_ins, DOWN, HORIZ, q)
            elif name in JOINT_SEGS:
                qs, worst, err = self.plan_joint_path(wps, q)
            else:
                qs, worst, err = self.plan_path(wps, q)
            if qs is None:
                return None, 401, f"{name}: {err}"
            segs[name] = qs; q = qs[-1]
        # **연속성 검사는 감김을 푼 뒤에 한다.** 먼저 검사하면 2π 감김(실제로는 안 움직임)이
        # 402 로 잡혀 여기까지 오지도 못한다 (2026-09-21: joint_6 6.03 rad 로 5/5 실패)
        segs, shifted = self._unwind_plan(segs)
        worst_all = 0.0
        for name, qs in segs.items():
            arr = np.asarray(qs, float)
            if len(arr) < 2:
                continue
            w = float(np.max(np.abs(np.diff(arr, axis=0))))
            if w > MAX_STEP:
                return None, 402, f"{name}: 인접점 관절변화 {w:.3f} rad"
            worst_all = max(worst_all, w)
        # 경로 검사 (SIM_TRACE_CARRY=1 은 기록만, SIM_PATH_AUDIT=1 은 위반이면 401)
        _audit_on = os.environ.get("SIM_PATH_AUDIT", "0") != "0"
        if _audit_on or os.environ.get("SIM_TRACE_CARRY", "0") != "0":
            try:
                _viol = self._audit_plan(segs, JOINT_SEGS)
            except Exception as _exc:      # noqa: BLE001 — 계측이 계획을 막으면 안 된다
                _viol = ""
                self.say(f"[경로검사] 실패(무시): {type(_exc).__name__}: {_exc}")
            if _audit_on and _viol:
                return None, 401, _viol
        # **계획 전체를 파일로 덤프한다** (SIM_PLAN_DUMP). Isaac 없이 도는 가지 감사
        # 도구(m0609_plan_audit.py)에 그대로 넣기 위한 것이다.
        if os.environ.get("SIM_PLAN_DUMP"):
            import json
            _lin = {"down", "lift", "wedge", "back", "touch", "push", "retreat"}
            _wp, _lab, _isl = [], [], []
            for _n in ("approach", "down", "lift", "carry_rotate", "wedge",
                       "back", "touch", "push", "retreat", "return"):
                for _q in segs.get(_n, []):
                    _wp.append([float(v) for v in _q])
                    _lab.append(_n)
                    _isl.append(_n in _lin and _n not in JOINT_SEGS)
            try:
                with open(os.environ["SIM_PLAN_DUMP"], "w") as _f:
                    json.dump({"home": [float(v) for v in self.q_home],
                               "waypoints": _wp, "labels": _lab, "linear": _isl}, _f)
                self.say(f"계획 덤프: {len(_wp)}점 → {os.environ['SIM_PLAN_DUMP']}")
            except Exception as _exc:      # noqa: BLE001
                self.say(f"[경고] 계획 덤프 실패: {_exc}")
        if shifted:
            # 같은 자세를 다른 값으로 표현한 것뿐이라 홈도 같이 맞춘다 (물리적으로 동일하다)
            self.q_home = np.asarray(segs["approach"][0], float).copy()
            self.say(f"손목 감김을 풀었다: {shifted} → 홈 관절각 "
                     f"{np.round(self.q_home, 3).tolist()}")
        # 돌려주는 값은 **월드 기준**이다 — verify()/survey() 가 월드 좌표와 견준다
        return {"segs": segs, "worst": worst_all, "spine_final": spine_final_w, "place_x": place_x_w,
                "floor_z": floor_z, "book": book, "dims": (T, Lb, W),
                "grasp_align_deg": round(math.degrees(_align), 3),
                # **지금 이 순간의** 칸 방향 폭. `dims` 는 장면을 만들 때 잰 값이라
                # 그 뒤 책이 기울면 낡는다 — 손가락을 얼마나 벌릴지는 지금 값으로 정해야 한다
                "span_now": float(self._span_along(bb, self._close_axis_w()))}, 0, ""

    def _close_axis_w(self):
        """손가락이 닫히는 방향 (월드 단위벡터). `SIM_GRIP_ROT90` 을 따른다."""
        _cy, _sy = math.cos(self.tray_yaw), math.sin(self.tray_yaw)
        return (np.array([-_sy, _cy, 0.0]) if GRIP_ROT90
                else np.array([_cy, _sy, 0.0]))

    @staticmethod
    def _span_along(bb, axis):
        """축정렬 상자가 `axis` 방향으로 차지하는 폭. 상자라서 성분 절댓값의 가중합이다.

        2026-09-24: 여기서 월드 x 만 집었다가 **책 높이 227 mm 의 절반**을 두께 반폭으로
        읽었다. 트레이에 선 책은 두께축이 월드 y 이고, 서가에 꽂힌 책은 월드 x 다 —
        어느 쪽이든 **손가락이 닫히는 축**으로 재야 한다.
        """
        a = np.abs(np.asarray(axis, float))
        d = np.array([bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]], float)
        return float(np.dot(a, d))

    def fit_bookends(self, place_x, thickness):
        """북엔드 한 쌍을 이 책 두께에 맞춘다 (책마다 두께가 달라서 — 꽂기 전에만 옮긴다)"""
        key = min(self.bookends, key=lambda k: abs(k - place_x)) if self.bookends else None
        if key is None:
            return
        gap = abs(key - place_x)
        if gap > 0.03:
            # 조용히 돌아가면 북엔드가 **만들어진 자리에 그대로 남는다.** 칸 x 를 옮긴 뒤
            # (2026-09-20 M0609 에서 +0.04) --place-dx 를 같이 안 옮기면 매번 여기로 빠지는데,
            # 로그가 없어서 북엔드가 제 자리에 없는 줄도 모른다 (2026-09-21 점검에서 발견).
            self.say(f"[경고] 북엔드를 못 맞춘다: 꽂을 x {place_x:+.4f} 에서 가장 가까운 "
                     f"북엔드가 {key:+.4f} ({gap*100:.1f} cm 차이). "
                     f"--place-dx 를 칸 좌표에 맞출 것 — 북엔드가 엉뚱한 자리에 남는다")
            return
        pL, pR, y, z = self.bookends[key]
        half = thickness / 2 + DIV_GAP + DIV_T / 2
        q = np.array([1.0, 0.0, 0.0, 0.0])
        # **key(=--place-dx 값) 가 아니라 실제 꽂을 x 를 중심으로** 놓는다.
        # key 를 쓰면 3 cm 창 안에서도 그만큼 어긋난 자리에 세워진다
        SingleXFormPrim(pL).set_world_pose(np.array([place_x - half, y, z]), q)
        SingleXFormPrim(pR).set_world_pose(np.array([place_x + half, y, z]), q)

    # ---------------------------------------------------------------- 실행 조립
    def home_moves(self):
        q_now = self.robot.get_joint_positions()[self.idx_arm]
        return [named(MoveJoint(q, speed_scale=0.6, timeout_s=15), "home")
                for q in tucked_joint_moves(q_now, self.q_home, self.conf["poses"]["stow"])]

    def snap_to_home(self, settle_steps=180):
        """시작 자세를 홈으로 **고정**한다 (움직여서 가는 대신 그 자세로 시작).

        로봇 USD 의 기본 관절값은 AMR 담당 소유라 건드리지 않고, 재생 시작 시점에만 바꾼다.
        시작 홈 이동(약 12초)이 사라지고, 매번 같은 자세에서 시작하므로 재시작이 빨라진다.
        """
        q = self.robot.get_joint_positions()
        q[self.idx_arm] = self.q_home
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(np.zeros_like(q))
        # 관절 위치만 옮기면 **구동 목표는 옛 자세에 남아** 첫 동작에서 팔이 튄다
        # (실측: 고정 직후 첫 작업이 M406 으로 실패). 목표도 같은 값으로 맞춘다.
        # joint_indices 를 빼면 일부 환경에서 지령이 반영되지 않는다 (아래 _apply 주석과 같은 이유).
        # 여기서 빠지면 '구동 목표를 같은 값으로 맞춘다' 는 이 함수의 목적이 조용히 무산된다.
        self.robot.apply_action(ArticulationAction(
            joint_positions=q, joint_indices=np.arange(len(q))))
        # 기본 상태로도 저장해 두면 world.reset() 뒤에도 같은 자세로 돌아온다
        try:
            self.robot.set_joints_default_state(positions=q)
        except Exception as exc:     # noqa: BLE001
            # 삼키면 world.reset() 뒤 에셋 기본 자세로 돌아가는데 이유를 알 수 없다
            self.say(f"기본 자세 저장 실패(무시하고 진행): {exc}")
        # 순간이동 뒤에는 트레이 책도 흔들린다. 충분히 가라앉힌 뒤 준비 완료로 본다
        # (실측: 30 스텝만 두면 첫 작업이 M406 으로 실패)
        for _ in range(settle_steps):
            self.arm.update(); self.world.step(render=False)
        err = float(np.max(np.abs(self.robot.get_joint_positions()[self.idx_arm] - self.q_home)))
        self.say(f"시작 자세를 홈으로 고정 (오차 {err:.4f} rad)")
        return err

    def job_sequence(self, name, plan):
        s = plan["segs"]; book = plan["book"]
        T = plan.get("dims", (self.T, self.L, self.W))[0]
        # **벌림은 지금 폭으로, 조임은 규격 두께로.** 둘은 다른 양이다
        # (`SIM_GRIP_OPEN_MEASURED`, 기본 꺼짐).
        #
        # `dims` 는 **장면을 만들 때** 잰 값이다. 그 뒤 책이 트레이에서 2~3° 기울면
        # 칸 방향 폭이 35.2 → 43~45 mm 로 자라는데, 벌림은 여전히 35.2 로 계산된다:
        #
        #     벌림 22.60 mm  vs  기울기 2.4° 인 책의 반폭 22.34 mm  →  여유 0.26 mm
        #                        기울기 2.7° →  반폭 22.93 mm      →  **여유 −0.33 mm**
        #
        # 즉 **열린 손가락이 이미 책에 닿아 있거나 파고들어 있다.** 2026-09-24 실측에서
        # 기준선 판의 기울기가 1.9~2.7° 였으니 가장 큰 판은 이미 음수였다. 파지정렬을
        # 켜면 손끝이 1~4.5 mm 더 쓸고 들어가 책을 더 기울인다 — 그래서 정렬이
        # 역효과였다.
        #
        # 조임(grip)은 규격 두께를 그대로 쓴다. 기운 폭으로 조이면 손가락이 책에
        # 닿기도 전에 멈춘다(헛쥠).
        #
        # 대가: 벌림 22.60 → 27.35 mm 면 이웃까지 여유가 8.40 → 3.65 mm 로 준다.
        # 그래서 기본은 꺼 두고 A/B 로 견준다.
        _span_now = float(plan.get("span_now", 0.0) or 0.0)
        if os.environ.get("SIM_GRIP_OPEN_MEASURED", "0") != "0" and _span_now > T:
            o = _span_now / 2 + GRIP_CLEAR
            self.say(f"[벌림] 지금 칸 방향 폭 {_span_now*1000:.1f} mm "
                     f"(장면 만들 때 {T*1000:.1f}) → 손가락을 {o*1000:.2f} mm 로 벌린다 "
                     f"[SIM_GRIP_OPEN_MEASURED]. 규격으로 벌리면 {(T/2+GRIP_CLEAR)*1000:.2f} mm 라 "
                     f"책과 여유가 {((T/2+GRIP_CLEAR) - _span_now/2)*1000:+.2f} mm 뿐이다")
        else:
            o = T / 2 + GRIP_CLEAR
            if _span_now > T:
                self.say(f"[벌림] 손가락 {o*1000:.2f} mm · 지금 책 반폭 "
                         f"{_span_now/2*1000:.2f} mm → 여유 {(o - _span_now/2)*1000:+.2f} mm"
                         + ("  **음수다 — 열린 손가락이 책에 박힌다**"
                            if o < _span_now / 2 else ""))
        # **놓는 순서** (SIM_RELEASE_OPEN_FIRST=1): 손을 먼저 벌리고 그다음 책을 동적으로·충돌 켬.
        # 키네마틱 파지는 운반 동안 책 충돌을 꺼서 손가락이 지령 폭(T/2 − 4 mm)까지 책 **속으로**
        # 들어가 있다 (2026-09-22 야간 v01: 손가락 13.6 mm = 지령값 그대로, 책 반두께 17.6 mm).
        # 그 상태로 충돌을 켜면 겹침이 한꺼번에 풀리며 책이 튀어 0.44 s 만에 바닥(z 0.075)에 있었다.
        # 기본(꺼짐)은 지금 순서 그대로: detach → 벌림.
        if GRASP_KINEMATIC and os.environ.get("SIM_RELEASE_OPEN_FIRST", "0") != "0":
            # **벌린 직후에도 찍는다.** 놓는 일은 두 단계다 — 손가락을 벌리고, 책을
            # 동적으로 되돌린다. 기울기가 어느 쪽에서 붙는지 두 줄이 있어야 갈린다:
            #   놓기직전 → 벌린뒤   차이가 나면 **손가락이 튕긴 것**
            #   벌린뒤   → 놓음     차이가 나면 **동적 전환·정착**
            # 2026-09-24 에 이 구분이 없어서 "떠오름이 기울기의 원인" 이라는 틀린
            # 인과를 세웠다 (떠오름을 두 방법으로 없앴는데 기울기는 한쪽만 줄었다).
            release = [Call("release", lambda: self.trace(book, "놓기직전", plan)),
                       named(SetGripper(o, settle_s=0.4), "release"),
                       Call("release", lambda: self.trace(book, "벌린뒤", plan)),
                       Call("detach", self.detach),
                       named(Wait(0.4), "release")]
        else:
            release = [Call("detach", self.detach),
                       Call("release", lambda: self.trace(book, "떼어낸뒤", plan)),
                       named(SetGripper(o, settle_s=0.4), "release"),
                       named(Wait(0.4), "release")]
        return Sequence(name, [
            named(SetGripper(o), "approach"), JointPath("approach", s["approach"][1:], 0.5 * SPEED_SCALE),
            JointPath("down", s["down"][1:], 0.25 * SPEED_SCALE),
            # 얇은 책은 4 mm 를 그대로 조이면 손가락이 책을 밀어낸다 → 두께에 비례해 줄인다
            named(SetGripper(max(0.0, T / 2 - min(0.004, 0.15 * T)), settle_s=0.6), "grip"),
            Call("attach", lambda: self.attach(book)),
            Call("attach", lambda: self.trace(book, "파지")),
            JointPath("lift", s["lift"][1:], 0.25 * SPEED_SCALE),
            JointPath("carry_rotate", s["carry_rotate"][1:], 0.35 * SPEED_SCALE), JointPath("wedge", s["wedge"][1:], 0.35 * SPEED_SCALE),
            *release,
            Call("release", lambda: self.trace(book, "놓음", plan)),
            JointPath("back", s["back"][1:], 0.3 * SPEED_SCALE), named(SetGripper(0.0, settle_s=0.4), "touch"),
            JointPath("touch", s["touch"][1:], 0.3 * SPEED_SCALE), JointPath("push", s["push"][1:], 0.12 * SPEED_SCALE), named(Wait(0.3), "push"),
            JointPath("retreat", s["retreat"][1:], 0.35 * SPEED_SCALE), named(SetGripper(o, settle_s=0.3), "retreat"),
            JointPath("return", s["return"][1:], 0.5 * SPEED_SCALE),
        ])

    def release_clearances(self, book, plan):
        """놓기 직전 책이 **무엇에 얼마나 가까운가** — 칸 바닥·북엔드 (m, 양수 = 떨어져 있음).

        왜: 2026-09-24 실측에서 놓는 순간 책이 16 mm 떠오르며 2° 돌았다. 그 두 가지는
        **침투 해소**의 두 얼굴이다 — 운반 동안 책 충돌을 꺼 두므로, 놓으며 켜는 순간
        겹쳐 있던 만큼 밀려난다. 겹친 상대를 찾으면 고칠 곳이 정해진다.
        """
        bb = self.aabb(book)
        # **날값을 같이 남긴다.** 차이만 적으면 어느 쪽이 움직였는지 못 가린다 —
        # 2026-09-24 에 "밑면이 선반 판보다 위인가 아래인가" 를 묻는 데 한 판이 더 들었다.
        out = {"밑면z": float(bb[2]),
               "칸바닥z": float(plan.get("floor_z", float("nan"))),
               "설정판z": float(getattr(self, "shelf_floor_z", float("nan"))),
               "밑면−칸바닥": float(bb[2]) - float(plan.get("floor_z", float("nan")))}
        key = (min(self.bookends, key=lambda k: abs(k - plan.get("place_x", 0.0)))
               if getattr(self, "bookends", None) else None)
        if key is not None:
            for _nm, _pth in zip(("북엔드왼", "북엔드오"), self.bookends[key][:2]):
                try:
                    nb = self.aabb(_pth)
                    ov = np.minimum(bb[3:], nb[3:]) - np.maximum(bb[:3], nb[:3])
                    out[_nm] = float(np.min(ov)) * -1.0   # 양수 = 떨어져 있음
                except Exception:      # noqa: BLE001
                    pass
        return out

    def trace(self, book, tag, plan=None):
        """문제를 찾을 때 책 위치를 단계별로 남긴다 (책 종류가 섞이면 실패 지점이 달라진다).

        **기울기를 같이 남긴다.** 2026-09-24 에 꽂힌 책이 늘 2~3° 비뚤다는 것을 알았는데,
        그것이 **놓기 전에 이미 있는지 놓은 뒤에 생기는지**를 가를 자료가 없었다.
        `놓기직전` 과 `놓음` 두 줄을 견주면 한 판으로 갈린다.
        """
        b = self.aabb(book); c = (b[:3] + b[3:]) / 2
        self.say(f"  [{tag}] {book.rsplit('/', 1)[1]} 중심 {np.round(c, 3).tolist()} "
                 f"크기 {np.round([b[3] - b[0], b[4] - b[1], b[5] - b[2]], 3).tolist()} "
                 f"틀어짐 {self.axis_skew_deg(book):.2f}°"
                 + ("" if plan is None else "  " + " · ".join(
                     (f"{k} {v:.4f}" if k.endswith("z") else f"{k} {v*1000:+.1f} mm")
                     for k, v in self.release_clearances(book, plan).items())))

    def axis_skew_deg(self, book) -> float:
        """책이 **세상 축에서** 얼마나 틀어져 있는가 (도). 반듯하면 0.

        축 이름은 상관없다 — 어느 축이 어디로 가든, 세 축이 세상 축에 나란하기만 하면
        0 이다. 그래서 90° 회전(트레이 ↔ 서가)에는 안 걸리고 **기울기만** 잡는다.
        """
        try:
            q = SingleXFormPrim(book).get_world_pose()[1]
            R = R_from_quat(np.asarray(q, float))
            worst = 0.0
            for j in range(3):
                v = R[:, j]
                # 가장 가까운 세상 축과의 각 (부호 무시)
                c = float(np.max(np.abs(v)))
                worst = max(worst, math.degrees(math.acos(min(1.0, max(0.0, c)))))
            return worst
        except Exception:      # noqa: BLE001 - 계측이 작업을 막으면 안 된다
            return float("nan")

    def diag_attach_tick(self):
        """`attach()` **한 스텝 뒤** 앵커 기대값과 실제 책 자세의 차이를 손 기준으로 적는다.

        왜 한 스텝 뒤인가: `attach()` 는 USD 에 세운 자세를 쓰고 **그 값을 되읽어** 앵커를
        만든다. PhysX 는 자기 상태를 따로 갖고 있어, 한 스텝 돌려야 실제 값이 드러난다.
        이 차이가 M406(운반 중 어긋남)과 같이 움직이면 가설 H-b 가 맞는 것이다.

        SIM_DIAG_M406=1 일 때만 돈다. **아무 것도 바꾸지 않는다.**
        """
        pending = getattr(self, "_diag_attach", None)
        if pending is None or os.environ.get("SIM_DIAG_M406") != "1":
            return
        self._diag_attach = None
        book, want_rel = pending
        try:
            hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
            bp, _ = SingleXFormPrim(book).get_world_pose()
            got_rel = R_from_quat(np.asarray(hq, float)).T @ (
                np.asarray(bp, float) - np.asarray(hp, float))
            d = got_rel - want_rel
            self.say(f"[DIAG] attach_offset_hand={np.round(d, 5).tolist()} "
                     f"크기 {float(np.linalg.norm(d))*1000:.2f} mm ({book.rsplit('/', 1)[-1]})")
        except Exception as exc:      # noqa: BLE001 - 계측이 작업을 막으면 안 된다
            self.say(f"[DIAG] attach_offset 못 쟀다: {type(exc).__name__}: {exc}")

    def start_delivery(self):
        """이송을 예약한다. 앵커가 잡힌 뒤 부른다.

        **출발 자리로 옮기지 않는다.** 레벨이 이미 트레이를 출발 자리에 놓아 두었다
        (레벨의 authored translate 가 `SIM_TRAY_FROM` 과 같은 값이다). 동적 강체를
        `set_world_pose` 로 옮기는 것은 순간이동이라, 같은 자리로 옮기더라도 속도·접촉이
        어긋나 책이 튄다. 레벨이 정답이니 그대로 둔다 (2026-09-22).
        """
        if not self._deliver or self._deliver["n"] > 1:
            return
        d = self._deliver
        _dt = max(1e-4, float(self.world.get_physics_dt()))
        d["n"] = max(1, int(TRAY_DELIVERY_S / _dt))
        d["wait"] = max(0, int(TRAY_SETTLE_S / _dt))
        # 출발점은 **레벨에 있는 지금 자리**로 잡는다. 상수와 다르면 레벨을 따른다
        _p_now, _q_now = SingleXFormPrim(self.tray).get_world_pose()
        _gap = float(np.linalg.norm(np.asarray(_p_now, float) - d["from"]))
        if _gap > 0.005:
            self.say(f"[이송] 레벨 트레이가 상수와 {_gap*100:.1f} cm 다르다 — "
                     f"**레벨을 따른다** {np.round(_p_now, 4).tolist()}")
            d["to"] = d["to"] + (np.asarray(_p_now, float) - d["from"])
            d["from"] = np.asarray(_p_now, float)
        d["q"] = np.asarray(_q_now, float)
        # **이송 동안만 키네마틱으로 바꾼다** (컨베이어와 같다).
        # 동적 강체에 속도를 줘 봤더니 반납기 위 마찰이 매 스텝 상쇄해서 82.5 cm 중
        # 6.4 cm 만 갔다 (2026-09-22 실측). 키네마틱은 물리가 밀지 못하고, 목표 자세와의
        # 차이가 곧 속도라 **책이 마찰로 실려 온다.** 도착하면 다시 동적으로 돌린다.
        UsdPhysics.RigidBodyAPI.Apply(
            self.stage.GetPrimAtPath(self.tray)).CreateKinematicEnabledAttr().Set(True)
        self.say(f"[이송] 책이 칸에 앉기를 {TRAY_SETTLE_S:.1f}초 기다린 뒤 "
                 f"{TRAY_DELIVERY_S:.1f}초 동안 민다 (대기 {d['wait']} + 이동 {d['n']} 스텝)")

    def _move_tray_group(self, world_p, world_q):
        """트레이와 **그 위의 책을 같은 강체처럼** 옮긴다 (follow_tray 와 같은 방식)."""
        cur_p, cur_q = SingleXFormPrim(self.tray).get_world_pose()
        Rc = R_from_quat(np.asarray(cur_q, float))
        Rw = R_from_quat(np.asarray(world_q, float))
        held = getattr(self, "_held_book", None)
        rel = {}
        for b in getattr(self, "_tray_books", {}):
            if b == held:
                continue
            bp, bq = SingleXFormPrim(b).get_world_pose()
            rel[b] = (Rc.T @ (np.asarray(bp, float) - np.asarray(cur_p, float)),
                      Rc.T @ R_from_quat(np.asarray(bq, float)))
        SingleXFormPrim(self.tray).set_world_pose(np.asarray(world_p, float),
                                                  np.asarray(world_q, float))
        for b, (rp, rR) in rel.items():
            SingleXFormPrim(b).set_world_pose(np.asarray(world_p, float) + Rw @ rp,
                                              quat_from_R(Rw @ rR))

    def lock_base_mass(self):
        """차체를 **무겁게** 만들어 팔 반작용에 들리지 않게 한다. 장면 준비 때 한 번 부른다.

        조인트·자세붙잡기를 둘 다 시험했고 둘 다 실패했다 (위 `BASE_MASS_KG` 주석 참조).
        질량은 물리 그대로라 솔버와 싸우지 않고, 주행은 어차피 순간이동이라 영향이 없다.
        """
        if not FIX_BASE:
            return
        link = f"{BOT.root}/{BASE_LOCK_LINK}"
        prim = self.stage.GetPrimAtPath(link)
        if not prim.IsValid():
            self.say(f"[차체] 링크를 못 찾았다: {link} — 질량을 올리지 않는다")
            return
        api = UsdPhysics.MassAPI.Apply(prim)
        before = api.GetMassAttr().Get()
        api.CreateMassAttr().Set(BASE_MASS_KG)
        self.say(f"[차체] {BASE_LOCK_LINK} 질량 {before} → {BASE_MASS_KG} kg "
                 f"(팔 반작용에 들리지 않게) [SIM_FIX_BASE]")

    def hold_base_tick(self):
        """작업 중에는 **아티큘레이션의 월드 자세를 매 스텝 같은 값으로** 고정한다.

        앞서 두 가지를 시험하고 둘 다 실패했다 (2026-09-22):
          - 외부 고정 조인트: 아티큘레이션과 닫힌 루프를 만들어 **덜덜 떨다 주저앉았다**
          - 루트 XForm 붙잡기: USD 루트를 써도 **PhysX 링크는 따라오지 않아** 효과가 없었다
        아티큘레이션 자체의 자세를 쓰면 물리 상태가 같이 갱신되므로 둘 다 피한다.

        주행 중(작업이 아닐 때)에는 아무것도 하지 않는다 — 주행에 지장이 없다.
        """
        if not FIX_BASE or getattr(self, "robot", None) is None:
            return
        if not getattr(self, "job_active", False):
            if getattr(self, "_hold_pose", None) is not None:
                self._hold_pose = None
                self.say("[차체고정] 작업이 끝나 고정을 풀었다 — 주행할 수 있다")
            return
        try:
            if getattr(self, "_hold_pose", None) is None:
                p_w, q_w = self.robot.get_world_pose()
                self._hold_pose = (np.asarray(p_w, float).copy(),
                                   np.asarray(q_w, float).copy())
                self.say(f"[차체고정] 작업 동안 아티큘레이션을 고정한다 "
                         f"{np.round(self._hold_pose[0], 3).tolist()} [SIM_FIX_BASE]")
            self.robot.set_world_pose(*self._hold_pose)
        except Exception as exc:      # noqa: BLE001 — 버전마다 API 가 다르다
            if not getattr(self, "_hold_warned", False):
                self._hold_warned = True
                self.say(f"[차체고정] 고정 실패 ({type(exc).__name__}: {exc}) — 그냥 둔다")

    def tray_watch(self, every=60):
        """트레이가 어디 있는지 주기적으로 찍는다 — **떨어지면 언제 떨어졌는지** 알아야 한다.

        화면으로는 "바닥에 있다" 까지만 보이고, 배치 로그는 "놓았다" 라고만 한다.
        그 사이 어디서 어긋나는지는 **시간에 따라 찍어야** 보인다.
        SIM_TRAY_WATCH=0 으로 끈다.
        """
        if not TRAY_WATCH:
            return
        self._watch_n = getattr(self, "_watch_n", 0) + 1
        if self._watch_n % every:
            return
        p, _ = SingleXFormPrim(self.tray).get_world_pose()
        b = self.aabb(self.tray)
        a, _ = SingleXFormPrim(self._tray_anchor).get_world_pose() if self._tray_anchor else (p, None)
        self.say(f"[트레이] {self._watch_n:5d} 스텝  원점 z {float(p[2]):.4f}  "
                 f"바닥면 z {float(b[2]):.4f}  중심 ({float(p[0]):.3f}, {float(p[1]):.3f})  "
                 f"앵커 ({float(a[0]):.3f}, {float(a[1]):.3f})  "
                 f"이송중={bool(getattr(self, '_deliver', None))} 작업중={getattr(self, 'job_active', False)}")
        if os.environ.get("SIM_TRACE_BASE", "0") != "0" and getattr(self, "robot", None) is not None:
            # 부유 베이스 처짐 추적 (2026-09-22 야간): 루트 XForm 과 물리 팔 베이스를 나란히
            _rp, _ = SingleXFormPrim(R).get_world_pose()
            _lp, _lq = SingleXFormPrim(BASE_LINK).get_world_pose()
            _wp, _wq = SingleXFormPrim(R + "/world").get_world_pose()
            self.say(f"[베이스] {self._watch_n:5d} 스텝  루트 {np.round(np.asarray(_rp, float), 3).tolist()}  "
                     f"world링크 {np.round(np.asarray(_wp, float), 3).tolist()} q {np.round(np.asarray(_wq, float), 3).tolist()}  "
                     f"팔베이스 {np.round(np.asarray(_lp, float), 3).tolist()}")

    def _tray_rigid(self):
        """트레이를 강체로 다룰 핸들 (속도를 주려면 필요하다). 없으면 None."""
        if getattr(self, "_tray_rp", "미설정") == "미설정":
            try:
                from isaacsim.core.prims import SingleRigidPrim
                self._tray_rp = SingleRigidPrim(self.tray)
                self._tray_rp.initialize()
            except Exception as exc:      # noqa: BLE001
                self.say(f"[이송] 트레이 강체 핸들 없음 ({type(exc).__name__}) — 자세로 옮긴다")
                self._tray_rp = None
        return self._tray_rp

    def deliver_tick(self):
        """이송 한 스텝. 끝나면 **그 자리에서 앵커를 다시 잡는다**.

        컨베이어처럼 일정한 속도로 간다. 도중에는 `follow_tray()` 가 손대지 않는다 —
        둘이 같은 물체를 서로 다른 목표로 옮기면 책이 칸에서 튄다.
        """
        d = self._deliver
        if not d or d["n"] <= 1:
            return
        # **먼저 기다린다.** 책이 트레이 바닥에 내려앉을 시간을 준다
        if d.get("wait", 0) > 0:
            d["wait"] -= 1
            if d["wait"] == 0:
                self.say("[이송] 책 안착 대기 끝 — 트레이가 출발한다")
            return
        d["t"] += 1
        # **부드럽게 붙이고 부드럽게 뗀다** (smoothstep). 선형으로 밀면 출발·도착 순간
        # 속도가 0↔최고로 튀어 그 충격에 책이 앞으로 미끄러져 나간다
        # (2026-09-22 실측: 선형이면 진행 방향 앞쪽 한 권이 트레이 밖으로 빠졌다).
        # 3s²-2s³ 는 양 끝에서 속도가 0 이라 충격이 없다.
        s = min(1.0, d["t"] / d["n"])
        u = s * s * (3.0 - 2.0 * s)
        want = np.asarray(d["from"] + (d["to"] - d["from"]) * u, float)
        # **데크에 닿지 않게 띄워서 간다.** 처음 5% 구간에 걸쳐 부드럽게 들어올린다
        # (한 번에 올리면 그 스텝만 순간이동처럼 보인다).
        _r = min(1.0, s / 0.05)
        want[2] += TRAY_CLEAR_M * (_r * _r * (3.0 - 2.0 * _r))
        # **속도로 민다.** `set_world_pose` 는 순간이동이라 접촉이 생기지 않아
        # 책이 트레이를 따라오지 못한다 (2026-09-22 실측: 트레이만 가고 책은 남았다).
        # GUI 에서 손으로 끌 때 책이 따라오는 것은 힘으로 끌기 때문이다.
        # 속도를 주면 마찰이 생겨 책이 칸에 담긴 채 같이 간다.
        # 키네마틱 목표 자세를 준다. PhysX 가 (목표 − 현재)/dt 를 속도로 삼아
        # 접촉·마찰을 만들어 주므로 책이 칸에 담긴 채 실려 온다.
        SingleXFormPrim(self.tray).set_world_pose(want, np.asarray(d["q"], float))
        if s >= 1.0:
            # **다시 동적으로.** 띄워 둔 TRAY_CLEAR_M 만큼 스스로 내려앉고,
            # 이제부터 트레이는 데크 위에 얹혀 마찰로 실려 간다
            UsdPhysics.RigidBodyAPI.Apply(
                self.stage.GetPrimAtPath(self.tray)).CreateKinematicEnabledAttr().Set(False)
            self._deliver = None
            # 도착 자리에서 앵커와의 관계를 다시 잡는다 — 이후 주행하면 따라온다
            self.rebase_tray_to(d["to"], d["q"])
            # **측정값을 찍는다.** 예전에는 목표값 `d["to"]` 를 찍어서, 트레이가 6.4 cm 만
            # 가고 멈춘 실행도 "안착 [4.993…]" 이라고 초록불이 떴다 (2026-09-22).
            _mp, _ = SingleXFormPrim(self.tray).get_world_pose()
            _err = float(np.linalg.norm(np.asarray(_mp, float) - d["to"]))
            self.say(f"[이송] 트레이 **실측** {np.round(np.asarray(_mp, float), 3).tolist()} "
                     f"(목표 {np.round(d['to'], 3).tolist()}, 오차 {_err*100:.1f} cm) "
                     f"{'OK' if _err < 0.03 else '**빗나감**'}")

    def rebase_tray_to(self, world_p, world_q):
        """다음 스텝에 **트레이를 이 월드 자세에 그대로 두고**, 앵커와의 관계만 다시 잡는다.

        주행 복귀 보정이 부르는 것이다. 보정은 루트를 옮겨 팔 베이스를 출발 자리에 맞추는데,
        트레이 앵커(차체)도 함께 끌려간다. 그러면 추종이 트레이를 따라 옮겨 **칸 중심에
        정확히 있던 책이 밀린다** (2026-09-21 실측: 트레이 월드 6.1 cm, 책 팔기준 5.4 cm
        → 운반 중 406). 보정 전 트레이는 이미 출발 팔 기준 칸 중심에 있으므로,
        **트레이는 두고 관계만 갱신**하면 보정 뒤에도 칸 중심이 유지된다.
        """
        self._rebase_tray_to = (np.asarray(world_p, float), np.asarray(world_q, float))

    def follow_tray(self):
        """트레이(와 트레이 위 책)를 로봇에 붙어 있게 유지한다. 매 스텝 부른다.

        왜 이렇게 하나: 트레이는 로봇의 **형제 prim** 이라 로봇이 주행해도 따라오지 않는다.
        고정 조인트로 묶으면 트레이가 흔들려 책이 칸에서 벗어났다 — 키네마틱으로 직접 옮긴다.
        책은 손에 들려 있지 않은 것만 **트레이와 같은 변위**로 옮긴다 (미끄러짐 방지).
        """
        if not TRAY_CARRY:
            return
        if not getattr(self, "_tray_anchor", None):
            return
        # **작업 중에는 아무것도 옮기지 않는다.** 팔이 움직이면 그 반작용으로 베이스가
        # 흔들리는데, 그때마다 트레이를 순간이동시키면 그 위의 책이 밀려 파지가 깨진다
        # (실측: 책만 얼렸을 때 상승 3.2~4.1cm, 기준 5.0cm 미달).
        # 작업 중에는 AMR 이 서 있으므로 따라갈 것도 없다.
        if getattr(self, "job_active", False):
            return
        # **이송 중에는 손대지 않는다.** 둘이 같은 트레이를 서로 다른 목표로 옮기면
        # 책이 칸에서 튄다 (deliver_tick 이 끝나면서 앵커를 다시 잡아 준다)
        if getattr(self, "_deliver", None):
            return
        ap, aq = SingleXFormPrim(self._tray_anchor).get_world_pose()
        Ra = R_from_quat(np.asarray(aq, float))
        # 복귀 보정이 요청했으면 **트레이는 두고 관계만 다시 잡는다** (rebase_tray_to 참조)
        rebase = getattr(self, "_rebase_tray_to", None)
        if rebase is not None:
            self._rebase_tray_to = None
            # **기록해 둔 목표값이 아니라 지금 실제 트레이 자세를 쓴다.**
            # 목표값과 앵커를 서로 다른 순간의 값으로 섞으면 그 차이가 상대 위치에
            # 그대로 박힌다 (2026-09-22: 20 cm 어긋난 채 고정됐다).
            tp, tq = SingleXFormPrim(self.tray).get_world_pose()
            tp = np.asarray(tp, float); tq = np.asarray(tq, float)
            _cmd_tp, _ = rebase
            self._tray_rel_p = Ra.T @ (tp - np.asarray(ap, float))
            self._tray_rel_R = Ra.T @ R_from_quat(tq)
            self.say(f"[추종] 복귀 보정 뒤 트레이를 보정 전 자리에 고정: "
                     f"{np.round(tp, 4).tolist()}")
            return
        want_p = np.asarray(ap, float) + Ra @ self._tray_rel_p
        want_R = Ra @ self._tray_rel_R
        cur_p, _ = SingleXFormPrim(self.tray).get_world_pose()
        # **서 있을 때는 손대지 않는다.** 베이스는 가만히 있어도 미세하게 떨리는데,
        # 그 떨림마다 책을 순간이동시키면 파지를 방해한다 — 들어 올려도 책이 안 따라오고
        # `405 책 상승 -2.1cm` 로 죽었다 (2026-09-21 A/B 실측, 추종 OFF 는 406 까지 갔다).
        # 주행은 0.3 m/s 기준 한 스텝에 5 mm 라 이 문턱을 넉넉히 넘는다.
        if float(np.linalg.norm(want_p - np.asarray(cur_p, float))) < TRAY_FOLLOW_MIN_M:
            return
        want_q = quat_from_R(want_R)
        SingleXFormPrim(self.tray).set_world_pose(want_p, want_q)
        # 기록해 둔 책을 트레이와 **같은 강체처럼** 옮긴다 (마찰에 기대지 않는다).
        # 잡고 있는 책은 손이 들고 가므로 건드리지 않는다.
        held = getattr(self, "_held_book", None)
        for b, (rel_p, rel_R) in self._tray_books.items():
            if b == held:
                continue
            SingleXFormPrim(b).set_world_pose(want_p + want_R @ rel_p,
                                              quat_from_R(want_R @ rel_R))

    def _lock_book_to_tray(self, book, i):
        """주행 중에 책이 트레이 위에서 미끄러지지 않게 고정 조인트로 묶는다"""
        tp, tq = SingleXFormPrim(self.tray).get_world_pose()
        bp, bq = SingleXFormPrim(book).get_world_pose()
        Rt = R_from_quat(np.asarray(tq, float))
        rel_p = Rt.T @ (np.asarray(bp, float) - np.asarray(tp, float))
        rel_q = quat_from_R(Rt.T @ R_from_quat(np.asarray(bq, float)))
        path = f"/World/bs_book_locks/lock_{i}"
        j = UsdPhysics.FixedJoint.Define(self.stage, path)
        j.CreateBody0Rel().SetTargets([self.tray])
        j.CreateBody1Rel().SetTargets([book])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        j.CreateExcludeFromArticulationAttr().Set(True)
        return path

    def unlock_book(self, book):
        """그 책의 트레이 고정을 푼다. 손이 들기 직전에 부른다"""
        path = self._book_locks.pop(book, None) if hasattr(self, "_book_locks") else None
        if path and self.stage.GetPrimAtPath(path).IsValid():
            self.stage.RemovePrim(path)

    def attach(self, book):
        """파지 순간 손과 책을 고정 조인트로 붙인다 (지침 허용 방식, 마찰 파지는 손목 회전에서 실패 확인)"""
        # 트레이에서 몇 도 기운 채로 잡으면 그 기울기가 그대로 서가까지 간다 (꽂힘 판정 실패).
        # 고정 조인트로 붙이기 직전에 세운 자세로 맞춘다 (자세만, 위치는 그대로).
        # 트레이 고정을 **먼저 푼다.** 안 풀면 트레이 조인트와 손 조인트가 서로 당긴다
        self.unlock_book(book)
        if os.environ.get("SIM_TRACE_GRIP", "0") != "0":
            # 닫힌 직후(충돌이 아직 켜진 상태) 손가락과 책의 AABB 를 나란히 (2026-09-22 야간).
            # 폭 13.6 mm 가 "책에 안 닿음" 인지 "책 속으로 파고듦" 인지 가른다.
            try:
                _bb = self.aabb(book)
                _txt = [f"폭(한쪽) {self.grip_width() * 1000:.1f} mm",
                        f"책 AABB {np.round(_bb, 3).tolist()}"]
                for _fl in BOT.finger_links:
                    _fb = self.aabb(R + "/" + _fl)
                    _txt.append(f"{_fl} AABB {np.round(_fb, 3).tolist()} (책 윗면 대비 손가락 최저 "
                                f"{(float(_fb[2]) - float(_bb[5])) * 100:+.1f} cm)")
                self.say("[파지추적] " + " | ".join(_txt))
            except Exception as _exc:      # noqa: BLE001 — 계측이 작업을 막으면 안 된다
                self.say(f"[파지추적] 실패: {type(_exc).__name__}: {_exc}")
        self._held_book = book      # follow_tray() 가 이 책은 안 건드린다
        getattr(self, "_tray_books", {}).pop(book, None)   # 놓은 뒤에는 서가에 있어야 한다
        if book in self.upright_q:
            # **기하 중심을 축으로 돌린다.** 프림 원점(피벗)을 그대로 두고 자세만 바꾸면
            # 피벗이 기하에서 멀 때 책이 통째로 날아간다 — 이 레벨의 책들은 피벗이
            # 기하에서 **75~177 cm** 떨어져 있어서, 몇 도만 돌려도 수십 cm 씩 튀어
            # 그리퍼 밖으로 빠져 바닥에 떨어졌다 (2026-09-22 실측).
            # 중심을 고정하고 원점을 다시 계산하면 눈에 보이는 책은 제자리에서 돈다.
            # **조금 기운 것은 그냥 둔다.** 그리퍼가 이미 물고 있는 책을 돌리면 그 회전
            # 자체가 손 안에서의 어긋남이 된다 (2026-09-22: 3.9 cm 로 `406`).
            # 크게 누운 경우에만 바로잡는다.
            # **트레이 기준 자세로 다시 만든다** — 로봇이 돌았으면 기준도 같이 돌아야 한다
            _up_w = np.asarray(self.upright_q[book], float)
            if book in getattr(self, "upright_rel", {}):
                _Rt = R_from_quat(np.asarray(
                    SingleXFormPrim(self.tray).get_world_pose()[1], float))
                _up_w = quat_from_R(_Rt @ self.upright_rel[book])
            _q_now = np.asarray(SingleXFormPrim(book).get_world_pose()[1], float)
            _tilt = quat_angle(_q_now, _up_w)
            if _tilt > UPRIGHT_SNAP_RAD:
                _c_now = np.asarray(self.center(book), float)
                _loc = np.asarray(self.grasp_local[book][0], float)  # 책 좌표계의 (중심 − 원점)
                _R_up = R_from_quat(_up_w)
                SingleXFormPrim(book).set_world_pose(_c_now - _R_up @ _loc, _up_w)
                self.say(f"[파지] 책이 {math.degrees(_tilt):.1f}° 기울어 **중심을 축으로** 세웠다")
            elif _tilt > math.radians(1.0):
                self.say(f"[파지] 책이 {math.degrees(_tilt):.1f}° 기울었지만 그냥 잡는다 "
                         f"(문턱 {math.degrees(UPRIGHT_SNAP_RAD):.0f}°)")
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose(); bp, bq = SingleXFormPrim(book).get_world_pose()
        Rh = R_from_quat(hq); rel_p = Rh.T @ (np.asarray(bp) - np.asarray(hp)); rel_q = quat_from_R(Rh.T @ R_from_quat(bq))
        # **정렬이 손에 닿았는가.** 위의 `[파지]` 줄은 책의 **월드** 기울기라, 손을
        # 돌려도 그대로다 — 2026-09-24 에 그 줄로 SIM_GRASP_ALIGN 이 먹었는지 보려다
        # 못 갈랐다. 여기서는 **손이 기준자세에서 얼마나 돌아 있는지**를 찍는다.
        # 정렬을 켰는데 이 값이 0 에 가까우면 명령이 손까지 못 갔다는 뜻이고,
        # 정렬한 각과 같으면 손은 돌았는데 효과가 없었다는 뜻이다. 둘은 고칠 곳이 다르다.
        try:
            _a = math.degrees(quat_angle(np.asarray(hq, float), np.asarray(self.DOWN, float)))
            # `hq` 는 손 **링크**의 자세이고 `DOWN` 은 손끝 기준이라 규약 차이로 180°
            # 가까이 나온다. 2026-09-24 에 179.94° 가 찍혀 읽을 수 없었다. 규약 차이를
            # 접어 **기준자세에서 벗어난 양**만 남긴다 — 정렬을 켜면 이 값이 그만큼 커져야 한다.
            _dev = min(_a, 180.0 - _a)
            self.say(f"[파지] 손이 기준자세에서 {_dev:.2f}° 벗어나 있다 (원값 {_a:.2f}°) · "
                     f"손 기준 책 자세 {np.round(np.asarray(rel_q, float), 4).tolist()} · "
                     f"책이 세상 축에서 {self.axis_skew_deg(book):.2f}° 틀어져 있다")
        except Exception as _exc:      # noqa: BLE001 - 계측이 작업을 막으면 안 된다
            self.say(f"[파지] 손 기울기 계산 실패: {type(_exc).__name__}: {_exc}")
        if GRASP_KINEMATIC:
            # **책을 키네마틱으로 만들어 손에 붙여 옮긴다.**
            # 고정 조인트는 만들어지긴 하는데 접촉력에 밀린다 — 들어 올리는 0.7초 동안
            # 손 기준으로 4.7cm 기울어져 `406` 이 났다 (2026-09-21 실측, 어긋남 곡선이
            # 0→4.7cm 로 매끈하게 자라다 평형에서 멈춘다 = 구속이 아니라 접촉이 이긴다).
            # 키네마틱은 물리가 밀지 못한다. 놓을 때 detach() 가 다시 동적으로 돌린다.
            UsdPhysics.RigidBodyAPI.Apply(
                self.stage.GetPrimAtPath(book)).CreateKinematicEnabledAttr().Set(True)
            self._held_rel = (rel_p, Rh.T @ R_from_quat(np.asarray(bq, float)))
            # **운반 중에는 잡은 책의 충돌을 끈다.** 키네마틱 책은 무한 질량처럼 굴어서,
            # 트레이·이웃 책·로봇 링크에 스치기만 해도 **로봇을 밀어 올린다**
            # (2026-09-22 실측: 팔이 도는 동안 AMR 이 바닥에서 떴다).
            # 손에 고정돼 있으니 운반 동안 충돌은 필요 없다. 놓기 직전에 다시 켠다.
            self._held_coll = []
            for _m in Usd.PrimRange(self.stage.GetPrimAtPath(book)):
                if _m.HasAPI(UsdPhysics.CollisionAPI):
                    _a = UsdPhysics.CollisionAPI(_m).GetCollisionEnabledAttr()
                    self._held_coll.append(_m.GetPath())
                    _a.Set(False)
            if self._held_coll:
                self.say(f"[파지] 운반 동안 책 충돌을 껐다 ({len(self._held_coll)}개) "
                         f"— 놓기 직전에 다시 켠다")
            self._diag_attach = (book, np.asarray(rel_p, float))
            return
        j = UsdPhysics.FixedJoint.Define(self.stage, GRASP_JOINT)
        j.CreateBody0Rel().SetTargets([HAND_LINK]); j.CreateBody1Rel().SetTargets([book])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        j.CreateExcludeFromArticulationAttr().Set(True)
        # **앵커가 기대한 관계**를 적어 둔다. 다음 스텝에 실제와 비교하면
        # "USD 에 쓴 자세와 PhysX 가 가진 자세가 다른가"(가설 H-b)를 숫자로 알 수 있다.
        # 여기 rel_p 는 바로 위에서 **USD 에 세운 자세를 쓴 직후 되읽은** 값이다
        self._diag_attach = (book, np.asarray(rel_p, float))

    def detach(self):
        book = self._held_book
        self._held_book = None
        self._held_rel = None
        if book and GRASP_KINEMATIC:
            # 다시 동적으로 — 놓은 뒤에는 선반에 닿아 멈춰야 한다
            _bp = self.stage.GetPrimAtPath(book)
            if _bp.IsValid():
                UsdPhysics.RigidBodyAPI.Apply(_bp).CreateKinematicEnabledAttr().Set(False)
            for _p in getattr(self, "_held_coll", []):
                _pr = self.stage.GetPrimAtPath(_p)
                if _pr.IsValid() and _pr.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(_pr).GetCollisionEnabledAttr().Set(True)
            if getattr(self, "_held_coll", None):
                self.say(f"[배치] 책 충돌을 다시 켰다 ({len(self._held_coll)}개)")
                self._held_coll = []
        if self.stage.GetPrimAtPath(GRASP_JOINT).IsValid():
            self.stage.RemovePrim(GRASP_JOINT)

    def follow_hand(self):
        """잡은 책을 손에 붙어 있게 유지한다. 매 스텝 부른다 (키네마틱 파지일 때만)"""
        book = getattr(self, "_held_book", None)
        rel = getattr(self, "_held_rel", None)
        if not book or rel is None:
            return
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
        Rh = R_from_quat(np.asarray(hq, float))
        SingleXFormPrim(book).set_world_pose(np.asarray(hp, float) + Rh @ rel[0],
                                             quat_from_R(Rh @ rel[1]))

    def book_origin(self, b):
        return np.asarray(SingleXFormPrim(b).get_world_pose()[0], float)

    def grip_width(self):
        """지금 그리퍼 **한쪽** 폭(m). 실물에서도 읽을 수 있는 신호라 파지 판정 후보다.

        **한쪽 기준이다** — 전체 벌림의 절반이다. 그러니 쥐었을 때 기대값은 책 두께가
        아니라 **책 반두께**다. 예전 주석은 "책 두께 ±3 mm" 라고 했는데 틀렸고,
        그 탓에 JUDGE 가 13.6 mm 를 35.2 mm 와 견줘 늘 'ng' 를 냈다 (2026-09-22).

        그리고 **키네마틱 파지(SIM_GRASP_KINEMATIC=1)에서는 이 신호에 판별력이 없다.**
        운반 동안 책 충돌을 꺼 두기 때문에 손가락이 책을 뚫고 지령 폭
        (`두께/2 − min(4 mm, 0.15·두께)`)까지 닫힌다 — 빈손이어도 같은 값이 나온다.
        마찰 파지일 때만 "책이 손가락을 반두께에서 막는다" 가 성립한다.
        """
        return float(BOT.grip_width(self.robot.get_joint_positions()[self.idx_fing]))

    def book_in_hand(self, b):
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
        return R_from_quat(hq).T @ (self.book_origin(b) - np.asarray(hp, float))

    def book_in_hand_center(self, b):
        """손 기준 (형상 중심 위치, 책 회전행렬). 406 판정식 비교용 (SIM_DRIFT_LOG / SIM_DRIFT_METRIC).

        `book_in_hand` 는 책 prim **원점**을 본다. 이 레벨의 책은 원점이 형상에서 75~177 cm
        떨어져 있어 손이 조금만 돌아도 cm 단위로 벌어진다 (v29). 형상 중심 = 책 자세 ⊗ c_loc.
        c_loc 을 모르는 책(grasp_local 에 없음)은 원점을 그대로 쓴다.
        """
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
        Rh = R_from_quat(np.asarray(hq, float))
        bp, bq = SingleXFormPrim(b).get_world_pose()
        Rb = R_from_quat(np.asarray(bq, float))
        c = np.asarray(bp, float)
        if b in self.grasp_local:
            c = c + Rb @ np.asarray(self.grasp_local[b][0], float)
        return Rh.T @ (c - np.asarray(hp, float)), Rh.T @ Rb

    def book_in_hand_grip(self, b):
        """손 기준 **쥔 점**(윗면 중심)의 위치. 406 판정에 쓰라고 만든 것.

        원점 기준도 형상중심 기준도 **미끄러짐을 재지 않는다.** 둘 다 책이 손 안에서
        조금 돌면 지렛대만큼 증폭된다 — 이 레벨의 책은 원점이 형상에서 75~177 cm
        떨어져 있어서(2026-09-22 실측), 0.3° 만 돌아도 1.7 m × 0.0052 rad ≈ 9 mm 가
        찍힌다. 2026-09-24 마찰 파지 판이 그것이다: 회전 0.3°, 원점 기준 0.1 cm,
        형상중심 기준 1.0 cm — **같은 상태를 재는 세 자가 10배씩 다르다.**

        미끄러짐이란 **손가락이 닿은 자리가 책 위에서 옮겨가는 것**이다. 그러니
        손가락이 잡은 그 점을 손 기준으로 보면 된다. 안 미끄러지면 안 움직인다.
        지렛대가 없으므로 회전에 증폭되지 않는다.
        """
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
        Rh = R_from_quat(np.asarray(hq, float))
        bp, bq = SingleXFormPrim(b).get_world_pose()
        Rb = R_from_quat(np.asarray(bq, float))
        g = np.asarray(bp, float)
        if b in self.grasp_local:
            c_loc, up_loc, hz = self.grasp_local[b]
            g = g + Rb @ (np.asarray(c_loc, float) + np.asarray(up_loc, float) * float(hz))
        return Rh.T @ (g - np.asarray(hp, float))

    def book_origin_lever(self, b):
        """책 원점에서 형상 중심까지의 거리 (m) — **406 증폭 배율**이 곧 이 값이다."""
        if b not in self.grasp_local:
            return 0.0
        return float(np.linalg.norm(np.asarray(self.grasp_local[b][0], float)))

    def shelf_shelf_floor(self, name="secondFloor"):
        """서가 한 층의 낱권 책을 **월드 x 순서로** 돌려준다: [(경로, aabb), …]"""
        sc = self.stage.GetPrimAtPath(f"/World/books/{name}")
        if not sc.IsValid():
            return []
        rows = []
        for c in sc.GetChildren():
            path = str(c.GetPath())
            b = self.aabb(path)
            if not np.all(np.isfinite(b)) or np.any(b[3:] - b[:3] <= 0):
                continue
            rows.append((path, b))
        rows.sort(key=lambda r: float(r[1][0]))
        return rows

    def apply_shelf_gap(self):
        """`SIM_SHELF_GAP` 대로 서가에 빈칸을 **더** 낸다. 레벨 파일은 건드리지 않는다.

        레벨에 이미 있는 빈칸은 `config/ground_truth_slots.yaml` 에 있다. 이 함수는
        **거기 없는 폭이 필요할 때만** 쓴다 — 예를 들어 좌우 여유 5 mm 짜리 한계
        시험을 하고 싶은데 마침 그런 칸이 없을 때다.

        빼는 방법은 프림 비활성(`SetActive(False)`)이다. 런타임에만 꺼지고 레벨
        파일에는 아무 흔적도 남지 않는다.

        돌려주는 값: 낸 빈칸 dict (없으면 None)
        """
        if not SHELF_GAP:
            return None
        parts = [p.strip() for p in SHELF_GAP.split(",")]
        try:
            start = int(parts[0])
            want = float(parts[1]) / 1000.0
        except (IndexError, ValueError):
            raise RuntimeError(
                f"SIM_SHELF_GAP 형식이 잘못됐다: {SHELF_GAP!r} — "
                f'"<시작 인덱스>,<목표 폭 mm>[,<층>]" 이어야 한다 (예: "12,55")')
        floor = parts[2] if len(parts) > 2 else "secondFloor"
        rows = self.shelf_shelf_floor(floor)
        if not rows:
            raise RuntimeError(f"서가 층 '{floor}' 에 낱권 책이 없다 — "
                               f"이름이 맞는지 볼 것 (secondFloor / thirdFloor)")
        if not 0 <= start < len(rows):
            raise RuntimeError(f"시작 인덱스 {start} 가 범위 밖이다 (0~{len(rows)-1}, {floor} {len(rows)}권)")
        removed, acc = [], 0.0
        i = start
        while i < len(rows) and acc < want:
            path, b = rows[i]
            acc += float(b[3] - b[0])
            removed.append((i, path, b))
            i += 1
        for _i, path, _b in removed:
            self.stage.GetPrimAtPath(path).SetActive(False)
        # **실제로 열린 폭**은 뺀 책 두께의 합이 아니라 남은 이웃 사이 거리다
        # (책들이 서로 겹쳐 있어서 두께 합보다 좁게 열린다).
        left = rows[start - 1][1] if start > 0 else None
        right = rows[i][1] if i < len(rows) else None
        x_lo = float(left[3]) if left is not None else float(removed[0][2][0])
        x_hi = float(right[0]) if right is not None else float(removed[-1][2][3])
        opened = x_hi - x_lo
        gt = {
            "floor": floor,
            "removed_index": [r[0] for r in removed],
            "removed": [r[1].rsplit("/", 1)[-1] for r in removed],
            "requested_width_m": round(want, 4),
            "opened_width_m": round(opened, 4),
            "gap_x_min_world": round(x_lo, 4),
            "gap_x_max_world": round(x_hi, 4),
            "gap_center_world": [round((x_lo + x_hi) / 2, 4),
                                 round(float(removed[0][2][1] + removed[0][2][4]) / 2, 4),
                                 round(float(removed[0][2][2] + removed[0][2][5]) / 2, 4)],
            "neighbor_left": (rows[start - 1][1][3].round(4).tolist() if start > 0 else None),
            "neighbor_right": (rows[i][1][0].round(4).tolist() if i < len(rows) else None),
            "remaining": len(rows) - len(removed),
        }
        self.say(f"[빈칸] {floor} {len(rows)}권 중 {len(removed)}권을 뺐다 "
                 f"(인덱스 {gt['removed_index']}) — 요청 {want*1000:.0f} mm, "
                 f"**실제로 열린 폭 {opened*1000:.1f} mm** "
                 f"(월드 x {x_lo:.4f}~{x_hi:.4f}). 레벨 파일은 그대로다")
        if opened < want - 0.002:
            self.say(f"[빈칸] 주의: 요청보다 {(want-opened)*1000:.1f} mm 좁게 열렸다 — "
                     f"책들이 서로 겹쳐 있어 두께 합만큼 열리지 않는다")
        self.shelf_gap = gt
        return gt

    def shelf_book_boxes(self, board_z=None, tol=0.10, exclude=()):
        """서가에 꽂힌 **낱권 책**들의 월드 AABB. **우리가 꽂은 책도 센다.**

        왜 필요한가 (2026-09-23 실측): 서가 낱권 책 64권은 `RigidBodyAPI` 도
        `CollisionAPI` 도 없다 — 순전히 장식이다. 그래서 **꽉 찬 칸에 꽂아도 책이
        옆 책을 그냥 통과하고**, `verify()` 는 우리 책 하나만 보므로 그걸 성공으로
        보고한다. 물리가 말해 주지 않으니 기하가 말해야 한다.

        `/World/books/<층>/<책>` 이 서가 책이고, 우리가 다루는 책은
        `/World/tray_books/*`(레벨 트레이) 또는 `/World/bs_books/*`(스폰) 이다.
        `board_z` 를 주면 그 선반판 위 `tol` 안에 있는 책만 돌려준다.

        **우리가 꽂은 책도 센다** (2026-09-24 실측으로 드러났다). 예전에는 우리 책을
        통째로 뺐는데, 그러면 **한 권 꽂은 뒤에도 그 칸이 비어 보인다** — 실제로
        두 권째가 같은 칸(폭 59.8 mm 그대로)으로 끌려가 옆 책을 3.8 mm 파고들었고,
        임계 5 mm 아래라 `code 0` 으로 "성공" 보고까지 했다.

        **트레이에 있는 우리 책은 안 센다** — 판 높이로 걸러지므로 저절로 빠진다.
        `exclude` 는 지금 판정하려는 책이다. 안 빼면 자기 자신과 100% 겹친다.
        """
        root = self.stage.GetPrimAtPath("/World/books")
        if not root.IsValid():
            return []
        mine = set(self.books)
        out = []
        for floor in root.GetChildren():
            for c in floor.GetChildren():
                path = str(c.GetPath())
                if path in mine:
                    continue
                b = self.aabb(path)
                if not np.all(np.isfinite(b)) or np.any(b[3:] - b[:3] <= 0):
                    continue
                if board_z is not None and abs(float(b[2]) - float(board_z)) > tol:
                    continue
                # 서가가 둘이면 다른 서가의 책이 같은 판 높이에 있다 — 현재 서가 상자로 거른다
                if not book_in_shelf(b, self.shelf_aabb_world):
                    continue
                out.append((path, b))
        # **우리가 꽂은 책을 그 판의 책으로 편입한다.** 판 높이로 거르므로 트레이에
        # 남아 있는 것은 저절로 빠진다.
        _skip = set(exclude) if not isinstance(exclude, str) else {exclude}
        for path in self.books:
            if path in _skip:
                continue
            b = self.aabb(path)
            if not np.all(np.isfinite(b)) or np.any(b[3:] - b[:3] <= 0):
                continue
            if board_z is None or abs(float(b[2]) - float(board_z)) > tol:
                continue
            out.append((path, b))
        return out

    def jam_report(self, bb, board_z, exclude=()):
        """꽂은 책이 **옆 책 속에 박혀 있는가** — 침투 깊이(m)와 최악의 이웃.

        겹침 판정은 세 축 모두 겹칠 때만 성립하고, 침투 깊이는 **떼어내는 데 필요한
        최소 이동량**(세 축 겹침 중 가장 작은 값)으로 잰다.

        주의: 서가 책들끼리도 AABB 가 이미 −0.7~−4.6 mm 겹쳐 있다(9/23 실측,
        `secondFloor` 43권). 그래서 임계값 기본을 5 mm 로 둔다 — 이건 **관측된
        바닥값**이지 통과시키려고 올린 문턱이 아니다.
        """
        worst, who, axes = 0.0, "", None
        for path, nb in self.shelf_book_boxes(board_z, exclude=exclude):
            ov = np.minimum(bb[3:], nb[3:]) - np.maximum(bb[:3], nb[:3])
            if np.any(ov <= 0):
                continue                      # 한 축이라도 안 겹치면 떨어져 있다
            d = float(np.min(ov))
            if d > worst:
                worst, who, axes = d, path.rsplit("/", 1)[-1], ov.copy()
        return worst, who, axes

    def verify(self, plan):
        """꽂힌 책 판정 (multi_book 과 같은 기준)"""
        bb = self.aabb(plan["book"])
        _, Lb, W = plan.get("dims", (self.T, self.L, self.W))
        T0 = plan.get("dims", (self.T, self.L, self.W))[0]
        skew = skew_deg_from_spans(float(bb[3] - bb[0]), float(bb[4] - bb[1]), T0, W)
        checks = {
            "upright": abs((bb[5] - bb[2]) - Lb) < 0.02,
            "depth": abs((bb[4] - bb[1]) - W) < 0.02,
            "spine": abs(bb[1] - plan["spine_final"]) < 0.015,
            "x": abs((bb[0] + bb[3]) / 2 - plan["place_x"]) < 0.015,
            "floor": abs(bb[2] - plan["floor_z"]) < 0.03,
        }
        # **눕혀 꽂힌 것을 이름으로 잡는다.** `upright` 는 z 높이만 보므로 z 축 둘레
        # 90° 회전을 통과시킨다 (2026-09-24 LIVE4: 85.8° 인데 upright True). 그때
        # `depth` 가 잡긴 했지만 "깊이가 안 맞다" 보다 "비뚤어졌다" 가 무슨 일인지를
        # 말해 준다. **정사각 단면이면 각을 못 재므로 검사하지 않는다** — 모르는 것을
        # 통과·불통과 어느 쪽으로도 세지 않는다.
        if not math.isnan(skew):
            checks["skew"] = abs(skew) <= SKEW_TOL_DEG
        else:
            self.say("[꽂은 자세] 단면이 정사각이라 비뚤어짐을 못 잰다 — skew 검사 건너뜀")
        # 옆 책과 겹쳤는가 (T1). 물리가 막아 주지 않으므로 기하로 잰다 — 위 머리말 참조.
        if JAM_CHECK:
            jam, who, axes = self.jam_report(bb, plan.get("floor_z"), exclude=plan["book"])
            checks["no_jam"] = jam <= JAM_TOL
            # **겹침이 안 나도 찍는다.** 비뚤어짐은 겹쳐야만 생기는 게 아니라
            # 시연 구간(-45 ~ +52 mm) 안에서도 작게 일어나고 있을 수 있고,
            # 그건 upright/spine/x 어느 검사도 안 보고 있다. 매 판 남겨서 추세를 본다.
            _T, _Lb, _W = plan.get("dims", (self.T, self.L, self.W))
            _sx = float(bb[3] - bb[0])
            _sy = float(bb[4] - bb[1])
            _off = float((bb[0] + bb[3]) / 2 - plan["place_x"])
            # **부풀음을 각도로 바꿀 때 지렛대를 찍지 않는다.** 가로 두 폭이 다 있으면
            # 어느 쪽이 지렛대인지 고를 필요가 없다. 수평 yaw 를 t 라 하면
            #   Sx = T·cos t + W·sin t ,  Sy = T·sin t + W·cos t
            # 두 식을 더하고 빼면 (cos t + sin t) 와 (cos t − sin t) 가 바로 나온다.
            _skew = skew                  # 위에서 이미 쟀다 — 두 번 재면 갈라진다
            self.say(f"[꽂은 자세] 중심이 목표에서 {_off*1000:+.1f} mm · "
                     f"가로 {_sx*1000:.1f} x {_sy*1000:.1f} mm "
                     f"(규격 두께 {_T*1000:.1f} · 폭 {_W*1000:.1f} mm) "
                     f"→ 수평 기울기 {_skew:+.2f}°")
            if jam > 0:
                # **겹침을 왜 냈는지 한 줄로 가른다**: 책이 옆으로 밀린 것인가(중심 이동),
                # 비뚤어진 것인가(폭 부풀음). 2026-09-24 +60 mm 판에서 x 검사(±15 mm)는
                # 통과하는데 14 mm 겹친 일이 있었다 — 중심만 봐서는 못 가른다.
                self.say(f"[겹침] 꽂은 책이 '{who}' 와 {jam*1000:.1f} mm 겹친다 "
                         f"(축별 {np.round(axes*1000, 1).tolist()} mm, 임계 {JAM_TOL*1000:.0f} mm) "
                         f"— 서가 책은 콜리전이 없어 물리로는 안 막힌다")
                self.say(f"  까닭 가르기: 중심이 목표에서 {_off*1000:+.1f} mm · "
                         f"기울기 {_skew:+.2f}° 로 x 폭이 {(_sx - _T)*1000:+.1f} mm 부풀었다 → "
                         f"{'비뚤어짐' if (_sx - _T) > abs(_off) else '옆으로 밀림'}")
            else:
                _boxes = self.shelf_book_boxes(plan.get("floor_z"), exclude=plan["book"])
                n = len(_boxes)
                # **겹침 0 이 "여유가 있다" 를 뜻하지 않는다.** 겹침 0 으로 통과한 판의
                # 실제 여유가 한쪽 4.9 mm 였던 적이 있다(임계 5 mm). 통과와 아슬아슬함을
                # 가르려면 숫자가 있어야 하는데, 여기 그 숫자가 없었다 — 침투량은
                # 겹쳤을 때만 찍히고 안 겹쳤을 때의 여유는 아무 데도 안 남았다.
                _lo, _ln, _ro, _rn = side_clearances(bb, _boxes)
                _fmt = (lambda d, who: "이웃 없음" if d is None
                        else f"{d*1000:+.1f} mm ({who.rsplit('/', 1)[-1]})")
                self.say(f"[겹침] 옆 책과 겹치지 않음 (그 판의 서가 책 {n}권과 대조) · "
                         f"좌 {_fmt(_lo, _ln)} · 우 {_fmt(_ro, _rn)}")
        return all(checks.values()), checks, bb

    def survey(self):
        """**장면 전체**의 책 상태를 본다 — 꽂은 책만 보면 놓친다.

        왜: `verify()` 는 이번에 꽂기로 한 책 하나만 본다. 그래서
        **다른 책이 쓰러지거나 떨어져도 "성공" 으로 보고된다.**
        실제로 4권 4/4 라고 보고한 녹화에서 트레이에 누운 책이 보였다 (2026-09-20 지적).

        판정: 세운 높이(가장 큰 변)가 z 축과 맞으면 '서 있음', 아니면 '누움'.
        트레이 높이 근처면 트레이, 서가 높이 근처면 서가로 나눈다.
        """
        out = []
        for b in self.books:
            bb = self.aabb(b)
            size = bb[3:] - bb[:3]
            T, Lb, W = self.dims.get(b, (self.T, self.L, self.W))
            z0 = float(bb[2])
            # **높이만 보면 틀린다.** 서가에서 한참 떨어진 자리에 있어도, 손에 들린
            # 채 그 높이에 있어도 "서가" 로 셌다 (2026-09-24 LIVE2: 한 사이클만
            # 성공했는데 "서가 2권" 이 나왔다). 시연 중에 이 차이는 곧바로 해롭다 —
            # **책을 떨어뜨렸는데 "꽂았다" 로 보고되면 아무도 모른다.**
            _sx = None
            try:
                _bb = np.asarray(self.shelf_aabb_world, float)
                _sx = (_bb[0], _bb[3], _bb[1], _bb[4])
            except Exception:      # noqa: BLE001 - 서가 상자를 모르면 옛 판정 그대로
                _sx = None
            # 판은 목록이다 — 위 판에 꽂힌 책을 아래 판 높이로 재면 '바닥/기타' 가 된다
            _fz = nearest_board(z0, self.known_boards(), 0.08)
            where = classify_place(bb, self.shelf_floor_z if _fz is None else _fz,
                                   self.tray_floor_z, _sx)
            # **자리마다 '바른 자세'가 다르다.**
            #   트레이: 책등이 위 → **깊이(W)** 가 수직
            #   서가  : 세워 꽂음 → **높이(L)** 가 수직
            # 이걸 하나로 보면 트레이의 정상 자세를 '쓰러짐' 으로 잘못 읽는다 (2026-09-20).
            want = W if where == "트레이" else Lb
            ok_pose = abs(size[2] - want) < 0.025
            out.append({"book": b.rsplit("/", 1)[-1], "바른자세": bool(ok_pose),
                        "위치": where, "밑면z": round(z0, 4),
                        "중심xy": [round(float((bb[0] + bb[3]) / 2), 3),
                                   round(float((bb[1] + bb[4]) / 2), 3)],
                        "기대수직": round(float(want), 3),
                        "크기": [round(float(v), 3) for v in size]})
        bad = [o for o in out if not o["바른자세"]
               or o["위치"] in ("바닥/기타", "서가높이·서가밖")]
        return out, bad


class _Backend:
    def __init__(self, scene):
        self.s = scene
        self._grip = 0.035

    @property
    def dt(self):
        return 1.0 / 60.0

    def get_joint_positions(self):
        return self.s.robot.get_joint_positions()[self.s.idx_arm]

    def _apply(self, arm_q=None):
        s = self.s
        q = s.robot.get_joint_positions().copy()
        if arm_q is not None:
            q[s.idx_arm] = np.asarray(arm_q, float)[:BOT.dof]
        q[s.idx_fing] = BOT.grip_targets(self._grip); q[s.base_idx] = s.base_hold
        # joint_indices 를 명시한다. 생략하면 일부 환경에서 지령이 반영되지 않아
        # 팔이 제자리에 머문다 (2026-09-20 M0609 에서 approach 제한 시간 초과).
        s.robot.apply_action(ArticulationAction(
            joint_positions=q, joint_indices=np.arange(len(q))))

    def say(self, m):
        self.s.say(m)

    def clash_report(self):
        """팔 링크와 장면 물체의 겹침을 한 줄로 요약한다 (막힌 이유를 눈 없이 알아내려고)"""
        try:
            return self.s.clash_report()
        except Exception as e:
            return f"[진단] 겹침 검사 실패: {e}"

    def set_joint_targets(self, positions):
        self._apply(positions)

    def get_ee_pose(self):
        p, Rm = self.s.ik.compute_end_effector_pose()
        return np.asarray(p, float), quat_from_R(Rm)

    def compute_ik(self, position, orientation, seed=None):
        q, ok = self.s.ik_joints(position, orientation, self.get_joint_positions())
        return (q, True) if ok else (np.zeros(BOT.dof), False)   # 7 이 박혀 있었다 (2026-09-21)

    def set_gripper_width(self, w):
        self._grip = float(w)
        self._apply()

    def get_gripper_width(self):
        return BOT.grip_width(self.s.robot.get_joint_positions()[self.s.idx_fing])
