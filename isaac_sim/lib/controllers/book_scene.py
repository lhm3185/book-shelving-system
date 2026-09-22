"""트레이 → 서가 도서관식 꽂기 장면·계획·실행 조립 (Isaac Sim 5.1.0).

multi_book.py 에서 검증한 코드(트레이 4권 4/4, 튐 0)를 작업 명령 단위로 쓸 수 있게 옮겼다.
SimulationApp 을 만든 뒤에 import 해야 한다.

좌표: 명령은 arm_base_link(= panda_link0) 기준, 물체는 AABB 중심 (FRAMES_CONTRACT).
"""
import math
import os
import sys

import numpy as np
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
from arm_planning import tucked_joint_moves  # noqa: E402
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
DECK_Z = BOT.deck_z         # 트레이가 놓이는 면의 월드 높이 (로봇별)
# 링크가 이보다 낮으면 받침판을 뚫는 것으로 본다 (팔 베이스가 판 위에 바로 붙어 있다)
# 팔이 받침판을 이만큼까지 파고드는 것은 눈감아 준다 (충돌 구가 근사값이라 여유가 필요하다)
DECK_SINK_M = float(os.environ.get("SIM_DECK_SINK", "0.02"))
TIP_DOWN = 0.035            # 책등 윗면에서 손끝이 내려가 잡는 깊이
GRIP_CLEAR = 0.005
SPINE_INSET = 0.02          # 계획상 최종 책등이 서가 앞면에서 들어가는 거리
MEASURED_INSET = 0.024      # 실측 최종 책등 위치 (밀기 후). 꽂힌 책 AABB 중심 → 서가 앞면 역산에 쓴다
DIV_H, DIV_T, DIV_GAP = 0.07, 0.01, 0.003
VEL_LIMIT = np.array(BOT.vel_limit)             # URDF 실측 (로봇별, 프로파일에서)
# 계획 인접점 최대 관절 변화 (초과 = 불연속, M402). 순간이동을 잡으려는 값이지,
# 빠른 구간을 막으려는 값이 아니다. 손목 특이점에서 0.124 rad(7°) 가 남는데
# 32배까지 잘게 나눠도 줄지 않는다 — 진짜 작은 불연속이다 (2026-09-21 실측).
MAX_STEP = float(os.environ.get("SIM_MAX_STEP", "0.12"))
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


class BookScene:
    """레벨 + 트레이 + 책 N권 + 북엔드. 로봇·IK·팔 제어기까지 준비한다"""

    def __init__(self, app, usd, tray_usd, tray_center, n_books, place_dx, say, before_reset=None,
                 book_variants=None):
        self.app, self.say = app, say
        open_stage(usd); app.update()
        while is_stage_loading():
            app.update()
        st = self.stage = get_current_stage()
        self._cache = create_bbox_cache()

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
        if not sources:
            raise RuntimeError("쓸 수 있는 책 원본이 없다")
        shelf = self.aabb(SHELF)
        for p in ([s[0] for s in sources if s[0] is not None]
                  + ([str(c.GetPath()) for c in st.GetPrimAtPath("/World/fixtures").GetChildren()]
                     if st.GetPrimAtPath("/World/fixtures").IsValid() else [])):
            st.GetPrimAtPath(p).SetActive(False)

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
        self.tray = "/World/bs_tray"
        add_reference_to_stage(tray_usd, self.tray)
        # **트레이는 수평이어야 한다.** 팔 베이스 자세를 그대로 쓰면 거기 섞인 뒤집힘·기울기가
        # 트레이에 그대로 들어가 책이 미끄러진다 (2026-09-20: 책 6권이 한쪽에 뭉쳤다).
        # 방향(yaw)만 따르고 나머지는 버린다.
        _yaw = _yaw0
        _tray_q = np.array([math.cos(_yaw / 2), 0.0, 0.0, math.sin(_yaw / 2)])
        SingleXFormPrim(self.tray).set_world_pose(
            np.array([_tray_w[0], _tray_w[1], DECK_Z]), _tray_q)
        say(f"트레이 자세: yaw {math.degrees(_yaw):.1f}° (수평 유지)")
        # **월드 값을 따로 들고 있는다** — 아래 계산들은 월드 기준이라 팔 기준 값을 그대로
        # 쓰면 x,y 는 팔 기준·z 는 월드인 잡종 좌표가 된다 (2026-09-20 실제로 IK 실패).
        self.tray_center_w = _tray_w.copy()
        self.tray_yaw = float(_yaw0)      # 파지·삽입 자세의 기준 방향
        # **팔 기준 값은 주행해도 안 변한다.** refresh_base() 가 이것으로 월드 값을 다시 만든다
        self._tray_rel = _tray_rel.copy()
        say(f"트레이 배치: 팔 기준 {np.round(_tray_rel[:2], 4).tolist()} "
            f"→ 월드 {np.round(_tray_w[:2], 3).tolist()}, 면 z {DECK_Z:.3f}")
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
            if p.IsA(UsdGeom.Mesh) and _tray_coll:
                UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
            for a in p.GetAttributes():
                n = a.GetName()
                if n.endswith("tray_pitch"): pitch = float(a.Get())
                if n.endswith("tray_slots"): nslots = int(a.Get())
                if n.endswith("floor_top_z"): floor_top = float(a.Get())
        # 칸은 **팔 기준 x 축**을 따라 늘어선다 (월드 x 가 아니다)
        slot_rel = [np.array([tray_center[0] + (i + 0.5 - nslots / 2) * pitch,
                              tray_center[1], 0.0]) for i in range(nslots)]
        _Ryaw = np.array([[math.cos(_yaw), -math.sin(_yaw), 0.0],
                          [math.sin(_yaw), math.cos(_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        slot_w = [_bp + _Ryaw @ r for r in slot_rel]
        slot_x = [float(w[0]) for w in slot_w]      # 호환용 (월드 x)

        # 책
        self.books = []
        self.dims = {}          # 책마다 치수가 다르다 (두께 T, 세운 높이 L, 깊이 W)
        self.grasp_local = {}   # 책 좌표계의 (중심, 위 방향, 반높이)
        self.upright_q = {}     # 트레이에서 세운 자세
        self.slot_pose = {}     # 트레이 칸에 놓았을 때의 (위치, 자세) — 데이터 촬영에서 재배치에 쓴다
        for i in range(min(n_books, nslots)):
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

        # 북엔드: 1차 고정 칸마다 한 쌍 (세운 책이 스스로 넘어지는 것 방지 — 칸막이 교훈)
        floor_z = self.shelf_floor_z = SHELF_ROW_Z
        spine_final = self.shelf_front_y + SPINE_INSET
        UsdGeom.Scope.Define(st, "/World/bs_bookends")
        self.bookends = {}      # place_x → (왼쪽 translate op, 오른쪽 translate op, y, z)
        for k, dx in enumerate(place_dx):
            px = link0_x + dx
            y0, y1 = spine_final + 0.02, spine_final + self.W_max
            paths = {}
            for side, sgn in (("L", -1), ("R", +1)):
                p = f"/World/bs_bookends/b{k}_{side}"
                cube = UsdGeom.Cube.Define(st, p)
                cube.CreateSizeAttr(1.0)
                cube.AddTranslateOp().Set(
                    Gf.Vec3d(px + sgn * (self.T_max / 2 + DIV_GAP + DIV_T / 2), (y0 + y1) / 2, floor_z + DIV_H / 2))
                cube.AddScaleOp().Set(Gf.Vec3f(DIV_T, y1 - y0, DIV_H))
                cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.25)])
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                # 책 두께에 맞춰 꽂기 직전에 옮긴다 → 정적 콜라이더로 두면 물리에 반영되지 않아 kinematic 강체로
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateKinematicEnabledAttr().Set(True)
                paths[side] = p
            self.bookends[round(px, 4)] = (paths["L"], paths["R"], (y0 + y1) / 2, floor_z + DIV_H / 2)

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
            if _t not in (None, 0.0):
                _d.CreateTargetPositionAttr().Set(0.0)
                _tuned.append(f"{_p.GetName()} 목표 {_t}°→0°")
        say(("팔 구동 게인 설정: 강성 %.0e 감쇠 %.0e 힘상한 %s" % (_stiff, _damp, _maxf or "에셋값")) if _stiff > 0
            else "팔 구동 게인: **에셋 기본값 사용** (ARM_DRIVE_STIFFNESS=0)")
        if _tuned:
            say(f"  구동 목표 보정: {_tuned}")

        if before_reset is not None:
            before_reset(st)      # ROS 그래프 설정 보완 등 — 재생(초기화) 전에 해야 반영된다
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
            _desc = yaml.safe_load(open(BOT.lula[1]))
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
        self.DOWN = self.orientation([0, 0, -1], _arm_x_w)
        self.HORIZ = self.orientation(_arm_y_w, _arm_x_w)
        self.say(f"파지 자세 기준: 팔 yaw {math.degrees(self.tray_yaw):.1f}°, "
                 f"물림축(월드) {np.round(_arm_x_w, 3).tolist()}")

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
        if BRANCH_GUARD and _br != "up":
            self.say("[경고] 홈이 팔꿈치↑ 가 아니다 — 가지 가드가 모든 IK 를 거절할 수 있다. "
                     "arm_<로봇>.yaml 의 poses.home 을 확인할 것")
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
            # world.step() 을 부르는 곳이 셋이다 (run_simulation, manipulation_executor 2군데).
            # 물리 콜백에 물리면 어디서 돌리든 한 번씩만 불린다.
            try:
                self.world.add_physics_callback(
                    "bs_tray_follow", lambda _dt: (self.follow_hand(), self.follow_tray()))
            except Exception as exc:     # noqa: BLE001
                self.say(f"[경고] 트레이 추종 콜백 등록 실패 — 주행하면 트레이가 뒤에 남는다: {exc}")

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

        # 방향은 yaw 만 쓴다. 베이스에 섞인 기울기를 손 자세에 넣으면 책이 기운다
        yaw = math.atan2(float(self.Rl0[1, 0]), float(self.Rl0[0, 0]))
        # **손 자세는 yaw 에만 의존한다 — yaw 가 그대로면 다시 만들지 않는다.**
        # 같은 값을 다시 계산해도 쿼터니언이 미세하게 달라지고, 그 차이로 IK 가
        # 다른 가지를 골라 `402 approach 2.9 rad` 로 죽는다. 제자리에서든 평행이동에서든
        # 마찬가지였다 (2026-09-21: 무작위 배치 시험에서 옮기기만 해도 3/4 가 실패).
        _yaw_changed = abs(yaw - self.tray_yaw) > 1e-6
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
            self.DOWN = self.orientation([0, 0, -1], _arm_x_w)
            self.HORIZ = self.orientation(_arm_y_w, _arm_x_w)
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
        (근거: 웹 클로드 v21 회신 §1, 도구 isaac_sim/isaac/tools/branch/m0609_kin.py)
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

    def ik_joints(self, target, ori, seed):
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
        for k, s0 in enumerate(_seeds):
            q, ok = self.lula.compute_inverse_kinematics(
                BOT.ee_frame, np.asarray(target, float), np.asarray(ori, float),
                np.asarray(s0, float), 0.004, 0.05)
            if not ok:
                continue
            q = np.asarray(q, float)
            q = self._unwrap_to(q, seed)
            if not self._branch_ok(q):
                continue          # 팔꿈치↓ 해는 받침판에 닿는다 — 조용히 쓰지 않는다
            # 받침판 검사는 **가드로만** 남긴다. 해를 버리면 IK 가 다른 가지로 튀고,
            # 그게 `down` 1.96 rad 점프의 정체였다 (웹 클로드 v21 회신 §Q2).
            # 팔꿈치↑ 에서는 발동하지 않아야 정상이다 — 발동하면 그 자체가 신호다.
            if not self._above_deck(q):
                self._deck_warn = getattr(self, "_deck_warn", 0) + 1
                if self._deck_warn in (1, 20, 200):
                    self.say(f"[받침판 경고] 팔꿈치↑ 인데 받침판에 닿는 해가 나왔다 "
                             f"({self._deck_warn}번째) q={np.round(q, 3).tolist()}")
            d = float(np.max(np.abs(q[:len(seed)] - seed)))
            if lim <= 0 or d <= lim:
                return q, True
            if best is None or d < best[1]:
                best = (q, d)
        if best is not None:
            return best[0], True          # 전부 멀면 그중 가장 가까운 해를 쓴다
        return np.asarray(seed, float), False

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

    # ---------------------------------------------------------------- 계획
    def plan_job(self, book, place_center_world):
        """책 하나를 집어, 꽂힌 뒤 AABB 중심이 place_center_world 가 되도록 꽂는 경로. 홈 → 홈"""
        moved = self.refresh_base()
        if moved > 0.01:
            self.say(f"팔 베이스가 {moved*100:.1f}cm 움직였다 — IK 기준을 다시 잡았다")
        bb = self.aabb(book); bc = (bb[:3] + bb[3:]) / 2
        T, Lb, W = self.dims.get(book, (self.T, self.L, self.W))
        DOWN, HORIZ = self.DOWN, self.HORIZ
        self.fit_bookends(float(place_center_world[0]), T)
        # **여기서부터 경유점은 팔이 보는 방향 기준으로 조립한다** (x 칸 방향, y 서가 방향).
        # 예전에는 월드 x/y 를 직접 썼고, 그래서 로봇이 돌아서면 경로가 통째로 깨졌다.
        # yaw 0 이면 예전 값과 완전히 같다 (원점만 옮긴 것이라 왕복하면 그대로 돌아온다).
        _pc = self.yaw_to_arm(place_center_world)
        place_x = float(_pc[0])
        y_front = float(_pc[1]) - W / 2 - MEASURED_INSET       # 꽂힌 책 중심 → 서가 앞면
        # 꽂힌 책 중심 → 칸 바닥. 목표 z 는 표준 책 높이를 가정하고 오므로, 책 높이가 다르면
        # 그대로 쓰면 책이 칸 바닥에서 뜨거나 파묻힌다. 같은 칸으로 볼 수 있으면 실제 칸 바닥에 맞춘다.
        floor_z = float(_pc[2]) - Lb / 2
        if abs(floor_z - self.shelf_floor_z) < 0.07:
            floor_z = self.shelf_floor_z
        else:
            # 명령이 가리키는 단과 SHELF_ROW_Z 가 다르다. 책은 명령대로 꽂히지만
            # **survey() 가 그 책을 '바닥/기타' 로 분류해 성공을 실패로 보고**하고,
            # 북엔드도 엉뚱한 높이에 만들어진다. SIM_SHELF_ROW_Z 를 안 넘겼을 때 생긴다
            # (2026-09-21 점검). 조용히 지나가지 않게 한다.
            self.say(f"[경고] 명령이 가리키는 선반판 {floor_z:.3f} 가 "
                     f"SIM_SHELF_ROW_Z({self.shelf_floor_z:.3f}) 와 {abs(floor_z-self.shelf_floor_z)*100:.1f} cm 다르다. "
                     f"꽂기는 되지만 **성공을 실패로 보고**하고 북엔드 높이가 틀린다 — "
                     f"SIM_SHELF_ROW_Z={floor_z:.3f} 로 다시 띄울 것")
        spine_final = y_front + SPINE_INSET
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
        segs = {}; q = self.q_home
        for name, wps in order:
            if name in JOINT_SEGS:
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
                "floor_z": floor_z, "book": book, "dims": (T, Lb, W)}, 0, ""

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
        o = T / 2 + GRIP_CLEAR
        return Sequence(name, [
            named(SetGripper(o), "approach"), JointPath("approach", s["approach"][1:], 0.5),
            JointPath("down", s["down"][1:], 0.25),
            # 얇은 책은 4 mm 를 그대로 조이면 손가락이 책을 밀어낸다 → 두께에 비례해 줄인다
            named(SetGripper(max(0.0, T / 2 - min(0.004, 0.15 * T)), settle_s=0.6), "grip"),
            Call("attach", lambda: self.attach(book)),
            Call("attach", lambda: self.trace(book, "파지")),
            JointPath("lift", s["lift"][1:], 0.25),
            JointPath("carry_rotate", s["carry_rotate"][1:], 0.35), JointPath("wedge", s["wedge"][1:], 0.35),
            Call("detach", self.detach), named(SetGripper(o, settle_s=0.4), "release"), named(Wait(0.4), "release"),
            Call("release", lambda: self.trace(book, "놓음")),
            JointPath("back", s["back"][1:], 0.3), named(SetGripper(0.0, settle_s=0.4), "touch"),
            JointPath("touch", s["touch"][1:], 0.3), JointPath("push", s["push"][1:], 0.12), named(Wait(0.3), "push"),
            JointPath("retreat", s["retreat"][1:], 0.35), named(SetGripper(o, settle_s=0.3), "retreat"),
            JointPath("return", s["return"][1:], 0.5),
        ])

    def trace(self, book, tag):
        """문제를 찾을 때 책 위치를 단계별로 남긴다 (책 종류가 섞이면 실패 지점이 달라진다)"""
        b = self.aabb(book); c = (b[:3] + b[3:]) / 2
        self.say(f"  [{tag}] {book.rsplit('/', 1)[1]} 중심 {np.round(c, 3).tolist()} "
                 f"크기 {np.round([b[3] - b[0], b[4] - b[1], b[5] - b[2]], 3).tolist()}")

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
        if not getattr(self, "_tray_anchor", None):
            return
        # **작업 중에는 아무것도 옮기지 않는다.** 팔이 움직이면 그 반작용으로 베이스가
        # 흔들리는데, 그때마다 트레이를 순간이동시키면 그 위의 책이 밀려 파지가 깨진다
        # (실측: 책만 얼렸을 때 상승 3.2~4.1cm, 기준 5.0cm 미달).
        # 작업 중에는 AMR 이 서 있으므로 따라갈 것도 없다.
        if getattr(self, "job_active", False):
            return
        ap, aq = SingleXFormPrim(self._tray_anchor).get_world_pose()
        Ra = R_from_quat(np.asarray(aq, float))
        # 복귀 보정이 요청했으면 **트레이는 두고 관계만 다시 잡는다** (rebase_tray_to 참조)
        rebase = getattr(self, "_rebase_tray_to", None)
        if rebase is not None:
            self._rebase_tray_to = None
            tp, tq = rebase
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
        self._held_book = book      # follow_tray() 가 이 책은 안 건드린다
        getattr(self, "_tray_books", {}).pop(book, None)   # 놓은 뒤에는 서가에 있어야 한다
        if book in self.upright_q:
            p_now = SingleXFormPrim(book).get_world_pose()[0]
            SingleXFormPrim(book).set_world_pose(np.asarray(p_now, float), self.upright_q[book])
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose(); bp, bq = SingleXFormPrim(book).get_world_pose()
        Rh = R_from_quat(hq); rel_p = Rh.T @ (np.asarray(bp) - np.asarray(hp)); rel_q = quat_from_R(Rh.T @ R_from_quat(bq))
        if GRASP_KINEMATIC:
            # **책을 키네마틱으로 만들어 손에 붙여 옮긴다.**
            # 고정 조인트는 만들어지긴 하는데 접촉력에 밀린다 — 들어 올리는 0.7초 동안
            # 손 기준으로 4.7cm 기울어져 `406` 이 났다 (2026-09-21 실측, 어긋남 곡선이
            # 0→4.7cm 로 매끈하게 자라다 평형에서 멈춘다 = 구속이 아니라 접촉이 이긴다).
            # 키네마틱은 물리가 밀지 못한다. 놓을 때 detach() 가 다시 동적으로 돌린다.
            UsdPhysics.RigidBodyAPI.Apply(
                self.stage.GetPrimAtPath(book)).CreateKinematicEnabledAttr().Set(True)
            self._held_rel = (rel_p, Rh.T @ R_from_quat(np.asarray(bq, float)))
            return
        j = UsdPhysics.FixedJoint.Define(self.stage, GRASP_JOINT)
        j.CreateBody0Rel().SetTargets([HAND_LINK]); j.CreateBody1Rel().SetTargets([book])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        j.CreateExcludeFromArticulationAttr().Set(True)

    def detach(self):
        book = self._held_book
        self._held_book = None
        self._held_rel = None
        if book and GRASP_KINEMATIC:
            # 다시 동적으로 — 놓은 뒤에는 선반에 닿아 멈춰야 한다
            _bp = self.stage.GetPrimAtPath(book)
            if _bp.IsValid():
                UsdPhysics.RigidBodyAPI.Apply(_bp).CreateKinematicEnabledAttr().Set(False)
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

    def book_in_hand(self, b):
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose()
        return R_from_quat(hq).T @ (self.book_origin(b) - np.asarray(hp, float))

    def verify(self, plan):
        """꽂힌 책 판정 (multi_book 과 같은 기준)"""
        bb = self.aabb(plan["book"])
        _, Lb, W = plan.get("dims", (self.T, self.L, self.W))
        checks = {
            "upright": abs((bb[5] - bb[2]) - Lb) < 0.02,
            "depth": abs((bb[4] - bb[1]) - W) < 0.02,
            "spine": abs(bb[1] - plan["spine_final"]) < 0.015,
            "x": abs((bb[0] + bb[3]) / 2 - plan["place_x"]) < 0.015,
            "floor": abs(bb[2] - plan["floor_z"]) < 0.03,
        }
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
            where = ("서가" if abs(z0 - self.shelf_floor_z) < 0.05
                     else "트레이" if abs(z0 - self.tray_floor_z) < 0.06
                     else "바닥/기타")
            # **자리마다 '바른 자세'가 다르다.**
            #   트레이: 책등이 위 → **깊이(W)** 가 수직
            #   서가  : 세워 꽂음 → **높이(L)** 가 수직
            # 이걸 하나로 보면 트레이의 정상 자세를 '쓰러짐' 으로 잘못 읽는다 (2026-09-20).
            want = W if where == "트레이" else Lb
            ok_pose = abs(size[2] - want) < 0.025
            out.append({"book": b.rsplit("/", 1)[-1], "바른자세": bool(ok_pose),
                        "위치": where, "밑면z": round(z0, 4),
                        "기대수직": round(float(want), 3),
                        "크기": [round(float(v), 3) for v in size]})
        bad = [o for o in out if not o["바른자세"] or o["위치"] == "바닥/기타"]
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
