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
SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
BOOK_SRC = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
ARM_JOINTS = BOT.arm_joints
FINGERS = BOT.grip_joints
DECK_Z = 0.286
TIP_DOWN = 0.035            # 책등 윗면에서 손끝이 내려가 잡는 깊이
GRIP_CLEAR = 0.005
SPINE_INSET = 0.02          # 계획상 최종 책등이 서가 앞면에서 들어가는 거리
MEASURED_INSET = 0.024      # 실측 최종 책등 위치 (밀기 후). 꽂힌 책 AABB 중심 → 서가 앞면 역산에 쓴다
DIV_H, DIV_T, DIV_GAP = 0.07, 0.01, 0.003
VEL_LIMIT = np.array(BOT.vel_limit)             # URDF 실측 (로봇별, 프로파일에서)
MAX_STEP = 0.12             # 계획 인접점 최대 관절 변화 (초과 = 불연속, M402)
GRASP_JOINT = "/World/bs_grasp_joint"


def named(primitive, name):
    """시뮬 상태 보고용 동작 이름 (book_placer.SIM_PHASE_ORDER 와 같은 이름)"""
    primitive.name = name
    return primitive


class JointPath(Primitive):
    """미리 계획한 관절 경유점을 일정한 관절 속도로 따라간다 (실행 중 IK 없음)"""

    def __init__(self, name, qs, speed=0.35):
        length = sum(float(np.max(np.abs(b - a))) for a, b in zip(qs[:-1], qs[1:]))
        super().__init__(max(4.0, length / speed * 2.0 + 3.0))
        self.name = name
        self.qs = [np.asarray(q, float) for q in qs]
        self.speed = speed

    def on_start(self, ctx):
        self._pts = [ctx.backend.get_joint_positions()] + self.qs
        self._i = 0
        self._target = self._pts[0].copy()

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
        if self._i >= len(self._pts) - 1 and \
                float(np.max(np.abs(ctx.backend.get_joint_positions() - self._pts[-1]))) <= 0.02:
            return Status.SUCCEEDED
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
        for p in ([s[0] for s in sources]
                  + ([str(c.GetPath()) for c in st.GetPrimAtPath("/World/fixtures").GetChildren()]
                     if st.GetPrimAtPath("/World/fixtures").IsValid() else [])):
            st.GetPrimAtPath(p).SetActive(False)

        # 트레이
        self.tray = "/World/bs_tray"
        add_reference_to_stage(tray_usd, self.tray)
        SingleXFormPrim(self.tray).set_world_pose(np.array([tray_center[0], tray_center[1], DECK_Z]), np.array([1.0, 0, 0, 0]))
        pitch = nslots = floor_top = None
        for p in Usd.PrimRange(st.GetPrimAtPath(self.tray)):
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
            for a in p.GetAttributes():
                n = a.GetName()
                if n.endswith("tray_pitch"): pitch = float(a.Get())
                if n.endswith("tray_slots"): nslots = int(a.Get())
                if n.endswith("floor_top_z"): floor_top = float(a.Get())
        slot_x = [tray_center[0] + (i + 0.5 - nslots / 2) * pitch for i in range(nslots)]

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
            target = np.array([slot_x[i], tray_center[1], DECK_Z + floor_top + (b[5] - b[2]) / 2 + 0.002])
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
        self.robot = SingleArticulation(prim_path=R, name="rf")
        link0_x = float(SingleXFormPrim(BASE_LINK).get_world_pose()[0][0])
        self.tray_floor_z = DECK_Z + floor_top
        self.slot_x = slot_x
        self.tray_y = float(tray_center[1])
        self.T, self.L, self.W = self.dims[self.books[0]]   # 기본값 (홈 자세·여러 종류일 때의 대표값)
        self.T_max = max(d[0] for d in self.dims.values())
        self.W_max = max(d[2] for d in self.dims.values())
        self.shelf_front_y = float(shelf[1])

        # 북엔드: 1차 고정 칸마다 한 쌍 (세운 책이 스스로 넘어지는 것 방지 — 칸막이 교훈)
        floor_z = self.shelf_floor_z = 0.355 * 1.4
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

        l0p, l0q = SingleXFormPrim(BASE_LINK).get_world_pose()
        self.l0p = np.asarray(l0p, float); self.Rl0 = R_from_quat(np.asarray(l0q, float))
        if BOT.lula[0] == "supported":
            cfg = interface_config_loader.load_supported_lula_kinematics_solver_config(BOT.lula[1])
        else:
            # Isaac 기본 지원 목록에 Doosan 이 없다 — descriptor·URDF 를 직접 준다
            cfg = {"robot_description_path": BOT.lula[1], "urdf_path": BOT.lula[2]}
        self.lula = LulaKinematicsSolver(**cfg)
        self.lula.set_robot_base_pose(l0p, l0q)
        self.ik = ArticulationKinematicsSolver(r, self.lula, BOT.ee_frame)

        ee_p, ee_R = self.ik.compute_end_effector_pose()
        hand_p = np.array(SingleXFormPrim(HAND_LINK).get_world_pose()[0])
        lf = np.array(SingleXFormPrim(R + "/" + BOT.finger_links[0]).get_world_pose()[0])
        rf = np.array(SingleXFormPrim(R + "/" + BOT.finger_links[1]).get_world_pose()[0])
        self._a_loc = np.round(ee_R.T @ ((ee_p - hand_p) / np.linalg.norm(ee_p - hand_p)))
        self._c_loc = np.round(ee_R.T @ ((rf - lf) / (np.linalg.norm(rf - lf) or 1.0)))
        self.DOWN = self.orientation([0, 0, -1], [1, 0, 0])
        self.HORIZ = self.orientation([0, 1, 0], [1, 0, 0])

        # 로봇마다 자세·그리퍼 값이 다르다. franka 는 arm.yaml(검증 완료), 나머지는 arm_<이름>.yaml
        _cfg_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
        _cfg = os.path.join(_cfg_dir, "arm.yaml" if BOT.name == "franka" else f"arm_{BOT.name}.yaml")
        conf = yaml.safe_load(open(_cfg))
        conf["gripper"]["tolerance_m"] = 0.006; conf["tolerance"]["position_m"] = 0.008; conf["tolerance"]["joint_rad"] = 0.03
        self.conf = conf
        self.arm = ArmController(_Backend(self), conf)

        tray_top = DECK_Z + floor_top + self.W_max
        self.home_tip = np.array([tray_center[0], tray_center[1], tray_top + 0.30])
        seed = r.get_joint_positions()[self.idx_arm].copy()
        seed[0] = math.atan2(tray_center[1] - l0p[1], tray_center[0] - l0p[0])
        self.q_home, ok = self.ik_joints(self.home_tip, self.DOWN, seed)
        self.open_tray = self.T_max / 2 + GRIP_CLEAR
        say(f"장면 준비: 트레이 칸 {nslots}개, 책 {len(self.books)}권, 원본 {len(sources)}종, 홈 IK {ok}")
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

    def aabb(self, p):
        self._cache.Clear()
        return np.array(compute_aabb(self._cache, p, include_children=True), float)

    def center(self, p):
        b = self.aabb(p)
        return (b[:3] + b[3:]) / 2

    def to_world(self, p_arm):
        return self.l0p + self.Rl0 @ np.asarray(p_arm, float)

    def to_arm(self, p_world):
        return self.Rl0.T @ (np.asarray(p_world, float) - self.l0p)

    def orientation(self, approach, closing):
        b_l = np.cross(self._a_loc, self._c_loc); a_w = np.array(approach, float); c_w = np.array(closing, float)
        Lm = np.stack([self._a_loc, self._c_loc, b_l], axis=1); Wm = np.stack([a_w, c_w, np.cross(a_w, c_w)], axis=1)
        return quat_from_R(Wm @ Lm.T)

    def ik_joints(self, target, ori, seed):
        q, ok = self.lula.compute_inverse_kinematics(BOT.ee_frame, np.asarray(target, float), np.asarray(ori, float),
                                                     np.asarray(seed, float), 0.004, 0.05)
        return np.asarray(q, float), bool(ok)

    def plan_path(self, waypoints, seed, step_m=0.005, step_rad=0.035):
        qs = [np.asarray(seed, float)]; prev_p = prev_q = None; worst = 0.0
        for p_, q_ in waypoints:
            p_ = np.asarray(p_, float); q_ = np.asarray(q_, float)
            n = 1 if prev_p is None else max(1, int(math.ceil(np.linalg.norm(p_ - prev_p) / step_m)),
                                             int(math.ceil(quat_angle(prev_q, q_) / step_rad)))
            for i in range(1, n + 1):
                t = i / n
                tp = p_ if prev_p is None else prev_p + (p_ - prev_p) * t
                tq = q_ if prev_q is None else slerp(prev_q, q_, t)
                sol, ok = self.ik_joints(tp, tq, qs[-1])
                if not ok:
                    return None, worst, f"IK 실패 {np.round(tp, 3).tolist()}"
                worst = max(worst, float(np.max(np.abs(sol - qs[-1])))); qs.append(sol)
            prev_p, prev_q = p_, q_
        return qs, worst, ""

    def book_on_tray_near(self, p_world, tol=0.03):
        """트레이 위에 있는 책 중 AABB 중심이 p_world 에서 tol 안인 것"""
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
        bb = self.aabb(book); bc = (bb[:3] + bb[3:]) / 2
        T, Lb, W = self.dims.get(book, (self.T, self.L, self.W))
        DOWN, HORIZ = self.DOWN, self.HORIZ
        self.fit_bookends(float(place_center_world[0]), T)
        place_x = float(place_center_world[0])
        y_front = float(place_center_world[1]) - W / 2 - MEASURED_INSET       # 꽂힌 책 중심 → 서가 앞면
        # 꽂힌 책 중심 → 칸 바닥. 목표 z 는 표준 책 높이를 가정하고 오므로, 책 높이가 다르면
        # 그대로 쓰면 책이 칸 바닥에서 뜨거나 파묻힌다. 같은 칸으로 볼 수 있으면 실제 칸 바닥에 맞춘다.
        floor_z = float(place_center_world[2]) - Lb / 2
        if abs(floor_z - self.shelf_floor_z) < 0.07:
            floor_z = self.shelf_floor_z
        spine_final = y_front + SPINE_INSET
        grasp = np.array([bc[0], bc[1], bb[5] - TIP_DOWN])
        if book in self.grasp_local:
            c_loc, up_loc, hz = self.grasp_local[book]
            p_w, q_w = SingleXFormPrim(book).get_world_pose()
            top = np.asarray(p_w, float) + R_from_quat(np.asarray(q_w, float)) @ (c_loc + up_loc * hz)
            grasp = np.array([top[0], top[1], top[2] - TIP_DOWN])   # 책 자신의 윗면 중심에서 내려간다
        pre = grasp + np.array([0, 0, 0.13]); lift = grasp + np.array([0, 0, 0.17])
        grip_z = floor_z + Lb / 2 + 0.004
        y_pre = y_front - (W - TIP_DOWN) - 0.03
        transfer = np.array([place_x, y_pre - 0.02, grip_z + 0.06]); pre_ins = np.array([place_x, y_pre, grip_z + 0.01])
        wedge = np.array([place_x, y_front + 0.10 - (W - TIP_DOWN), grip_z])
        back = wedge - np.array([0, TIP_DOWN + 0.025, 0])
        push_z = floor_z + Lb / 2
        touch = np.array([place_x, wedge[1] - TIP_DOWN - 0.008, push_z])
        push = np.array([place_x, spine_final + 0.002 - 0.010, push_z])
        retreat = np.array([place_x, y_front - 0.13, push_z])
        order = [("approach", [(self.home_tip, DOWN), (pre, DOWN)]), ("down", [(pre, DOWN), (grasp, DOWN)]),
                 ("lift", [(grasp, DOWN), (lift, DOWN)]),
                 ("carry_rotate", [(lift, DOWN), (transfer, DOWN), (pre_ins, HORIZ)]),
                 ("wedge", [(pre_ins, HORIZ), (wedge, HORIZ)]), ("back", [(wedge, HORIZ), (back, HORIZ)]),
                 ("touch", [(back, HORIZ), (touch, HORIZ)]), ("push", [(touch, HORIZ), (push, HORIZ)]),
                 ("retreat", [(push, HORIZ), (retreat, HORIZ)]), ("return", [(retreat, HORIZ), (self.home_tip, DOWN)])]
        segs = {}; q = self.q_home; worst_all = 0.0
        for name, wps in order:
            qs, worst, err = self.plan_path(wps, q)
            if qs is None:
                return None, 401, f"{name}: {err}"
            if worst > MAX_STEP:
                return None, 402, f"{name}: 인접점 관절변화 {worst:.3f} rad"
            segs[name] = qs; q = qs[-1]; worst_all = max(worst_all, worst)
        return {"segs": segs, "worst": worst_all, "spine_final": spine_final, "place_x": place_x,
                "floor_z": floor_z, "book": book, "dims": (T, Lb, W)}, 0, ""

    def fit_bookends(self, place_x, thickness):
        """북엔드 한 쌍을 이 책 두께에 맞춘다 (책마다 두께가 달라서 — 꽂기 전에만 옮긴다)"""
        key = min(self.bookends, key=lambda k: abs(k - place_x)) if self.bookends else None
        if key is None or abs(key - place_x) > 0.03:
            return
        pL, pR, y, z = self.bookends[key]
        half = thickness / 2 + DIV_GAP + DIV_T / 2
        q = np.array([1.0, 0.0, 0.0, 0.0])
        SingleXFormPrim(pL).set_world_pose(np.array([key - half, y, z]), q)
        SingleXFormPrim(pR).set_world_pose(np.array([key + half, y, z]), q)

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
        self.robot.apply_action(ArticulationAction(joint_positions=q))
        # 기본 상태로도 저장해 두면 world.reset() 뒤에도 같은 자세로 돌아온다
        try:
            self.robot.set_joints_default_state(positions=q)
        except Exception:
            pass
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

    def attach(self, book):
        """파지 순간 손과 책을 고정 조인트로 붙인다 (지침 허용 방식, 마찰 파지는 손목 회전에서 실패 확인)"""
        # 트레이에서 몇 도 기운 채로 잡으면 그 기울기가 그대로 서가까지 간다 (꽂힘 판정 실패).
        # 고정 조인트로 붙이기 직전에 세운 자세로 맞춘다 (자세만, 위치는 그대로).
        if book in self.upright_q:
            p_now = SingleXFormPrim(book).get_world_pose()[0]
            SingleXFormPrim(book).set_world_pose(np.asarray(p_now, float), self.upright_q[book])
        hp, hq = SingleXFormPrim(HAND_LINK).get_world_pose(); bp, bq = SingleXFormPrim(book).get_world_pose()
        Rh = R_from_quat(hq); rel_p = Rh.T @ (np.asarray(bp) - np.asarray(hp)); rel_q = quat_from_R(Rh.T @ R_from_quat(bq))
        j = UsdPhysics.FixedJoint.Define(self.stage, GRASP_JOINT)
        j.CreateBody0Rel().SetTargets([HAND_LINK]); j.CreateBody1Rel().SetTargets([book])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        j.CreateExcludeFromArticulationAttr().Set(True)

    def detach(self):
        if self.stage.GetPrimAtPath(GRASP_JOINT).IsValid():
            self.stage.RemovePrim(GRASP_JOINT)

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
        s.robot.apply_action(ArticulationAction(joint_positions=q))

    def set_joint_targets(self, positions):
        self._apply(positions)

    def get_ee_pose(self):
        p, Rm = self.s.ik.compute_end_effector_pose()
        return np.asarray(p, float), quat_from_R(Rm)

    def compute_ik(self, position, orientation, seed=None):
        q, ok = self.s.ik_joints(position, orientation, self.get_joint_positions())
        return (q, True) if ok else (np.zeros(7), False)

    def set_gripper_width(self, w):
        self._grip = float(w)
        self._apply()

    def get_gripper_width(self):
        return BOT.grip_width(self.s.robot.get_joint_positions()[self.s.idx_fing])
