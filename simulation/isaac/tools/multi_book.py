"""트레이의 책 여러 권을 차례로 집어 서가 1번 칸의 서로 다른 위치에 도서관식으로 꽂는다.

한 권짜리(pick_place_book.py)에서 검증한 방식을 작업 단위(job)로 나눠 반복한다.
- 파지: 책등을 위에서, 손가락은 트레이 칸 간격 방향(X)으로 닫힘
- 꽂기: 손목을 세워 앞마구리부터 끼우기 → 놓기 → 뒤로 → 그리퍼 닫고 책등 밀기 → 후퇴 → 홈
- 경로는 실행 전에 5mm·2° 간격으로 미리 풀고 관절 보간만 실행한다 (튐 방지)

레벨 파일은 수정하지 않는다. 트레이·책·북엔드는 메모리에서만 배치한다.

GPU PC:
    ~/isaacsim/python.sh ~/arm/isaac/multi_book.py --plan-only          # 칸×위치 조합 IK·연속성만 확인
    ~/isaacsim/python.sh ~/arm/isaac/multi_book.py --books 3 --out /tmp/mb
"""
import argparse
import json
import math
import os
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
import _paths  # noqa: E402
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=_paths.default_usd())
ap.add_argument("--tray", default=_paths.default_tray())
ap.add_argument("--tray-center", type=float, nargs=2, default=[2.36, -2.94])
ap.add_argument("--books", type=int, default=3, help="트레이에 세울 책 수(칸 0 부터)")
ap.add_argument("--jobs", type=int, default=None, help="실행할 작업 수 (기본: 책 수)")
ap.add_argument("--place-dx", type=float, nargs="+", default=[-0.35, -0.27, -0.43, -0.19, -0.51, -0.11],
                help="꽂을 위치 x = 팔 원점 x + 값. 순서대로 작업에 배정 (간격 8cm)")
ap.add_argument("--out", default="/tmp/multi_book")
ap.add_argument("--plan-only", action="store_true")
ap.add_argument("--capture-every", type=int, default=15)
ap.add_argument("--max-steps", type=int, default=20000)
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import cv2
import numpy as np
import yaml
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.sensors.camera import Camera

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers"))
from arm_primitives import ArmController, Primitive, SetGripper, Sequence, Status, Wait  # noqa: E402

os.makedirs(f"{args.out}/frames", exist_ok=True)
LOG = []
def say(m):
    LOG.append(m); sys.stderr.write(f"### {m}\n"); sys.stderr.flush()

R = "/World/ridgeback_franka"
SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
BOOK_SRC = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
DECK_Z = 0.286
TIP_DOWN = 0.035            # 책등 윗면에서 손끝이 내려가 잡는 깊이
GRIP_CLEAR = 0.005          # 벌렸을 때 책 표면과 손가락 사이 한쪽 여유
DIV_H, DIV_T, DIV_GAP = 0.07, 0.01, 0.003
VEL_LIMIT = np.array([2.175] * 4 + [2.61] * 3)   # URDF 실측
MAX_STEP = 0.12             # 계획 인접점 최대 관절 변화 (초과 = 불연속)

# ================================================================== 장면 구성
open_stage(args.usd); app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
def aabb(p):
    cache.Clear(); return np.array(compute_aabb(cache, p, include_children=True), float)

# 책 원본 참조·자세
src = stage.GetPrimAtPath(BOOK_SRC)
book_ref = None
for spec in src.GetPrimStack():
    for r in list(spec.referenceList.GetAddedOrExplicitItems()) + list(spec.payloadList.GetAddedOrExplicitItems()):
        book_ref = r.assetPath
book_ref = os.path.normpath(os.path.join(os.path.dirname(stage.GetRootLayer().realPath), book_ref))
src_q = SingleXFormPrim(BOOK_SRC).get_world_pose()[1]
shelf = aabb(SHELF)

# v3 의 데크 책·칸막이·선반 북엔드는 이 시험에서 쓰지 않는다
for p in [BOOK_SRC] + ([str(c.GetPath()) for c in stage.GetPrimAtPath("/World/fixtures").GetChildren()]
                       if stage.GetPrimAtPath("/World/fixtures").IsValid() else []):
    stage.GetPrimAtPath(p).SetActive(False)

# 트레이
TRAY = "/World/mb_tray"
add_reference_to_stage(args.tray, TRAY)
SingleXFormPrim(TRAY).set_world_pose(np.array([args.tray_center[0], args.tray_center[1], DECK_Z]), np.array([1.0, 0, 0, 0]))
pitch = nslots = floor_top = None
for p in Usd.PrimRange(stage.GetPrimAtPath(TRAY)):
    if p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
    for a in p.GetAttributes():
        n = a.GetName()
        if n.endswith("tray_pitch"): pitch = float(a.Get())
        if n.endswith("tray_slots"): nslots = int(a.Get())
        if n.endswith("floor_top_z"): floor_top = float(a.Get())
slot_x = [args.tray_center[0] + (i + 0.5 - nslots / 2) * pitch for i in range(nslots)]
say(f"트레이 칸 {nslots}개 간격 {pitch}  칸 x {[round(v, 3) for v in slot_x]}")

# 책
BOOKS = []
for i in range(min(args.books, nslots)):
    path = f"/World/mb_books/book_{i}"
    add_reference_to_stage(book_ref, path)
    prim = stage.GetPrimAtPath(path)
    UsdPhysics.RigidBodyAPI.Apply(prim); UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(0.5)
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("convexHull")
    xf = SingleXFormPrim(path); xf.set_world_pose(np.array([0.0, 0.0, 5.0 + i]), src_q)
    b = aabb(path); c = (b[:3] + b[3:]) / 2
    target = np.array([slot_x[i], args.tray_center[1], DECK_Z + floor_top + (b[5] - b[2]) / 2 + 0.002])
    pos, q = xf.get_world_pose(); xf.set_world_pose(np.array(pos) + (target - c), q)
    BOOKS.append(path)

# 로봇·IK
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
robot = SingleArticulation(prim_path=R, name="rf")
link0_x = float(SingleXFormPrim(R + "/panda_link0").get_world_pose()[0][0])

# 선반 북엔드 (작업마다 한 쌍). 위치는 책 치수를 알아야 정해지므로 먼저 치수를 잰다
b0 = aabb(BOOKS[0]); T = b0[3] - b0[0]; Lb = b0[4] - b0[1]; W = b0[5] - b0[2]
y_front = shelf[1]
floor_z = 0.355 * 1.4                     # v3: 선반 z 1.4배
spine_final = y_front + 0.02
place_xs = [link0_x + dx for dx in args.place_dx]
UsdGeom.Scope.Define(stage, "/World/mb_bookends")
for k, px in enumerate(place_xs[: (args.jobs or len(BOOKS))]):
    y0, y1 = spine_final + 0.02, spine_final + W
    for side, cx in (("L", px - T / 2 - DIV_GAP - DIV_T / 2), ("R", px + T / 2 + DIV_GAP + DIV_T / 2)):
        cube = UsdGeom.Cube.Define(stage, f"/World/mb_bookends/b{k}_{side}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(cx, (y0 + y1) / 2, floor_z + DIV_H / 2))
        cube.AddScaleOp().Set(Gf.Vec3f(DIV_T, y1 - y0, DIV_H))
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.25)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
say(f"책 치수 두께 {T:.3f} 길이 {Lb:.3f} 폭 {W:.3f}  꽂을 x {[round(v, 3) for v in place_xs]}")

cams = {}
if not args.plan_only:
    ctr = np.array([2.55, -2.8, 0.55])
    for name, (eye, tgt) in {"side": ((1.35, -4.35, 1.65), (2.45, -2.80, 0.55)),
                             "shelf": ((2.40, -3.35, 1.05), (2.40, -2.45, 0.62))}.items():
        cams[name] = Camera(prim_path=f"/World/mb_cam_{name}", resolution=(960, 540))
        set_camera_view(eye=np.array(eye), target=np.array(tgt), camera_prim_path=f"/World/mb_cam_{name}")

world.reset(); robot.initialize()
for c in cams.values():
    c.initialize()
idx_arm = [robot.get_dof_index(j) for j in ARM_JOINTS]
idx_fing = [robot.get_dof_index(j) for j in FINGERS]
base_idx = [robot.get_dof_index(j) for j in robot.dof_names if j.startswith("dummy_base")]
base_hold = robot.get_joint_positions()[base_idx]
for _ in range(120):
    world.step(render=False)

l0p, l0q = SingleXFormPrim(R + "/panda_link0").get_world_pose()
lula = LulaKinematicsSolver(**interface_config_loader.load_supported_lula_kinematics_solver_config("Franka"))
lula.set_robot_base_pose(l0p, l0q)
ik = ArticulationKinematicsSolver(robot, lula, "right_gripper")

# ================================================================== 수학·계획 도구
def quat_from_R(M):
    w = math.sqrt(max(0.0, 1 + M[0, 0] + M[1, 1] + M[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + M[0, 0] - M[1, 1] - M[2, 2])) / 2, M[2, 1] - M[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] + M[1, 1] - M[2, 2])) / 2, M[0, 2] - M[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] - M[1, 1] + M[2, 2])) / 2, M[1, 0] - M[0, 1])
    return np.array([w, x, y, z])

def R_from_quat(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def slerp(q0, q1, t):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float); d = float(np.dot(q0, q1))
    if d < 0: q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(d); return (math.sin((1-t)*th) * q0 + math.sin(t*th) * q1) / math.sin(th)

def quat_angle(q0, q1):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))

# 그리퍼 좌표축 실측 (pick_place_book.py 와 같은 방법)
ee_p, ee_R = ik.compute_end_effector_pose()
hand_p = np.array(SingleXFormPrim(R + "/panda_hand").get_world_pose()[0])
lf = np.array(SingleXFormPrim(R + "/panda_leftfinger").get_world_pose()[0])
rf = np.array(SingleXFormPrim(R + "/panda_rightfinger").get_world_pose()[0])
a_loc = np.round(ee_R.T @ ((ee_p - hand_p) / np.linalg.norm(ee_p - hand_p)))
c_loc = np.round(ee_R.T @ ((rf - lf) / (np.linalg.norm(rf - lf) or 1.0)))
def orientation(approach, closing):
    b_l = np.cross(a_loc, c_loc); a_w = np.array(approach, float); c_w = np.array(closing, float)
    Lm = np.stack([a_loc, c_loc, b_l], axis=1); Wm = np.stack([a_w, c_w, np.cross(a_w, c_w)], axis=1)
    return quat_from_R(Wm @ Lm.T)
DOWN = orientation([0, 0, -1], [1, 0, 0])
HORIZ = orientation([0, 1, 0], [1, 0, 0])

def ik_joints(target, ori, seed):
    q, ok = lula.compute_inverse_kinematics("right_gripper", np.asarray(target, float), np.asarray(ori, float),
                                            np.asarray(seed, float), 0.004, 0.05)
    return np.asarray(q, float), bool(ok)

def plan_path(waypoints, seed, step_m=0.005, step_rad=0.035):
    """직교 경유점을 촘촘히 나눠 IK 를 앞 해에서 이어 풀어 관절 경로로 만든다. (qs, 최대변화, 오류)"""
    qs = [np.asarray(seed, float)]; prev_p = prev_q = None; worst = 0.0
    for p_, q_ in waypoints:
        p_ = np.asarray(p_, float); q_ = np.asarray(q_, float)
        n = 1 if prev_p is None else max(1, int(math.ceil(np.linalg.norm(p_ - prev_p) / step_m)),
                                         int(math.ceil(quat_angle(prev_q, q_) / step_rad)))
        for i in range(1, n + 1):
            t = i / n
            tp = p_ if prev_p is None else prev_p + (p_ - prev_p) * t
            tq = q_ if prev_q is None else slerp(prev_q, q_, t)
            sol, ok = ik_joints(tp, tq, qs[-1])
            if not ok:
                return None, worst, f"IK 실패 {np.round(tp, 3).tolist()}"
            worst = max(worst, float(np.max(np.abs(sol - qs[-1])))); qs.append(sol)
        prev_p, prev_q = p_, q_
    return qs, worst, ""

class JointPath(Primitive):
    """미리 계획한 관절 경유점을 일정한 관절 속도로 따라간다 (실행 중 IK 없음)"""
    def __init__(self, name, qs, speed=0.35):
        length = sum(float(np.max(np.abs(b - a))) for a, b in zip(qs[:-1], qs[1:]))
        super().__init__(max(4.0, length / speed * 2.0 + 3.0))
        self.name = name; self.qs = [np.asarray(q, float) for q in qs]; self.speed = speed
    def on_start(self, ctx):
        self._pts = [ctx.backend.get_joint_positions()] + self.qs; self._i = 0; self._target = self._pts[0].copy()
    def on_update(self, ctx):
        budget = self.speed * ctx.backend.dt
        while budget > 1e-9 and self._i < len(self._pts) - 1:
            nxt = self._pts[self._i + 1]; gap = float(np.max(np.abs(nxt - self._target)))
            if gap <= budget:
                self._target = nxt.copy(); self._i += 1; budget -= gap
            else:
                self._target = self._target + (nxt - self._target) * (budget / gap); budget = 0.0
        ctx.backend.set_joint_targets(self._target)
        if self._i >= len(self._pts) - 1 and float(np.max(np.abs(ctx.backend.get_joint_positions() - self._pts[-1]))) <= 0.02:
            return Status.SUCCEEDED
        return Status.RUNNING

class Call(Primitive):
    def __init__(self, name, fn):
        super().__init__(timeout_s=1.0); self.name = name; self._fn = fn
    def on_update(self, ctx):
        self._fn(); return Status.SUCCEEDED

GRASP_JOINT = "/World/mb_grasp_joint"
def attach(book):
    """파지 순간 손과 책을 고정 조인트로 붙인다 (지침 허용 방식, 마찰 파지는 손목 회전에서 실패 확인됨)"""
    hp, hq = SingleXFormPrim(R + "/panda_hand").get_world_pose(); bp, bq = SingleXFormPrim(book).get_world_pose()
    Rh = R_from_quat(hq); rel_p = Rh.T @ (np.asarray(bp) - np.asarray(hp)); rel_q = quat_from_R(Rh.T @ R_from_quat(bq))
    j = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT)
    j.CreateBody0Rel().SetTargets([R + "/panda_hand"]); j.CreateBody1Rel().SetTargets([book])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
    j.CreateExcludeFromArticulationAttr().Set(True)

def detach():
    if stage.GetPrimAtPath(GRASP_JOINT).IsValid():
        stage.RemovePrim(GRASP_JOINT)

class IsaacBackend:
    def __init__(self): self._grip = 0.035
    @property
    def dt(self): return 1.0 / 60.0
    def get_joint_positions(self): return robot.get_joint_positions()[idx_arm]
    def set_joint_targets(self, positions):
        q = robot.get_joint_positions().copy(); q[idx_arm] = np.asarray(positions, float)[:7]; self.last_cmd = q[idx_arm].copy()
        q[idx_fing] = self._grip; q[base_idx] = base_hold
        robot.apply_action(ArticulationAction(joint_positions=q))
    def get_ee_pose(self):
        p, Rm = ik.compute_end_effector_pose(); return np.asarray(p, float), quat_from_R(Rm)
    def compute_ik(self, position, orientation, seed=None):
        q, ok = ik_joints(position, orientation, self.get_joint_positions()); return (q, True) if ok else (np.zeros(7), False)
    def set_gripper_width(self, w):
        self._grip = float(w); q = robot.get_joint_positions().copy(); q[idx_fing] = self._grip; q[base_idx] = base_hold
        robot.apply_action(ArticulationAction(joint_positions=q))
    def get_gripper_width(self): return float(np.mean(robot.get_joint_positions()[idx_fing]))

conf = yaml.safe_load(open(os.path.expanduser("~/arm/arm_config.yaml")))
conf["gripper"]["tolerance_m"] = 0.006; conf["tolerance"]["position_m"] = 0.008; conf["tolerance"]["joint_rad"] = 0.03
arm = ArmController(IsaacBackend(), conf)

# ================================================================== 작업 계획
tray_top = DECK_Z + floor_top + W
home_tip = np.array([args.tray_center[0], args.tray_center[1], tray_top + 0.30])
seed = robot.get_joint_positions()[idx_arm].copy()
seed[0] = math.atan2(args.tray_center[1] - l0p[1], args.tray_center[0] - l0p[0])
q_home, ok_home = ik_joints(home_tip, DOWN, seed)
say(f"데크(트레이) 홈 자세 IK {ok_home}: {np.round(q_home, 3).tolist()}")
OPEN_TRAY = T / 2 + GRIP_CLEAR           # 트레이에서는 옆 칸 책을 치지 않게 조금만 벌린다

def plan_job(book, place_x):
    """책 하나를 집어 place_x 에 꽂는 전체 경로. 모두 홈에서 시작해 홈으로 돌아온다"""
    bb = aabb(book); bc = (bb[:3] + bb[3:]) / 2
    grasp = np.array([bc[0], bc[1], bb[5] - TIP_DOWN])
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
    segs = {}
    order = [("approach", [(home_tip, DOWN), (pre, DOWN)]), ("down", [(pre, DOWN), (grasp, DOWN)]),
             ("lift", [(grasp, DOWN), (lift, DOWN)]),
             ("carry_rotate", [(lift, DOWN), (transfer, DOWN), (pre_ins, HORIZ)]),
             ("wedge", [(pre_ins, HORIZ), (wedge, HORIZ)]), ("back", [(wedge, HORIZ), (back, HORIZ)]),
             ("touch", [(back, HORIZ), (touch, HORIZ)]), ("push", [(touch, HORIZ), (push, HORIZ)]),
             ("retreat", [(push, HORIZ), (retreat, HORIZ)]), ("return", [(retreat, HORIZ), (home_tip, DOWN)])]
    q = q_home; worst_all = 0.0
    for name, wps in order:
        qs, worst, err = plan_path(wps, q)
        if qs is None:
            return None, 401, f"{name}: {err}"
        if worst > MAX_STEP:
            return None, 402, f"{name}: 인접점 관절변화 {worst:.3f} rad"
        segs[name] = qs; q = qs[-1]; worst_all = max(worst_all, worst)
    return {"segs": segs, "worst": worst_all, "spine_final": spine_final, "place_x": place_x, "book": book}, 0, ""

n_jobs = min(args.jobs or len(BOOKS), len(BOOKS), len(place_xs))
plans = []
say("----- 계획 (칸 → 꽂을 위치) -----")
for i in range(n_jobs):
    plan, code, err = plan_job(BOOKS[i], place_xs[i])
    say(f"작업 {i}: 칸 {i} (x {slot_x[i]:.3f}) → 서가 x {place_xs[i]:.3f}  "
        + (f"OK 최대 인접변화 {plan['worst']:.3f} rad" if plan else f"M{code} {err}"))
    plans.append((plan, code, err))

if args.plan_only:
    # PlaceBook 서버 설정용: arm_base_link(=panda_link0) 기준 AABB 중심 (FRAMES_CONTRACT)
    _Rl0 = R_from_quat(np.asarray(l0q, float)); _p0 = np.asarray(l0p, float)
    def _arm(pw): return np.round(_Rl0.T @ (np.asarray(pw, float) - _p0), 4).tolist()
    say(f"FRAME link0 world {np.round(_p0, 4).tolist()} quat_wxyz {np.round(np.asarray(l0q, float), 5).tolist()}")
    say(f"FRAME book dims thickness {T:.4f} length {Lb:.4f} width {W:.4f}")
    for i, b in enumerate(BOOKS):
        bb = aabb(b); say(f"FRAME tray_slot {i} aabb_center_arm {_arm((bb[:3] + bb[3:]) / 2)}")
    for px in place_xs:
        # 실측 최종 책 AABB: 책등이 서가 앞면에서 0.024 안쪽, 칸 바닥에 선 상태 (높이 Lb, 깊이 W)
        say(f"FRAME shelf_target x {px:.3f} aabb_center_arm {_arm((px, y_front + 0.024 + W / 2, floor_z + Lb / 2))}")
    say(f"FRAME shelf front_y_world {y_front:.4f} floor_z_world {floor_z:.4f}")
    # 모든 칸×위치 조합의 가능 여부 표
    say("----- 칸×위치 가능 여부 (O 가능 / 1 IK 실패 / 2 불연속) -----")
    for i in range(len(BOOKS)):
        row = []
        for px in place_xs:
            p_, c_, _ = plan_job(BOOKS[i], px)
            row.append("O" if p_ else str(c_)[-1])
        say(f"칸 {i} (x {slot_x[i]:.3f}): " + " ".join(f"{px:.2f}:{r}" for px, r in zip(place_xs, row)))
    app.close(); sys.exit(0)

# ================================================================== 실행
arm.enqueue(Sequence("start", [SetGripper(OPEN_TRAY)]))
from arm_primitives import MoveJoint  # noqa: E402
# 시작 자세 → 트레이 위 홈: 몸통을 크게 돌리므로 팔을 접은 채 돌린다 (arm_planning.tucked_joint_moves 설명)
from arm_planning import tucked_joint_moves  # noqa: E402
for q_via in tucked_joint_moves(robot.get_joint_positions()[idx_arm], q_home, conf["poses"]["stow"]):
    arm.enqueue(MoveJoint(q_via, speed_scale=0.6, timeout_s=15))
for i, (plan, code, err) in enumerate(plans):
    if plan is None:
        continue
    s = plan["segs"]; book = plan["book"]
    arm.enqueue(Sequence(f"j{i}", [
        SetGripper(OPEN_TRAY), JointPath("approach", s["approach"][1:], 0.5),
        JointPath("down", s["down"][1:], 0.25),
        SetGripper(max(0.0, T / 2 - 0.004), settle_s=0.6), Call("attach", lambda b=book: attach(b)),
        JointPath("lift", s["lift"][1:], 0.25),
        JointPath("carry_rotate", s["carry_rotate"][1:], 0.35), JointPath("wedge", s["wedge"][1:], 0.35),
        Call("detach", detach), SetGripper(OPEN_TRAY, settle_s=0.4), Wait(0.4),
        JointPath("back", s["back"][1:], 0.3), SetGripper(0.0, settle_s=0.4),
        JointPath("touch", s["touch"][1:], 0.3), JointPath("push", s["push"][1:], 0.12), Wait(0.3),
        JointPath("retreat", s["retreat"][1:], 0.35), SetGripper(OPEN_TRAY, settle_s=0.3),
        JointPath("return", s["return"][1:], 0.5),
    ]))

def book_pose(b):
    return np.asarray(SingleXFormPrim(b).get_world_pose()[0], float)
def book_in_hand(b):
    hp, hq = SingleXFormPrim(R + "/panda_hand").get_world_pose()
    return R_from_quat(hq).T @ (book_pose(b) - np.asarray(hp, float))

start_pose = {b: book_pose(b) for b in BOOKS}
q_prev = robot.get_joint_positions()[idx_arm].copy()
over80 = 0; peak = 0.0; last_phase = None; phase_log = []; frame = 0
job_err = {}; watch = {}; spikes = {}
step = 0
for step in range(args.max_steps):
    status = arm.update()
    world.step(render=(step % args.capture_every == 0))
    q_now = robot.get_joint_positions()[idx_arm]
    ratio = np.abs(q_now - q_prev) / world.get_physics_dt() / VEL_LIMIT; q_prev = q_now.copy()
    peak = max(peak, float(ratio.max())); over80 += int(ratio.max() > 0.8)
    ph = arm.phase
    if ratio.max() > 0.8:
        if over80 <= 12:
            if "_coll" not in globals():
                _coll = [pr for pr in stage.Traverse() if pr.HasAPI(UsdPhysics.CollisionAPI) and not str(pr.GetPath()).startswith(R)]
                _coll = [(str(pr.GetPath()), aabb(str(pr.GetPath()))) for pr in _coll]
            for ln in ("panda_link5", "panda_link6", "panda_link7", "panda_hand", "panda_leftfinger", "panda_rightfinger"):
                lb = aabb(f"{R}/{ln}")
                hits = [n for n, ob in _coll if np.all(lb[:3] < ob[3:]) and np.all(ob[:3] < lb[3:])]
                if hits: say(f"    {ln} AABB 겹침 {hits[:4]}  링크 {np.round(lb, 3).tolist()}")
            jm = int(ratio.argmax()); say(f"  초과 step {step} {ph} j{jm+1} {ratio[jm]*100:.0f}% 실제 {q_now[jm]:.3f} 명령 {getattr(arm.ctx.backend, 'last_cmd', q_now)[jm]:.3f}")
        k = f"{ph} j{int(ratio.argmax())+1}"; spikes[k] = [spikes.get(k, [0, 0])[0] + 1, max(spikes.get(k, [0, 0])[1], round(float(ratio.max()), 2))]
    job = ph.split(":")[0]; name = ph.split(":")[-1]
    ji = int(job[1:]) if job.startswith("j") and job[1:].isdigit() else None
    if ji is not None and name in ("carry_rotate", "wedge") and "rel0" in watch:
        dev = float(np.linalg.norm(book_in_hand(plans[ji][0]["book"]) - watch["rel0"]))
        if dev > 0.03 and ji not in job_err:
            job_err[ji] = (406, f"운반 중 손 안에서 책 {dev*100:.1f}cm 어긋남"); arm.cancel(); break
    if ph != last_phase:
        phase_log.append((step, ph)); prev = (last_phase or "").split(":")[-1]
        pji = int(last_phase.split(":")[0][1:]) if last_phase and last_phase.startswith("j") and last_phase.split(":")[0][1:].isdigit() else None
        if ji is not None and name == "lift":
            watch["z0"] = book_pose(plans[ji][0]["book"])[2]; watch["rel0"] = book_in_hand(plans[ji][0]["book"])
        if pji is not None and prev == "lift":
            rise = book_pose(plans[pji][0]["book"])[2] - watch.get("z0", 0)
            if rise < 0.08:
                job_err[pji] = (405, f"들어 올린 뒤 책 상승 {rise*100:.1f}cm"); arm.cancel(); break
        if pji is not None and prev == "retreat":
            watch["ry"] = None
        if ji is not None and name == "retreat":
            watch["ry"] = book_pose(plans[ji][0]["book"])[1]
        say(f"step {step:5d} → {ph}")
        last_phase = ph
    if step % args.capture_every == 0:
        for cn, c in cams.items():
            rgba = c.get_rgba()
            if rgba is not None and rgba.size:
                cv2.imwrite(f"{args.out}/frames/{cn}_{frame:05d}.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))
        frame += 1
    if status is Status.FAILED:
        say(f"실패: M{arm.error_code} {arm.error}")
        # 막힌 원인 기록: 팔 링크와 트레이·책의 AABB 겹침
        links = ["panda_link%d" % k for k in range(1, 8)] + ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        obst = [TRAY] + BOOKS
        hit = []
        for ln in links:
            lb = aabb(f"{R}/{ln}")
            for ob in obst:
                obb = aabb(ob)
                if np.all(lb[:3] < obb[3:]) and np.all(obb[:3] < lb[3:]):
                    hit.append((ln, ob.split("/")[-1]))
        say(f"실패 시점 링크-장애물 AABB 겹침: {hit or '없음'}  관절 {np.round(robot.get_joint_positions()[idx_arm],2).tolist()}")
        if ji is not None: job_err[ji] = (arm.error_code or 404, arm.error)
        break
    if arm.idle:
        say(f"전체 완료 step {step}"); break

for _ in range(120):
    world.step(render=False)

# ================================================================== 판정
results = []
for i, (plan, code, err) in enumerate(plans):
    if plan is None:
        results.append({"job": i, "success": False, "error_code": code, "error": err}); continue
    b = plan["book"]; bb = aabb(b)
    upright = abs((bb[5] - bb[2]) - Lb) < 0.02; depth = abs((bb[4] - bb[1]) - W) < 0.02
    spine_ok = abs(bb[1] - plan["spine_final"]) < 0.015
    x_ok = abs((bb[0] + bb[3]) / 2 - plan["place_x"]) < 0.015
    floor_ok = abs(bb[2] - floor_z) < 0.03
    ok = upright and depth and spine_ok and x_ok and floor_ok and i not in job_err
    ec = job_err.get(i, (0, ""))[0] if not ok and i in job_err else (0 if ok else 409)
    results.append({"job": i, "success": bool(ok), "error_code": ec, "error": job_err.get(i, (0, ""))[1],
                    "aabb": np.round(bb, 3).tolist(), "upright": bool(upright), "spine_y": round(float(bb[1]), 3),
                    "x_center": round(float((bb[0] + bb[3]) / 2), 3)})
    say(f"작업 {i}: {'성공' if ok else '실패'}  M{ec}  세움 {upright} 깊이 {depth} 책등 y {bb[1]:.3f}(목표 {plan['spine_final']:.3f}) "
        f"x {((bb[0]+bb[3])/2):.3f}(목표 {plan['place_x']:.3f})")

# 작업마다 걸린 시간
starts = [(st, int(p.split(":")[0][1:])) for st, p in phase_log if p.startswith("j") and p.split(":")[0][1:].isdigit()]
job_steps = {}
for st, j in starts:
    job_steps.setdefault(j, [st, st]); job_steps[j][1] = st
dt = world.get_physics_dt()
job_sec = {j: round((e - s) * dt, 1) for j, (s, e) in job_steps.items()}
say(f"작업별 소요(초, 마지막 단계 시작까지): {job_sec}  전체 {round(step * dt, 1)}")
say(f"관절 각속도: 최대 한계의 {peak*100:.0f}%, 80% 초과 {over80}스텝")
for k, (n, r) in spikes.items(): say(f"  80% 초과 구간 {k}: {n}스텝 최대 {r*100:.0f}%")
n_ok = sum(r["success"] for r in results)
say(f"결과: {n_ok}/{len(results)} 성공")
json.dump({"results": results, "joint_over80_steps": over80, "joint_peak_ratio": round(peak, 3), "spikes": spikes,
           "job_sec": job_sec, "phases": phase_log, "log": LOG},
          open(f"{args.out}/result.json", "w"), indent=1, ensure_ascii=False, default=float)
app.close()
