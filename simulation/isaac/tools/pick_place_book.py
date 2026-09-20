"""팀 레벨에서 데크 위 책 1권을 집어 선반 1번 칸에 직선 삽입하고 복귀한다.

동작 정의는 arm_primitives.py 를 그대로 쓰고, 여기서는 Isaac Sim 백엔드만 붙인다.
저장된 레벨 파일은 수정하지 않는다 (선반 충돌체·책 질량은 메모리에서만 추가).

GPU PC:
    ~/isaacsim/python.sh ~/arm/isaac/pick_place_book.py --usd ~/Desktop/ing_library_env.usd --out /tmp/pp
    (--gui 를 주면 화면에 띄워 실시간으로 본다)
"""
import argparse
import json
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", default="/tmp/pp")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--book-mass", type=float, default=0.5)
ap.add_argument("--capture-every", type=int, default=15)
ap.add_argument("--max-steps", type=int, default=6000)
ap.add_argument("--shelf-scale-z", type=float, default=1.4, help="칸 높이를 늘려 위에서 내려놓을 공간 확보")
ap.add_argument("--robot-shift", type=float, default=0.15,
                help="로봇 배치 위치를 선반 쪽(+Y)으로 옮기는 거리(m). 레벨의 초기 배치 변경이지 AMR 주행이 아니다. "
                     "선반을 옮기면 나머지 선반·바닥·벽과 배치가 어긋나므로 로봇을 옮긴다")
ap.add_argument("--jitter", type=float, default=0.0, help="책 시작 위치를 x·y 로 ±이 값(m)만큼 무작위로 흔든다")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--start-delay", type=float, default=0.0, help="동작 시작 전 대기(초). 녹화 준비용")
ap.add_argument("--mode", choices=["place", "spine_push", "spine_out"], default="spine_out",
                help="place: 바닥 바로 위에서 놓기 / spine_push: 책등 위로 세워 끼우고 눌러 넣기 / "
                     "spine_out: 도서관식 — 책등이 로봇 쪽을 보게 세워 끼우고 책등을 밀어 넣기")
ap.add_argument("--plan-only", action="store_true", help="경로 IK 만 풀어 출력하고 끝낸다")
ap.add_argument("--inject", choices=["none", "no_grasp", "drop_in_carry"], default="none",
                help="(검증용) 실패를 일부러 만들어 오류 감지(M405·M406)가 동작하는지 시험한다")
ap.add_argument("--selftest-stop-at", type=int, default=0,
                help="(검증용) 1회차 실행 중 이 스텝 수에서 프로그램으로 정지한다")
ap.add_argument("--selftest-replays", type=int, default=0,
                help="(검증용) 헤드리스에서 재생·정지를 프로그램으로 N회 반복해 GUI 재실행 흐름을 시험한다")
ap.add_argument("--place-dx", type=float, default=None,
                help="넣는 위치 x = 팔 원점 x + 이 값. 기본: spine_out −0.35 (팔꿈치 한계 회피), 그 외 −0.10")
ap.add_argument("--home", choices=["deck", "config"], default="deck",
                help="deck: 데크(트레이)를 내려다보는 자세를 IK 로 계산해 홈으로 쓴다")
ap.add_argument("--prebuilt", action="store_true",
                help="build_level_v2.py 로 저장한 레벨. 로봇 이동·선반 스케일·충돌체·질량·칸막이를 다시 적용하지 않는다")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import cv2
import numpy as np
import yaml
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.sensors.camera import Camera

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers"))
from arm_primitives import ArmController, MoveLinear, MoveJoint, Primitive, SetGripper, Sequence, Status, Wait  # noqa: E402
from arm_planning import tucked_joint_moves  # noqa: E402

os.makedirs(f"{args.out}/frames", exist_ok=True)
LOG = []
def say(m):
    LOG.append(m); sys.stderr.write(f"### {m}\n"); sys.stderr.flush()

R = "/World/ridgeback_franka"
SHELF = "/World/bookshelves/shelf_brown__book_shelf_01"
BOOK = "/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15"
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]

# ------------------------------------------------------------------ 레벨
open_stage(args.usd); app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()

# 선반에 충돌체가 없다 → 정적 충돌체를 메모리에서 붙인다 (레벨 파일은 그대로)
n = 0
for p in ([] if args.prebuilt else Usd.PrimRange(stage.GetPrimAtPath(SHELF))):
    if p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")
        n += 1
if not args.prebuilt: say(f"선반 충돌체 추가: 메쉬 {n}개")

# 책 질량: 지정이 없어 밀도 1000 → 약 1.35kg 이 된다. 실제 책 수준으로 낮춘다
if not args.prebuilt:
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(BOOK)).CreateMassAttr().Set(args.book_mass)
if not args.prebuilt: say(f"책 질량 {args.book_mass} kg")

if args.prebuilt:
    say("prebuilt 레벨: 레벨 편집을 건너뛴다")
shelf_prim = SingleXFormPrim(SHELF)
sc = shelf_prim.get_local_scale() if not args.prebuilt else None
if not args.prebuilt:
    shelf_prim.set_local_scale(np.array([sc[0], sc[1], sc[2] * args.shelf_scale_z]))
if not args.prebuilt: say(f"선반 z 스케일 x{args.shelf_scale_z} (메모리에서만. 레벨 반영 시 선반 전체에 같은 배율)")

# 로봇 배치 위치를 옮긴다 (주행이 아니라 초기 배치). 데크 위 책도 같이 옮긴다
SHIFT = np.array([0.0, args.robot_shift, 0.0])
for path in (() if args.prebuilt else (R, BOOK)):
    xf = SingleXFormPrim(path)
    pp_, pq_ = xf.get_world_pose()
    xf.set_world_pose(np.array(pp_) + SHIFT, pq_)
if not args.prebuilt: say(f"로봇·데크 위 책 배치를 +Y {args.robot_shift} m 옮김 (메모리에서만)")

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
robot = SingleArticulation(prim_path=R, name="rf")

# ------------------------------------------------------------------ 책 받침(칸막이)
# 두께 3.5cm 면으로 선 책은 시작할 때 6cm 높이에서 떨어지며 몇 초 안에 스스로 넘어졌다
# (팔이 닿기 전, step 45 캡처로 확인). 반납기 트레이의 칸막이처럼 양옆에 낮은 벽을 세운다.
# 손가락은 칸막이보다 위에서 책 윗부분을 잡으므로 부딪히지 않는다
_cache0 = create_bbox_cache()
bb0 = np.array(compute_aabb(_cache0, BOOK, include_children=True), float)
DECK_Z = 0.286
book_xf = SingleXFormPrim(BOOK)
bpos, bquat = book_xf.get_world_pose()
drop = 0.0 if args.prebuilt else (DECK_Z + 0.001) - bb0[2]
_rng = np.random.default_rng(args.seed)
jx, jy = (_rng.uniform(-args.jitter, args.jitter, 2) if args.jitter > 0 else (0.0, 0.0))
book_xf.set_world_pose(np.array(bpos) + np.array([jx, jy, drop]), bquat)
bb0 = bb0 + np.array([jx, jy, drop, jx, jy, drop])
if not args.prebuilt or args.jitter > 0: say(f"책 배치 보정 z {drop:+.3f} m, 흔들기 x {jx:+.3f} y {jy:+.3f}")

DIV_H, DIV_T, GAP = 0.07, 0.01, 0.003
def dividers(prefix, x_lo, x_hi, y_lo, y_hi, z_floor):
    cy = (y_lo + y_hi) / 2; ly = (y_hi - y_lo) + 0.02
    for side, cx in (("L", x_lo - GAP - DIV_T / 2), ("R", x_hi + GAP + DIV_T / 2)):
        world_scene_objects.append(FixedCuboid(
            prim_path=f"/World/pp_{prefix}_{side}", name=f"pp_{prefix}_{side}",
            position=np.array([cx, cy, z_floor + DIV_H / 2]),
            scale=np.array([DIV_T, ly, DIV_H]), size=1.0, color=np.array([0.2, 0.2, 0.25])))
world_scene_objects = []
if not args.prebuilt:
    dividers("tray", bb0[0], bb0[3], bb0[1], bb0[4], DECK_Z)
for obj in world_scene_objects:
    world.scene.add(obj)
if not args.prebuilt: say("데크 칸막이 2개 추가")

cams = {}
views = {} if args.gui else {"side": ((1.05, -4.85, 2.05), (2.70, -2.90, 0.60)),
         "front": ((4.55, -4.25, 1.75), (2.70, -2.80, 0.60))}
for name, (eye, tgt) in views.items():
    path = f"/World/pp_cam_{name}"
    cams[name] = Camera(prim_path=path, resolution=(960, 540))
    set_camera_view(eye=np.array(eye), target=np.array(tgt), camera_prim_path=path)

RUN_STATE = {"count": 0}

class PlanAbort(Exception):
    pass

class RunStopped(Exception):
    pass

def run_once():
    _steps = [0]
    def sim_step(render=False):
        """시뮬레이션 한 스텝. GUI 에서 정지가 눌리면 그 스텝 안에서 처리되므로 **스텝 직후에** 확인한다.
        스텝 전에만 확인했더니 정지 직후 관절값이 None 이 되어 프로그램이 종료됐다"""
        world.step(render=render)
        _steps[0] += 1
        if args.selftest_stop_at and RUN_STATE["count"] == 1 and _steps[0] == args.selftest_stop_at:
            world.stop()                                   # 자체 시험: 실행 도중 정지
        if (args.gui or args.selftest_replays > 0) and not world.is_playing():
            if stage.GetPrimAtPath("/World/pp_grasp_joint").IsValid():
                stage.RemovePrim("/World/pp_grasp_joint")
            raise RunStopped()

    def abort_with(code, msg):
        """계획 단계 실패. 로봇을 움직이지 않고 결과만 남긴다 (M401 IK 실패 / M402 경로 불연속)"""
        say(f"M{code} {msg}")
        json.dump({"success": False, "error_code": code, "error": msg, "log": LOG},
                  open(f"{args.out}/result.json", "w"), indent=1, ensure_ascii=False, default=float)
        raise PlanAbort(f"M{code}")
    world.reset()
    robot.initialize()
    for c in cams.values():
        c.initialize()

    idx_arm = [robot.get_dof_index(j) for j in ARM_JOINTS]
    idx_fing = [robot.get_dof_index(j) for j in FINGERS]
    base_idx = [robot.get_dof_index(j) for j in robot.dof_names if j.startswith("dummy_base")]
    base_hold = robot.get_joint_positions()[base_idx]

    for _ in range(90):                       # 책이 데크에 안착할 때까지
        sim_step(render=args.gui)
        if args.gui and not world.is_playing():
            raise RunStopped()

    cache = create_bbox_cache()
    def aabb(path):
        cache.Clear()
        return np.array(compute_aabb(cache, path, include_children=True), float)

    l0p, l0q = SingleXFormPrim(R + "/panda_link0").get_world_pose()
    cfg = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
    lula = LulaKinematicsSolver(**cfg)
    lula.set_robot_base_pose(l0p, l0q)       # 베이스가 월드 원점이 아니다. 안 넣으면 목표가 전부 어긋난다
    ik = ArticulationKinematicsSolver(robot, lula, "right_gripper")

    # ------------------------------------------------------------------ 그리퍼 좌표축 실측
    q = robot.get_joint_positions()
    ee_p, ee_R = ik.compute_end_effector_pose()
    hand_p = np.array(SingleXFormPrim(R + "/panda_hand").get_world_pose()[0])
    lf = np.array(SingleXFormPrim(R + "/panda_leftfinger").get_world_pose()[0])
    rf = np.array(SingleXFormPrim(R + "/panda_rightfinger").get_world_pose()[0])
    approach_w = (ee_p - hand_p) / np.linalg.norm(ee_p - hand_p)
    closing_w = (rf - lf) / (np.linalg.norm(rf - lf) or 1.0)
    a_loc = ee_R.T @ approach_w
    c_loc = ee_R.T @ closing_w
    say(f"right_gripper 로컬 접근축 {np.round(a_loc,2).tolist()} / 닫힘축 {np.round(c_loc,2).tolist()}")

    def quat_from_R(M):
        w = math.sqrt(max(0.0, 1 + M[0, 0] + M[1, 1] + M[2, 2])) / 2
        x = math.copysign(math.sqrt(max(0.0, 1 + M[0, 0] - M[1, 1] - M[2, 2])) / 2, M[2, 1] - M[1, 2])
        y = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] + M[1, 1] - M[2, 2])) / 2, M[0, 2] - M[2, 0])
        z = math.copysign(math.sqrt(max(0.0, 1 - M[0, 0] - M[1, 1] + M[2, 2])) / 2, M[1, 0] - M[0, 1])
        return np.array([w, x, y, z])

    def orientation(approach, closing):
        """그리퍼의 (실측) 로컬 접근축·닫힘축이 월드의 approach·closing 을 향하는 회전"""
        a_l = np.round(a_loc); c_l = np.round(c_loc)
        b_l = np.cross(a_l, c_l)
        a_w = np.array(approach, float); c_w = np.array(closing, float); b_w = np.cross(a_w, c_w)
        L = np.stack([a_l, c_l, b_l], axis=1)
        W = np.stack([a_w, c_w, b_w], axis=1)
        return quat_from_R(W @ L.T)

    HORIZ = orientation([0, 1, 0], [1, 0, 0])   # 손끝 +Y(선반 쪽), 손가락은 X 방향으로 닫힘

    # ------------------------------------------------------------------ 목표 계산 (원점이 아니라 바운딩박스 기준)
    bb = aabb(BOOK); shelf = aabb(SHELF)
    bc = (bb[:3] + bb[3:]) / 2
    half = (bb[3:] - bb[:3]) / 2
    say(f"책 AABB {np.round(bb,3).tolist()}  중심 {np.round(bc,3).tolist()}  반치수 {np.round(half,3).tolist()}")
    say(f"선반 AABB {np.round(shelf,3).tolist()}")

    DOWN = orientation([0, 0, -1], [1, 0, 0])   # 손끝 아래, 손가락은 X(책 두께) 방향으로 닫힘

    y_front = shelf[1]
    floor_z = 0.355 * args.shelf_scale_z          # 1번 칸 바닥 (원래 0.355, 스케일 반영)
    ceil_z = 0.705 * args.shelf_scale_z
    book_h = bb[5] - bb[2]
    TIP_DOWN = 0.035                               # 책 윗면에서 손끝이 내려가 잡는 깊이
    place_x = l0p[0] + (args.place_dx if args.place_dx is not None else (-0.35 if args.mode == "spine_out" else -0.10))
    book_place_cy = y_front + 0.03 + half[1]
    tip_place = np.array([place_x, book_place_cy, floor_z + book_h - TIP_DOWN + 0.012])
    tip_place_hover = tip_place + np.array([0, 0, 0.03])
    tip_place_out = np.array([place_x, y_front - 0.20, tip_place_hover[2]])
    tip_grasp = np.array([bc[0], bc[1], bb[5] - TIP_DOWN])
    tip_pre_grasp = tip_grasp + np.array([0, 0, 0.13])
    tip_lift = tip_grasp + np.array([0, 0, 0.17])
    say(f"칸 바닥 {floor_z:.3f} 천장 {ceil_z:.3f}  배치 손끝 {np.round(tip_place,3).tolist()}")

    # 선반 칸에도 같은 칸막이를 세워 놓은 책이 서 있게 한다 (북엔드)
    _shelf_divs = []
    for side, cx in (() if args.prebuilt else (("L", place_x - half[0] - GAP - DIV_T / 2), ("R", place_x + half[0] + GAP + DIV_T / 2))):
        d = FixedCuboid(prim_path=f"/World/pp_shelf_{side}", name=f"pp_shelf_{side}",
                        position=np.array([cx, book_place_cy, floor_z + DIV_H / 2]),
                        scale=np.array([DIV_T, 2 * half[1] + 0.02, DIV_H]), size=1.0, color=np.array([0.2, 0.2, 0.25]))
        world.scene.add(d); _shelf_divs.append(d)
    if not args.prebuilt: say("선반 북엔드 2개 추가")
    for _ in range(10):
        sim_step(render=False)

    joint_names = lula.get_joint_names()
    def ik_joints(target, ori, seed):
        q, ok = lula.compute_inverse_kinematics("right_gripper", np.asarray(target, float), np.asarray(ori, float),
                                                np.asarray(seed, float), 0.004, 0.05)
        return (np.asarray(q, float), bool(ok))

    cur = robot.get_joint_positions()[idx_arm]
    q_pre_grasp, ok1 = ik_joints(tip_pre_grasp, DOWN, cur)
    q_place_out, ok2 = ik_joints(tip_place_out, DOWN, q_pre_grasp if ok1 else cur)
    for name, t in {"pre_grasp": tip_pre_grasp, "grasp": tip_grasp, "lift": tip_lift,
                    "place_out": tip_place_out, "place_hover": tip_place_hover, "place": tip_place}.items():
        _, ok = ik_joints(t, DOWN, q_pre_grasp if ok1 else cur)
        say(f"IK 사전확인 {name:12s} {np.round(t,3).tolist()} -> {ok}")
    if not (ok1 and ok2):
        say("관절 경유점 IK 실패 — 중단"); raise PlanAbort("계획 실패")

    # ------------------------------------------------------------------ 백엔드
    class IsaacBackend:
        def __init__(self):
            self._grip = 0.035
        @property
        def dt(self): return 1.0 / 60.0
        def get_joint_positions(self): return robot.get_joint_positions()[idx_arm]
        def set_joint_targets(self, positions):
            q = robot.get_joint_positions().copy()
            q[idx_arm] = np.asarray(positions, float)[:7]; self.last_cmd = q[idx_arm].copy()
            q[idx_fing] = self._grip
            q[base_idx] = base_hold
            robot.apply_action(ArticulationAction(joint_positions=q))
        def get_ee_pose(self):
            p, Rm = ik.compute_end_effector_pose()
            return np.asarray(p, float), quat_from_R(Rm)
        def compute_ik(self, position, orientation, seed=None):
            q, ok = ik_joints(position, orientation, self.get_joint_positions())
            return (q, True) if ok else (np.zeros(7), False)
        def set_gripper_width(self, w):
            self._grip = float(w)
            q = robot.get_joint_positions().copy()
            q[idx_fing] = self._grip; q[base_idx] = base_hold
            robot.apply_action(ArticulationAction(joint_positions=q))
        def get_gripper_width(self):
            return float(np.mean(robot.get_joint_positions()[idx_fing]))

    conf = yaml.safe_load(open(os.path.expanduser("~/arm/arm_config.yaml")))
    conf["gripper"]["grasp_m"] = max(0.0, half[0] - 0.004)
    conf["gripper"]["tolerance_m"] = 0.006
    conf["tolerance"]["position_m"] = 0.008
    conf["tolerance"]["joint_rad"] = 0.03
    arm = ArmController(IsaacBackend(), conf)

    def R_from_quat(q):
        w, x, y, z = q
        return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                         [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                         [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

    def slerp(q0, q1, t):
        q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
        d = float(np.dot(q0, q1))
        if d < 0: q1, d = -q1, -d
        if d > 0.9995:
            q = q0 + t * (q1 - q0); return q / np.linalg.norm(q)
        th = math.acos(d)
        return (math.sin((1-t)*th) * q0 + math.sin(t*th) * q1) / math.sin(th)

    class Call(Primitive):
        """함수 하나를 한 번 실행하고 끝나는 프리미티브 (고정조인트 붙이기·떼기용)"""
        def __init__(self, name, fn):
            super().__init__(timeout_s=1.0); self.name = name; self._fn = fn
        def on_update(self, ctx):
            self._fn(); return Status.SUCCEEDED

    def quat_angle(q0, q1):
        d = abs(float(np.dot(np.asarray(q0, float), np.asarray(q1, float))))
        return 2.0 * math.acos(min(1.0, d))

    def plan_path(waypoints, seed, step_m=0.005, step_rad=0.035, label=""):
        """직교 경유점들을 촘촘히 나눠 IK 를 앞 해에서 이어 풀고, 관절 경로로 돌려준다.

        실행 중 매 스텝 IK 를 새로 풀면 해가 가지를 바꿔 튀었다(손목 세우기 구간 38회).
        미리 풀어 두면 실행은 관절 보간만 하므로 튈 수 없고, 계획 단계에서 불연속을 잡을 수 있다."""
        qs = [np.asarray(seed, float)]
        prev_p, prev_q = None, None
        worst = 0.0
        for p_, q_ in waypoints:
            p_ = np.asarray(p_, float); q_ = np.asarray(q_, float)
            if prev_p is None:
                prev_p, prev_q = p_, q_
                sol, ok = ik_joints(p_, q_, qs[-1])
                if not ok:
                    return None, f"{label}: 시작점 IK 실패", worst
                worst = max(worst, float(np.max(np.abs(sol - qs[-1])))); qs.append(sol); continue
            n = max(1, int(math.ceil(np.linalg.norm(p_ - prev_p) / step_m)),
                    int(math.ceil(quat_angle(prev_q, q_) / step_rad)))
            for i in range(1, n + 1):
                t = i / n
                sol, ok = ik_joints(prev_p + (p_ - prev_p) * t, slerp(prev_q, q_, t), qs[-1])
                if not ok:
                    return None, f"{label}: IK 실패 at {np.round(prev_p + (p_ - prev_p) * t, 3).tolist()}", worst
                d = float(np.max(np.abs(sol - qs[-1])))
                worst = max(worst, d)
                qs.append(sol)
            prev_p, prev_q = p_, q_
        return qs, "", worst

    class JointPath(Primitive):
        """미리 계획한 관절 경유점을 일정한 관절 속도로 따라간다. 실행 중 IK 를 풀지 않는다."""
        name = "joint_path"
        def __init__(self, name, qs, speed=0.6, timeout_s=None):
            length = sum(float(np.max(np.abs(b - a))) for a, b in zip(qs[:-1], qs[1:]))
            super().__init__(timeout_s if timeout_s else max(4.0, length / speed * 2.0 + 3.0))
            self.name = name; self.qs = [np.asarray(q, float) for q in qs]; self.speed = speed
        def on_start(self, ctx):
            self._pts = [ctx.backend.get_joint_positions()] + self.qs
            self._i = 0; self._target = self._pts[0].copy()
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
            if self._i >= len(self._pts) - 1:
                err = float(np.max(np.abs(ctx.backend.get_joint_positions() - self._pts[-1])))
                if err <= 0.02:
                    return Status.SUCCEEDED
            return Status.RUNNING

    GRASP_JOINT = "/World/pp_grasp_joint"
    def attach_book():
        """파지 순간 손과 책을 고정 조인트로 붙인다.
        프로젝트 지침: 마찰 기반 파지 튜닝은 범위 밖 → Surface Gripper 또는 파지 시 fixed joint.
        손목을 세우면 책이 옆으로 튀어나와 마찰만으로는 손 안에서 돌았다(캡처로 확인)."""
        hp, hq = SingleXFormPrim(R + "/panda_hand").get_world_pose()
        bp, bq = SingleXFormPrim(BOOK).get_world_pose()
        Rh = R_from_quat(hq)
        rel_p = Rh.T @ (np.asarray(bp) - np.asarray(hp))
        rel_q = quat_from_R(Rh.T @ R_from_quat(bq))
        j = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT)
        j.CreateBody0Rel().SetTargets([R + "/panda_hand"])
        j.CreateBody1Rel().SetTargets([BOOK])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_p]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rel_q[0]), Gf.Vec3f(*[float(v) for v in rel_q[1:]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        j.CreateExcludeFromArticulationAttr().Set(True)
        say("책 고정 조인트 연결")

    def detach_book():
        if stage.GetPrimAtPath(GRASP_JOINT).IsValid():
            stage.RemovePrim(GRASP_JOINT); say("책 고정 조인트 해제")

    HORIZ = orientation([0, 1, 0], [1, 0, 0])   # 손끝 +Y(선반 안쪽), 손가락은 X 방향 — 책등 뒤에서 민다

    # ------------------------------------------------------------------ 데크를 보는 홈 자세
    q_home = np.asarray(conf["poses"]["home"], float)
    if args.home == "deck":
        seed = robot.get_joint_positions()[idx_arm].copy()
        seed[0] = math.atan2(bc[1] - l0p[1], bc[0] - l0p[0])        # 1축을 책 쪽으로 돌린 시드
        q_try, ok_h = ik_joints(np.array([bc[0], bc[1], bb[5] + 0.30]), DOWN, seed)
        if ok_h:
            q_home = q_try
            say(f"데크 홈 자세(책 위 30cm): {np.round(q_home, 3).tolist()}")
        else:
            say("데크 홈 IK 실패 — config 홈 사용")
    # 시작 자세에서 홈까지 몸통을 크게 돌린다 → 팔을 접은 채 돌린다 (arm_planning.tucked_joint_moves 설명)
    for q_via in tucked_joint_moves(robot.get_joint_positions()[idx_arm], q_home, conf["poses"]["stow"]):
        arm.enqueue(MoveJoint(q_via, speed_scale=0.6, timeout_s=15))
    q_pre_grasp, ok1 = ik_joints(tip_pre_grasp, DOWN, q_home)

    if args.mode == "spine_out":
        down_qs, err1, w1 = plan_path([(tip_pre_grasp, DOWN), (tip_grasp, DOWN)], q_pre_grasp, label="내려가기")
        up_qs, err2, w2 = plan_path([(tip_grasp, DOWN), (tip_lift, DOWN)], down_qs[-1] if down_qs else q_pre_grasp, label="들기")
        say(f"계획 내려가기 {len(down_qs or [])}점 최대 {w1:.3f} rad {err1} / 들기 {len(up_qs or [])}점 최대 {w2:.3f} rad {err2}")
        if down_qs is None or up_qs is None:
            abort_with(401, f"파지 경로 IK 실패: {err1} {err2}")
        if max(w1, w2) > 0.12:
            abort_with(402, f"파지 경로 불연속: {max(w1, w2):.3f} rad")
        q_after_lift = up_qs[-1]
        arm.enqueue(Sequence("pick", [
            SetGripper(0.04),
            MoveJoint(q_pre_grasp, timeout_s=12),
            JointPath("down", down_qs[1:], speed=0.25),
            SetGripper(0.04 if args.inject == "no_grasp" else conf["gripper"]["grasp_m"], settle_s=0.6),
            *([] if args.inject == "no_grasp" else [Call("attach", attach_book)]),
            JointPath("lift", up_qs[1:], speed=0.25),
        ]))
    else:
        arm.enqueue(Sequence("pick", [
            SetGripper(0.04),
            MoveJoint(q_pre_grasp, timeout_s=12),                         # 큰 이동은 관절 보간
            MoveLinear((tip_grasp, DOWN), speed_scale=0.3),
            SetGripper(conf["gripper"]["grasp_m"], settle_s=0.6),
            MoveLinear((tip_lift, DOWN), speed_scale=0.3),
        ]))
    if args.mode == "spine_out":
        # 위에서 책등을 잡은 채 손목을 X축으로 90° 세운다: 책 긴 변 세로, 책등은 로봇 쪽(-Y), 앞마구리는 선반 안쪽
        L = 2 * half[1]                     # 책 길이 → 세로
        W = book_h                          # 책 폭(책등→앞마구리) → 선반 깊이
        grip_z = floor_z + L / 2 + 0.004    # 책 아래가 칸 바닥 4mm 위
        spine_final = y_front + 0.02        # 최종 책등: 선반 앞면 2cm 안쪽
        y_pre = y_front - (W - TIP_DOWN) - 0.03                                          # 앞마구리가 선반 앞 3cm 밖
        tip_transfer = np.array([place_x, y_pre - 0.02, grip_z + 0.06])                 # 책을 아래로 든 채 선반 앞
        tip_pre = np.array([place_x, y_pre, grip_z + 0.01])
        tip_wedge = np.array([place_x, y_front + 0.10 - (W - TIP_DOWN), grip_z])         # 앞마구리 10cm 끼움
        # 놓은 뒤 손끝이 책등 뒤로 완전히 빠져야 그리퍼를 닫을 수 있다. 손가락이 표지를 감싼 채 닫히면
        # 책을 집어 후퇴할 때 딸려 나온다(6회 중 2회 실패의 원인). 손끝이 책등 뒤 2.5cm 까지 빠진다
        tip_back = tip_wedge - np.array([0, TIP_DOWN + 0.025, 0])
        spine_wedge = tip_wedge[1] - TIP_DOWN
        push_z = floor_z + L / 2
        tip_touch = np.array([place_x, spine_wedge - 0.008, push_z])
        # 닫은 손끝의 실제 접촉점은 right_gripper 기준점보다 약 1cm 앞이다 (책등이 1.2cm 더 들어간 것으로 확인)
        tip_push = np.array([place_x, spine_final + 0.002 - 0.010, push_z])
        tip_retreat = np.array([place_x, y_front - 0.13, push_z])

        # 경로 전체를 앞 자세를 초기값으로 이어서 풀어 본다. 초기값을 바꿔 가며 풀면
        # 현재 자세와 동떨어진 해가 나와 관절 보간 중 팔이 뒤집혔다(캡처로 확인)
        N_ROT = 6
        rot_steps = [(tip_transfer + (tip_pre - tip_transfer) * (k / N_ROT), slerp(DOWN, HORIZ, k / N_ROT)) for k in range(1, N_ROT + 1)]
        chain = [("lift", tip_lift, DOWN), ("transfer", tip_transfer, DOWN)] + \
                [(f"rot{k}", t, q) for k, (t, q) in enumerate(rot_steps, 1)] + \
                [("wedge", tip_wedge, HORIZ), ("back", tip_back, HORIZ), ("touch", tip_touch, HORIZ),
                 ("push", tip_push, HORIZ), ("retreat", tip_retreat, HORIZ)]
        seed = q_pre_grasp if ok1 else q_home
        chain_ok = True
        for name, t, q in chain:
            qs, ok = ik_joints(t, q, seed)
            jump = float(np.max(np.abs(qs - seed))) if ok else float("nan")
            worst = int(np.argmax(np.abs(qs - seed))) + 1 if ok else -1
            say(f"경로 IK {name:8s} {np.round(t,3).tolist()} -> {ok}  최대변화 {jump:.2f} rad (joint{worst})  q={np.round(qs,2).tolist()}")
            chain_ok &= ok
            if ok: seed = qs
        if not chain_ok:
            say("세워 꽂기 경로 IK 실패 — 중단"); raise PlanAbort("계획 실패")
        if False:
            jumps = []; seed2 = q_pre_grasp if ok1 else q_home; minj4 = 0.0; maxj6 = 0.0
            for name, t, q in chain:
                qs, ok = ik_joints(t, q, seed2)
                if ok:
                    jumps.append(float(np.max(np.abs(qs - seed2)))); minj4 = min(minj4, qs[3]); maxj6 = max(maxj6, qs[5]); seed2 = qs
            say(f"PLAN place_x={place_x:.3f} 전체OK={chain_ok} 최대점프={max(jumps):.2f}rad joint4최소={minj4:.2f}(한계-3.07) joint6최대={maxj6:.2f}(한계3.75)")
            raise PlanAbort("plan-only")

        # 경로 계획: 운반 → 손목 세우기(연속 회전) → 끼우기 / 뒤로 → 닿기 → 밀기 → 후퇴
        carry_qs, e1a, w1a = plan_path([(tip_lift, DOWN), (tip_transfer, DOWN), (tip_pre, HORIZ)], q_after_lift, label="운반·손목세우기")
        wedge_qs, e1b, w1b = plan_path([(tip_pre, HORIZ), (tip_wedge, HORIZ)], carry_qs[-1] if carry_qs else q_after_lift, label="끼우기")
        ins_qs = (carry_qs + wedge_qs[1:]) if (carry_qs and wedge_qs) else None
        e1, w1 = (e1a or e1b), max(w1a, w1b)
        back_qs, e2a, w2a = plan_path([(tip_wedge, HORIZ), (tip_back, HORIZ)], ins_qs[-1] if ins_qs else q_after_lift, label="뒤로")
        touch_qs, e2b, w2b = plan_path([(tip_back, HORIZ), (tip_touch, HORIZ)], back_qs[-1] if back_qs else q_after_lift, label="닿기")
        e2, w2 = (e2a or e2b), max(w2a, w2b)
        push_qs, e3, w3 = plan_path([(tip_touch, HORIZ), (tip_push, HORIZ)], touch_qs[-1] if touch_qs else q_after_lift, label="밀기")
        out_qs, e4, w4 = plan_path([(tip_push, HORIZ), (tip_retreat, HORIZ)], push_qs[-1] if push_qs else q_after_lift, label="후퇴")
        for nm, qs_, e_, w_ in (("운반·손목세우기·끼우기", ins_qs, e1, w1), ("뒤로·닿기", back_qs, e2, w2),
                                ("밀기", push_qs, e3, w3), ("후퇴", out_qs, e4, w4)):
            say(f"계획 {nm:14s} {len(qs_ or [])}점  인접점 최대 관절변화 {w_:.3f} rad  {e_}")
        MAX_STEP = 0.12      # 5mm/2° 간격에서 이보다 크게 변하면 해가 가지를 바꾼 것(특이점 근처)
        if None in (ins_qs, back_qs, touch_qs, push_qs, out_qs):
            abort_with(401, "삽입 경로 IK 실패: " + " ".join(e for e in (e1, e2, e3, e4) if e))
        if max(w1, w2, w3, w4) > MAX_STEP:
            abort_with(402, f"삽입 경로 불연속: 인접점 최대 관절변화 {max(w1, w2, w3, w4):.3f} rad > {MAX_STEP}")
        if args.plan_only:
            say("plan-only 종료"); raise PlanAbort("plan-only")

        arm.enqueue(Sequence("insert_spine_out", [
            JointPath("carry_rotate", carry_qs[1:], speed=0.35),
            JointPath("wedge", wedge_qs[1:], speed=0.35),
            Call("detach", detach_book),
            SetGripper(0.04, settle_s=0.4),
            Wait(0.4),
        ]))
        arm.enqueue(Sequence("push_spine_out", [
            JointPath("back", back_qs[1:], speed=0.3),                                   # 손끝이 책등 뒤로 완전히 빠진다
            SetGripper(0.0, settle_s=0.4),                                               # 그 다음에 닫아 미는 면을 만든다
            JointPath("touch", touch_qs[1:], speed=0.3),
            JointPath("push", push_qs[1:], speed=0.12),                                  # 책등을 천천히 밀어 넣기
            Wait(0.3),
            JointPath("retreat", out_qs[1:], speed=0.35),
        ]))
    elif args.mode == "place":
        arm.enqueue(Sequence("place", [
            MoveJoint(q_place_out, speed_scale=0.6, timeout_s=15),
            MoveLinear((tip_place_hover, DOWN), speed_scale=0.25, timeout_s=15),   # 칸 안으로 수평 진입
            MoveLinear((tip_place, DOWN), speed_scale=0.2),
            SetGripper(0.04, settle_s=0.4),
            Wait(0.5),
            MoveLinear((tip_place_hover, DOWN), speed_scale=0.3),
            MoveLinear((tip_place_out, DOWN), speed_scale=0.4, timeout_s=15),      # 수평 후퇴
        ]))
    else:
        # 책등이 천장을 본다(메쉬 꼭짓점 78% 가 윗면). 위에서 잡은 곳이 곧 책등이다
        spine_top = floor_z + book_h                                # 다 들어갔을 때 책등 높이
        # 칸막이는 책 좌우(X)에 진입 방향(Y)과 나란히 서 있어 책이 그 사이로 들어간다. 넘어갈 필요가 없다.
        # 칸막이 위로 넘기려 높게(0.705) 들어갔더니 손목이 윗칸 바닥판에 걸렸다
        tip_hover = np.array([place_x, book_place_cy, floor_z + 0.045 + book_h - TIP_DOWN])  # 책 바닥이 칸 바닥 위 4.5cm
        tip_hover_out = np.array([place_x, y_front - 0.20, tip_hover[2]])
        tip_wedge = np.array([place_x, book_place_cy, floor_z + 0.025 + book_h - TIP_DOWN])  # 틈에 살짝 끼움 (바닥 위 2.5cm)
        tip_clear = tip_wedge + np.array([0, 0, 0.03])
        tip_touch = np.array([place_x, book_place_cy, spine_top + 0.008])                    # 닫은 손끝이 책등 8mm 위
        tip_press = np.array([place_x, book_place_cy, spine_top - 0.002])                    # 책등을 눌러 끝까지
        q_hover_out, okh = ik_joints(tip_hover_out, DOWN, q_place_out)
        for name, t in {"hover_out": tip_hover_out, "hover": tip_hover, "wedge": tip_wedge,
                        "touch": tip_touch, "press": tip_press}.items():
            _, ok = ik_joints(t, DOWN, q_hover_out if okh else q_place_out)
            say(f"IK 사전확인 {name:10s} {np.round(t,3).tolist()} -> {ok}")
        say(f"천장까지 여유: 손 윗부분 약 {tip_hover[2] + 0.23:.3f} / 천장 {ceil_z:.3f}")
        arm.enqueue(Sequence("insert", [
            MoveJoint(q_hover_out if okh else q_place_out, speed_scale=0.6, timeout_s=15),
            MoveLinear((tip_hover, DOWN), speed_scale=0.25, timeout_s=15),        # 칸막이 위로 수평 진입
            MoveLinear((tip_wedge, DOWN), speed_scale=0.15),                      # 틈에 살짝 끼우기
            SetGripper(0.04, settle_s=0.4),                                       # 놓기
            Wait(0.4),
            MoveLinear((tip_clear, DOWN), speed_scale=0.3),
        ]))
        arm.enqueue(Sequence("push_spine", [
            SetGripper(0.0, settle_s=0.3),                                        # 그리퍼를 닫아 누르는 면을 만든다
            MoveLinear((tip_touch, DOWN), speed_scale=0.3),
            MoveLinear((tip_press, DOWN), speed_scale=0.08, timeout_s=6),         # 책등을 천천히 눌러 밀어넣기
            Wait(0.3),
            MoveLinear((tip_hover, DOWN), speed_scale=0.3),
            MoveLinear((tip_hover_out, DOWN), speed_scale=0.4, timeout_s=15),     # 수평 후퇴
        ]))
    arm.enqueue(MoveJoint(q_home, speed_scale=0.6, timeout_s=15))

    # ------------------------------------------------------------------ 실행
    if args.start_delay > 0:
        say(f"{args.start_delay:.0f}초 뒤 시작")
        for _ in range(int(args.start_delay * 60)):
            sim_step(render=args.gui)

    phase_log = []; last_phase = None; frame = 0; status = Status.RUNNING
    # 관절 각속도 감시: 관절별 URDF 속도 한계(GPU PC lula_franka_gen.urdf 실측)의 80% 초과를 튐으로 본다.
    # 이전 기준(0.03 rad/step)은 시뮬 주기에 묶인 값이라 물리량으로 바꿨다 (웹 클로드 v6 회신 A4)
    VEL_LIMIT = np.array([2.175] * 4 + [2.61] * 3)
    VEL_RATIO_MAX = 0.8
    q_prev = robot.get_joint_positions()[idx_arm].copy()
    motion = {}                      # phase → [최대 한계비, 초과 스텝 수, 스텝 수]
    detected = []                    # (코드, 메시지) — 실행 중 감지한 실패
    watch = {}
    def book_origin():
        return np.asarray(SingleXFormPrim(BOOK).get_world_pose()[0], float)
    def tcp():
        return np.asarray(ik.compute_end_effector_pose()[0], float)
    def book_in_hand():
        """손(panda_hand) 좌표계에서 본 책 원점. 거리만 보면 손끝을 중심으로 책이 도는 미끄러짐을 놓친다(시험으로 확인)"""
        hp, hq = SingleXFormPrim(R + "/panda_hand").get_world_pose()
        return R_from_quat(hq).T @ (book_origin() - np.asarray(hp, float))
    step = 0
    for step in range(args.max_steps):
        if args.gui and not world.is_playing():
            detach_book(); raise RunStopped()
        status = arm.update()
        render = args.gui or step % args.capture_every == 0
        sim_step(render=render)
        q_now = robot.get_joint_positions()[idx_arm]
        ratio = np.abs(q_now - q_prev) / world.get_physics_dt() / VEL_LIMIT; q_prev = q_now.copy()
        ph = arm.phase if arm.phase != "idle" else (last_phase or "idle")
        m = motion.setdefault(ph, [0.0, 0, 0]); m[0] = max(m[0], float(ratio.max())); m[1] += int(ratio.max() > VEL_RATIO_MAX); m[2] += 1
        if ratio.max() > VEL_RATIO_MAX and m[1] <= 8:
            # 튐 원인 기록: 명령 대비 실제, 손목 링크와 고정 충돌체 AABB 겹침 (4권 시험에서 서가 윗판 충돌로 확인한 방식)
            if "coll" not in watch:
                st = get_current_stage()
                watch["coll"] = [(str(p_.GetPath()), aabb(str(p_.GetPath()))) for p_ in st.Traverse()
                                 if p_.HasAPI(UsdPhysics.CollisionAPI) and not str(p_.GetPath()).startswith(R) and str(p_.GetPath()) != BOOK]
            jm = int(ratio.argmax()); cmd = getattr(arm.ctx.backend, "last_cmd", q_now)
            hits = set()
            for ln in ("panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand", "panda_leftfinger", "panda_rightfinger"):
                lb = aabb(f"{R}/{ln}")
                hits.update(f"{ln}~{n_.split('/')[-1]}" for n_, ob in watch["coll"] if np.all(lb[:3] < ob[3:]) and np.all(ob[:3] < lb[3:]))
            say(f"  초과 step {step} {arm.phase} j{jm+1} {ratio[jm]*100:.0f}% 실제 {q_now[jm]:.3f} 명령 {cmd[jm]:.3f} 겹침 {sorted(hits)[:5]}")

        if args.inject == "drop_in_carry" and arm.phase.endswith("carry_rotate") and not watch.get("injected"):
            watch["injected"] = step
        if watch.get("injected") and step - watch["injected"] == 90:
            detach_book(); say("(시험) 운반 중 고정조인트 강제 해제")
        # 운반 중 이탈·미끄러짐 (M406): 손 좌표계에서 본 책 위치가 파지 시점보다 3cm 이상 변하면
        if "rel0" in watch and arm.phase.split(":")[-1] in ("carry_rotate", "wedge"):
            dev = float(np.linalg.norm(book_in_hand() - watch["rel0"]))
            if dev > 0.03:
                detected.append((406, f"운반 중 손 안에서 책이 {dev*100:.1f}cm 어긋남")); arm.cancel(); break

        if arm.phase != last_phase:
            _bp = book_origin(); _tp = tcp()
            phase_log.append((step, arm.phase, np.round(_bp, 3).tolist(), np.round(_tp, 3).tolist()))
            say(f"step {step:5d} 단계 → {arm.phase}   책원점 {np.round(_bp,3).tolist()} 손끝 {np.round(_tp,3).tolist()}")
            name = arm.phase.split(":")[-1]
            prev = (last_phase or "").split(":")[-1]
            if name == "lift":
                watch["book_z0"] = _bp[2]; watch["rel0"] = book_in_hand()
            if prev == "lift" and "book_z0" in watch:
                rise = _bp[2] - watch["book_z0"]
                if rise < 0.08:                                    # 손끝은 17cm 올린다
                    detected.append((405, f"들어 올린 뒤 책 상승 {rise*100:.1f}cm")); arm.cancel(); break
            if prev == "push" and args.mode == "spine_out":
                spine_now = aabb(BOOK)[1]
                if abs(spine_now - spine_final) > 0.015:
                    detected.append((407, f"밀기 후 책등 y {spine_now:.3f} (목표 {spine_final:.3f})"))
            if name == "retreat":
                watch["retreat_y0"] = _bp[1]
            if prev == "retreat" and "retreat_y0" in watch:
                back = watch["retreat_y0"] - _bp[1]
                if back > 0.01:
                    detected.append((408, f"후퇴 중 책이 {back*100:.1f}cm 딸려 나옴"))
            last_phase = arm.phase
        if not args.gui and step % args.capture_every == 0:
            for name_, c in cams.items():
                rgba = c.get_rgba()
                if rgba is not None and rgba.size:
                    cv2.imwrite(f"{args.out}/frames/{name_}_{frame:04d}.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))
            frame += 1
        if status is Status.FAILED:
            detected.append((arm.error_code or 404, arm.error)); say(f"실패: {arm.error}"); break
        if arm.idle:
            say(f"시퀀스 완료 step {step}"); break

    for _ in range(120):                       # 놓은 뒤 안정화
        sim_step(render=False)

    total_spikes = sum(v[1] for v in motion.values())
    for ph, (mx, sp, n) in motion.items():
        if sp or mx > VEL_RATIO_MAX * 0.75:
            say(f"관절 각속도 {ph:34s} 최대 한계의 {mx*100:.0f}%  80% 초과 {sp}스텝 / {n}스텝")
    say(f"관절 각속도 80% 초과 합계 {total_spikes}스텝")
    if total_spikes:
        detected.append((403, f"관절 각속도 한계 80% 초과 {total_spikes}스텝"))

    # 구간별 소요 (웹 클로드 v6 회신 — 사이클 타임 분해)
    dt = world.get_physics_dt()
    seg_steps = {}; pick_grip = 0; mj = 0
    for i, (st, phase, *_r) in enumerate(phase_log):
        end = phase_log[i + 1][0] if i + 1 < len(phase_log) else step
        if phase == "idle":
            continue
        name = phase.split(":")[-1]; seq = phase.split(":")[0] if ":" in phase else ""
        if seq:
            mj = 1
        if phase == "move_joint":
            sg = "복귀" if mj else "홈 이동"    # 작업 시퀀스 이전의 관절 이동(접어 돌기 포함)은 전부 홈 이동
        elif seq == "pick":
            if name == "set_gripper":
                pick_grip += 1; sg = "접근" if pick_grip == 1 else "파지"
            else:
                sg = {"move_joint": "접근", "down": "파지", "attach": "파지", "lift": "운반·손목세우기"}.get(name, "파지")
        elif name == "carry_rotate": sg = "운반·손목세우기"
        elif name == "wedge": sg = "끼우기"
        elif name == "retreat": sg = "후퇴"
        elif seq.startswith("insert") or name == "back": sg = "놓기·빠지기"
        elif seq.startswith("push"): sg = "밀기"
        else: sg = "기타"
        seg_steps[sg] = seg_steps.get(sg, 0) + (end - st)
    order = ["홈 이동", "접근", "파지", "운반·손목세우기", "끼우기", "놓기·빠지기", "밀기", "후퇴", "복귀", "기타"]
    seg_sec = {k: round(seg_steps[k] * dt, 2) for k in order if k in seg_steps}
    say("구간별 소요(초): " + " / ".join(f"{k} {v}" for k, v in seg_sec.items()) + f"  합계 {round(sum(seg_sec.values()), 2)}")
    bb2 = aabb(BOOK)
    inside_y = bb2[1] > y_front - 0.01 and bb2[4] < shelf[4] + 0.01
    inside_x = bb2[0] > shelf[0] and bb2[3] < shelf[3]
    on_floor = abs(bb2[2] - floor_z) < 0.03
    if args.mode == "spine_out":
        upright = abs((bb2[5] - bb2[2]) - 2 * half[1]) < 0.02      # 세로 = 책 길이
        depth_ok = abs((bb2[4] - bb2[1]) - book_h) < 0.02          # 깊이 = 책 폭
        on_floor = on_floor and upright and depth_ok
        say(f"세워짐 {upright} (높이 {bb2[5]-bb2[2]:.3f}) / 깊이 {depth_ok} ({bb2[4]-bb2[1]:.3f}) / 책등 y {bb2[1]:.3f} (선반 앞 {y_front:.3f})")
    placed_ok = bool(inside_x and inside_y and on_floor)
    if not placed_ok and not detected:
        detected.append((409, "최종 배치 검증 실패"))
    error_code = detected[0][0] if detected else 0
    result = {"status": status.value, "error_code": error_code, "errors": detected,
              "error": detected[0][1] if detected else "", "book_aabb_after": np.round(bb2, 3).tolist(),
              "inside_y": bool(inside_y), "inside_x": bool(inside_x), "on_slot_floor": bool(on_floor),
              "joint_over80_steps": int(total_spikes), "segments_sec": seg_sec,
              "joint_peak_ratio_by_phase": {k: round(v[0], 3) for k, v in motion.items()},
              "success": bool(error_code == 0 and placed_ok),
              "phases": phase_log, "log": LOG}
    json.dump(result, open(f"{args.out}/result.json", "w"), indent=1, ensure_ascii=False, default=float)
    say(f"결과 success={result['success']} 오류코드={error_code} 각속도초과={total_spikes} inside_x={inside_x} inside_y={inside_y} on_floor={on_floor} 책AABB={result['book_aabb_after']}")
    for name, c in cams.items():
        sim_step(render=True)
        rgba = c.get_rgba()
        if rgba is not None and rgba.size:
            cv2.imwrite(f"{args.out}/final_{name}.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))

if args.gui or args.selftest_replays > 0:
    # 재생(Play)을 누르면 실행, 정지(Stop) 후 다시 재생하면 처음 상태로 되돌려 다시 실행한다.
    # world.reset() 은 타임라인을 정지→재생해 로봇·책을 레벨에 저장된 초기 상태로 되돌린다
    app.update()
    say("대기 중 — Isaac Sim 의 재생(▶) 버튼을 누르면 시작합니다")
    run_count = 0
    while app.is_running():
        app.update()
        if args.selftest_replays > 0 and not world.is_playing():
            if run_count >= args.selftest_replays:
                break
            world.play()                         # 자체 시험: 재생 버튼 대신
        if not world.is_playing():
            continue
        run_count += 1
        RUN_STATE["count"] = run_count
        say(f"===== {run_count}회차 시작 =====")
        try:
            run_once()
        except RunStopped:
            say(f"{run_count}회차 도중 정지됨")
        except PlanAbort as e:
            say(f"{run_count}회차 중단: {e}")
        except Exception:
            # 정지 타이밍 등으로 예상 못 한 오류가 나도 창을 닫지 않는다. 기록만 남기고 다음 재생을 기다린다
            import traceback
            say(f"{run_count}회차 오류 (재생 상태={world.is_playing()}):\n" + traceback.format_exc())
        # 끝난 뒤에는 재생 상태를 유지하며 장면을 보여준다. 정지하면 다음 재생을 기다린다
        hold = 0
        while app.is_running() and world.is_playing():
            world.step(render=args.gui)
            hold += 1
            if args.selftest_replays > 0 and hold > 60:
                world.stop()                     # 자체 시험: 정지 버튼 대신
        if stage.GetPrimAtPath("/World/pp_grasp_joint").IsValid():
            stage.RemovePrim("/World/pp_grasp_joint")
        say("정지됨 — 다시 재생(▶)하면 처음부터 실행합니다")
else:
    try:
        run_once()
    except PlanAbort as e:
        say(f"중단: {e}")
app.close()
