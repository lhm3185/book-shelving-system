"""Nova Carter + M0609(+RG2) 결합 로봇 USD 를 만든다 (Isaac Sim 5.1.0).

만드는 구조
    /World/carter_m0609                 ← articulation root (둘을 **하나의 articulation** 으로)
        /carter                          Nova Carter (클라우드 에셋 참조)
        /arm                             M0609 + OnRobot RG2 (로컬 USD 참조)
        /arm_mount                       고정 조인트: carter chassis_link → arm base_link

주의한 것
    - 두 에셋 모두 자기 articulation root 를 갖고 있다. 그대로 두면 **articulation 이 둘**이 되어
      한 로봇으로 제어할 수 없다. 자식 쪽 root 를 걷어내고 맨 위에 하나만 둔다.
    - M0609 USD 는 `root_joint`(고정)로 월드에 박혀 있다. 그대로 두면 팔이 카터를 따라가지 않으므로
      비활성화하고 대신 carter 상판에 고정 조인트로 붙인다.

실행
    ISAAC_ENTRY=simulation/isaac/tools/build_carter_m0609.py ./scripts/run_isaac_tool.sh \\
        --out ~/Desktop/carter_m0609.usd
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--carter", default="", help="비우면 Isaac 에셋 루트의 nova_carter.usd")
ap.add_argument("--arm", default=os.path.expanduser("~/Desktop/Collected_m0609_gripper.usd"))
ap.add_argument("--arm-prim", default="/World/m0609",
                help="팔 USD 안에서 가져올 prim. 그리퍼가 /World 아래 **형제 prim** 으로 있는 파일이면 "
                     "`/World` 를 주어야 그리퍼까지 들어온다 (실측: 안 주면 그리퍼 관절 0개)")
ap.add_argument("--out", default=os.path.expanduser("~/Desktop/carter_m0609.usd"))
ap.add_argument("--mount-xyz", type=float, nargs=3, default=[0.0, 0.0, 0.555],
                help="팔을 올릴 위치. 기본값은 **카터 상판 앞쪽** — 상판 중앙(자동)에 놓으면 "
                     "팔이 카터 구조물과 간섭해 joint_1 이 막힌다 (2026-09-18 실측)")
ap.add_argument("--mount-yaw", type=float, default=0.0, help="팔 방향 (도)")
ap.add_argument("--riser", choices=["on", "off"], default="on",
                help="on(기본): 팔 받침판과 카터 상판 사이를 **어댑터 판**으로 채운다. "
                     "카터 상판에는 평평한 20 cm 자리가 없어 실제로도 어댑터 없이는 못 얹는다")
ap.add_argument("--riser-base", type=float, default=0.430,
                help="어댑터가 서는 상판 높이 (m). 장착점 (0,0) 주변 실측값 0.430")
ap.add_argument("--plate-offset", type=float, default=0.018,
                help="팔 원점에서 **받침판 밑면**까지 (m). AABB 최저점(0.510)은 옆으로 빠지는 "
                     "케이블 호스라 쓰면 안 된다 — 장착점 위로 광선을 쏴 잰 값 0.018")
# 위치 제어 게인: 1e5 면 어깨·팔꿈치가 중력에 처지고, 1e6 은 불안정했다. 1e7/1e5 에서 오차 0.002 rad (실측)
ap.add_argument("--arm-stiffness", type=float, default=1.0e7)
ap.add_argument("--arm-damping", type=float, default=1.0e5)
ap.add_argument("--grip-stiffness", type=float, default=1.0e3)
ap.add_argument("--grip-damping", type=float, default=1.0e2)
ap.add_argument("--zero-arm-targets", choices=["on", "off"], default="on",
                help="on(기본): 팔 드라이브 목표를 시작 자세(0°)에 맞춘다. off 로 두면 "
                     "재생하는 순간 팔이 90° 로 튀며 로봇이 넘어진다 (2026-09-18 실측)")
ap.add_argument("--fix-base", choices=["on", "off"], default="off",
                help="off(기본): 주행 가능 — AMR 담당이 라이다·주행 시험을 이어서 할 수 있어야 한다. "
                     "on: 베이스를 월드에 고정 (로봇팔 단독 시연용)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import math  # noqa: E402

import numpy as np  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, is_stage_loading  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

ROOT = "/World/carter_m0609"
CARTER = ROOT + "/carter"
ARM = ROOT + "/arm"


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


carter_url = args.carter or (get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd")
say(f"카터: {carter_url}")
say(f"팔  : {args.arm} ({args.arm_prim})")
if not os.path.exists(args.arm):
    say("팔 USD 가 없다")
    app.close()
    sys.exit(1)

import omni.usd  # noqa: E402
omni.usd.get_context().new_stage()
stage = get_current_stage()
while is_stage_loading():
    app.update()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
UsdGeom.Xform.Define(stage, ROOT)

add_reference_to_stage(carter_url, CARTER)
app.update()
while is_stage_loading():
    app.update()

arm_prim = stage.DefinePrim(ARM, "Xform")
arm_prim.GetReferences().AddReference(args.arm, args.arm_prim)
app.update()
while is_stage_loading():
    app.update()

# 팔 USD 전체(/World)를 가져온 경우, 같이 딸려 온 바닥·그래프는 끈다
DROP = {"GroundPlane", "ActionGraph", "Graph", "Environment", "Render", "OmniverseKit",
        "defaultLight", "OmniverseGlobalRenderSettings", "Vars"}
dropped = []
for child in stage.GetPrimAtPath(ARM).GetChildren():
    if child.GetName() in DROP:
        child.SetActive(False)
        dropped.append(child.GetName())
if dropped:
    say(f"딸려 온 prim 비활성화: {dropped}")

cache = create_bbox_cache()


def aabb(path):
    cache.Clear()
    return np.array(compute_aabb(cache, path, include_children=True), float)


cb = aabb(CARTER)
say(f"카터 크기 {np.round(cb[3:] - cb[:3], 3).tolist()}, 윗면 z {cb[5]:.3f}")

# 1) 팔을 카터 상판에 올린다
if args.mount_xyz == [0.0, 0.0, 0.0]:
    mount = np.array([(cb[0] + cb[3]) / 2, (cb[1] + cb[4]) / 2, cb[5]])
else:
    mount = np.array(args.mount_xyz, float)
xf = UsdGeom.Xformable(arm_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in mount]))
if args.mount_yaw:
    xf.AddRotateZOp().Set(float(args.mount_yaw))
say(f"팔 장착 위치 {np.round(mount, 3).tolist()}, yaw {args.mount_yaw}°")

# 2) 팔을 월드에 박아 두던 고정 조인트를 끈다 (카터를 따라 움직여야 한다)
disabled = []
for p in Usd.PrimRange(stage.GetPrimAtPath(ARM)):
    # root_joint 만 끈다. AssemblerFixedJoint 는 **그리퍼를 팔에 붙여 두는** 조인트라 끄면
    # 그리퍼가 articulation 에서 떨어져 나간다 (실측: 그리퍼 관절 0개)
    if p.IsA(UsdPhysics.FixedJoint) and p.GetName() == "root_joint":
        UsdPhysics.Joint(p).CreateJointEnabledAttr().Set(False)
        disabled.append(p.GetName())
say(f"끈 고정 조인트: {disabled or '없음'}")

# 3) articulation root 정리 — 맨 위 하나만 남긴다
removed = []
for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and p.GetPath().pathString != ROOT:
        p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        removed.append(p.GetPath().pathString)
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(ROOT))
say(f"자식 articulation root 제거: {removed}")


def find(path_root, names):
    hits = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(path_root)):
        if p.GetName() in names:
            hits.append(p.GetPath().pathString)
    return hits


chassis = find(CARTER, {"chassis_link"})
arm_base = find(ARM, {"base_link", "base_0", "link_0"})
say(f"카터 chassis: {chassis[:1]}, 팔 base: {arm_base[:1]}")
if not chassis or not arm_base:
    say("연결할 링크를 찾지 못했다")
    app.close()
    sys.exit(1)

# 4) 고정 조인트로 붙인다 (carter chassis_link → arm base_link)
joint = UsdPhysics.FixedJoint.Define(stage, ROOT + "/arm_mount")
joint.CreateBody0Rel().SetTargets([chassis[0]])
joint.CreateBody1Rel().SetTargets([arm_base[0]])
# 조인트의 로컬 위치는 **각 바디의 프레임 원점 기준**이다. 예전에는 chassis 의 AABB 중심을
# 기준으로 계산했는데, chassis_link 의 프레임 원점은 (0,0,0) 이고 AABB 중심은 (-0.234,0,0.315)
# 이라 **39 cm 어긋났다.** 재생하는 순간 물리가 팔을 그만큼 끌어당겨 로봇이 넘어지고,
# 스케일을 키우면 그 힘이 커져 날아갔다 (2026-09-18 실측). 실제 상대 변환에서 직접 뽑는다.
ch_w = UsdGeom.Xformable(stage.GetPrimAtPath(chassis[0])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
ab_w = UsdGeom.Xformable(stage.GetPrimAtPath(arm_base[0])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
rel = ab_w * ch_w.GetInverse()
local = np.array(rel.ExtractTranslation(), float)
qrel = rel.ExtractRotationQuat()
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in local]))
joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(qrel.GetReal()),
                                         Gf.Vec3f(*[float(v) for v in qrel.GetImaginary()])))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
say(f"고정 조인트 생성: {chassis[0].split('/')[-1]} → {arm_base[0].split('/')[-1]}, "
    f"로컬 {np.round(local, 4).tolist()} (프레임 원점 기준)")

# 4b) 장착 어댑터(리서) — 팔이 공중에 떠 보이는 것을 없앤다
#
# 왜 낮추지 않고 받침대를 넣나 (2026-09-18 실측):
#   - 장착점 (0,0) 바로 아래는 상판이 아니라 **front_RPLidar** 다 (윗면 0.436, 주변 상판 0.430).
#     팔을 닿을 때까지 내리면 라이다를 깔고 앉는다. AMR 담당이 그 라이다로 주행 시험 중이다.
#   - 뒤쪽 상판(0.464)은 평평하지만 그 위에 **XT_32 3D 라이다**(x -0.389~-0.180, 윗면 0.554)가 서 있다.
#     팔 받침판이 20.6 cm 라 뒤로 옮기면 XT_32 와 겹친다. 예전에 joint_1 이 막힌 원인이 이것이다.
#   - 즉 **평평한 20 cm 자리가 카터 상판에 없다.** 실제로도 어댑터 판 없이는 못 얹는다.
# 그래서 팔 높이는 그대로 두고(= 기존 좌표·IK 작업이 그대로 살아 있다) 사이를 받침대로 채운다.
# 시각 전용이다 — 팔은 지금까지처럼 고정 조인트가 잡는다. 충돌체를 넣으면 물리만 복잡해진다.
if args.riser == "on":
    plate_z = float(mount[2]) - args.plate_offset   # 팔 받침판 밑면 (실측 오프셋)
    base_z = float(args.riser_base)                # 받침대가 서는 상판 높이 (실측 0.430)
    chx = UsdGeom.Xformable(stage.GetPrimAtPath(chassis[0]))
    w2l = chx.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    riser = UsdGeom.Xform.Define(stage, chassis[0] + "/arm_riser")
    riser.AddTransformOp().Set(w2l)                # 월드 좌표로 그리되 chassis 를 따라 움직이게

    def slab(name, cx, cy, cz, sx, sy, sz):
        c = UsdGeom.Cube.Define(stage, chassis[0] + "/arm_riser/" + name)
        c.CreateSizeAttr().Set(2.0)                # 기본 큐브는 -1~+1 → 스케일이 곧 반치수
        x = UsdGeom.Xformable(c)
        x.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
        x.AddScaleOp().Set(Gf.Vec3f(sx / 2, sy / 2, sz / 2))

    top_t = 0.008
    slab("top_plate", float(mount[0]) - 0.012, float(mount[1]), plate_z - top_t / 2,
         0.206, 0.180, top_t)
    # 옆판은 라이다 시야(y ±0.039)를 피해 바깥쪽에 세운다
    for sgn, nm in ((+1, "wall_left"), (-1, "wall_right")):
        slab(nm, float(mount[0]) - 0.010, float(mount[1]) + sgn * 0.075,
             (base_z + plate_z - top_t) / 2, 0.180, 0.012, plate_z - top_t - base_z)
    say(f"장착 어댑터 생성: 상판 {base_z:.3f} → 받침판 {plate_z:.3f} "
        f"(높이 {(plate_z - base_z) * 1000:.0f} mm, 시각 전용)")

# 5) 팔 구동 게인 — URDF 임포트 값이 너무 낮아(강성 26~102) 위치 지령을 따라가지 못한다 (실측).
#    위치 제어가 되도록 올린다. 그리퍼는 물체를 쥐는 힘이라 낮게 둔다.
ARM_JOINTS = {f"joint_{i}" for i in range(1, 7)}
GRIP_JOINTS = {"finger_joint", "left_inner_knuckle_joint", "left_outer_knuckle_joint",
               "right_inner_knuckle_joint", "right_inner_finger_joint", "left_inner_finger_joint"}
tuned = []
retargeted = []
for p in Usd.PrimRange(stage.GetPrimAtPath(ARM)):
    name = p.GetName()
    if name not in ARM_JOINTS and name not in GRIP_JOINTS:
        continue
    drive = UsdPhysics.DriveAPI.Get(p, "angular") or UsdPhysics.DriveAPI.Apply(p, "angular")
    k, c = ((args.arm_stiffness, args.arm_damping) if name in ARM_JOINTS
            else (args.grip_stiffness, args.grip_damping))
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(float(k))
    drive.CreateDampingAttr().Set(float(c))
    # 드라이브 목표를 **재생 시작 자세(0°)에 맞춘다**.
    # URDF 임포트가 joint_3·joint_5 의 목표를 90° 로 써 두었는데 실제 시작 자세는 0° 다.
    # 그대로 두면 강성 1e7 이 재생 순간 팔을 85° 확 끌어당겨 로봇이 통째로 넘어진다
    # (2026-09-18 실측). 무게중심 문제가 아니었다.
    # 반대로 시작 자세를 90° 로 바꾸는 방법(JointStateAPI)은 로봇 전체를 90° 눕혀 버려서 못 쓴다.
    if name in ARM_JOINTS and args.zero_arm_targets == "on":
        before = drive.GetTargetPositionAttr().Get()
        drive.CreateTargetPositionAttr().Set(0.0)
        if before not in (None, 0.0):
            retargeted.append(f"{name} {before}°→0°")
    tuned.append(name)
say(f"구동 게인 조정 {len(tuned)}개 (팔 강성 {args.arm_stiffness:.0e}, 그리퍼 {args.grip_stiffness:.0e})")
if retargeted:
    say(f"드라이브 목표를 시작 자세에 맞춤: {retargeted}")

# 6) 시연은 로봇 정지 상태다. 바퀴를 속도 0 으로 눌러도 미끄러지므로(실측 4.1 rad) 월드에 고정한다
if args.fix_base == "on":
    fix = UsdPhysics.FixedJoint.Define(stage, ROOT + "/base_fix")
    fix.CreateBody1Rel().SetTargets([chassis[0]])     # body0 없음 = 월드
    say("베이스를 월드에 고정 (--fix-base off 로 끄면 주행 가능)")

joints = [(p.GetName(), p.GetTypeName()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)) if p.IsA(UsdPhysics.Joint)]
rev = [n for n, t in joints if "Revolute" in str(t)]
say(f"관절 합계 {len(joints)}개 (회전 {len(rev)}개): {rev[:12]}")

os.makedirs(os.path.dirname(args.out), exist_ok=True)
stage.GetRootLayer().Export(args.out)
say(f"저장: {args.out}")
app.close()
