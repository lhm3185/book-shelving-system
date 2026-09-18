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
ap.add_argument("--arm-prim", default="/World/m0609", help="팔 USD 안에서 가져올 prim")
ap.add_argument("--out", default=os.path.expanduser("~/Desktop/carter_m0609.usd"))
ap.add_argument("--mount-xyz", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                help="팔을 올릴 위치 (비우면 carter 상판 중앙 자동)")
ap.add_argument("--mount-yaw", type=float, default=0.0, help="팔 방향 (도)")
ap.add_argument("--arm-stiffness", type=float, default=1.0e5)
ap.add_argument("--arm-damping", type=float, default=1.0e4)
ap.add_argument("--grip-stiffness", type=float, default=1.0e3)
ap.add_argument("--grip-damping", type=float, default=1.0e2)
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
ch = aabb(chassis[0])
ch_mid = (ch[:3] + ch[3:]) / 2
local = mount - ch_mid
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in local]))
joint.CreateLocalRot0Attr().Set(Gf.Quatf(math.cos(math.radians(args.mount_yaw) / 2), 0, 0,
                                         math.sin(math.radians(args.mount_yaw) / 2)))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
say(f"고정 조인트 생성: {chassis[0].split('/')[-1]} → {arm_base[0].split('/')[-1]}, 로컬 {np.round(local, 3).tolist()}")

# 5) 팔 구동 게인 — URDF 임포트 값이 너무 낮아(강성 26~102) 위치 지령을 따라가지 못한다 (실측).
#    위치 제어가 되도록 올린다. 그리퍼는 물체를 쥐는 힘이라 낮게 둔다.
ARM_JOINTS = {f"joint_{i}" for i in range(1, 7)}
GRIP_JOINTS = {"finger_joint", "left_inner_knuckle_joint", "left_outer_knuckle_joint",
               "right_inner_knuckle_joint", "right_inner_finger_joint", "left_inner_finger_joint"}
tuned = []
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
    tuned.append(name)
say(f"구동 게인 조정 {len(tuned)}개 (팔 강성 {args.arm_stiffness:.0e}, 그리퍼 {args.grip_stiffness:.0e})")

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
