"""팀 프레임 매핑용 실측: panda_link0/panda_hand/right_gripper 상대 자세, 레벨의 ROS2 TF 발행 그래프 유무."""
import argparse, sys
ap = argparse.ArgumentParser(); ap.add_argument("--usd", required=True); args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from pxr import Usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
def say(m): sys.stderr.write(f"### {m}\n"); sys.stderr.flush()
open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
R = "/World/ridgeback_franka"

# 레벨 안 OmniGraph / ROS2 노드
ros_nodes = []
for p in stage.Traverse():
    t = p.GetTypeName()
    attr = p.GetAttribute("node:type")
    nt = attr.Get() if attr and attr.IsValid() else None
    if nt and "ros2" in str(nt).lower():
        ros_nodes.append((str(p.GetPath()), nt))
say(f"레벨의 ROS2 OmniGraph 노드: {len(ros_nodes)}개 {ros_nodes[:8]}")

world = World(stage_units_in_meters=1.0); robot = SingleArticulation(prim_path=R, name="rf")
world.reset(); robot.initialize()
for _ in range(10): world.step(render=False)

def T_of(path):
    p, q = SingleXFormPrim(path).get_world_pose()
    w, x, y, z = q
    Rm = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = p; return T
def rel(parent, child):
    return np.linalg.inv(parent) @ child

T_base = T_of(R + "/base_link"); T_l0 = T_of(R + "/panda_link0"); T_hand = T_of(R + "/panda_hand"); T_l7 = T_of(R + "/panda_link7")
say(f"base_link → panda_link0 평행이동 {np.round(rel(T_base, T_l0)[:3,3],4).tolist()} 회전 단위행렬? {np.allclose(rel(T_base,T_l0)[:3,:3], np.eye(3), atol=1e-4)}")
say(f"panda_link7 → panda_hand 평행이동 {np.round(rel(T_l7, T_hand)[:3,3],4).tolist()}")

cfg = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
lula = LulaKinematicsSolver(**cfg)
l0p, l0q = SingleXFormPrim(R + "/panda_link0").get_world_pose(); lula.set_robot_base_pose(l0p, l0q)
names = lula.get_joint_names(); q = np.array([robot.get_joint_positions()[robot.get_dof_index(n)] for n in names])
for frame in ("panda_hand", "right_gripper"):
    p, Rm = lula.compute_forward_kinematics(frame, q)
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = p
    r = rel(T_hand, T)
    Rr = r[:3, :3]
    w = np.sqrt(max(0.0, 1 + Rr[0,0] + Rr[1,1] + Rr[2,2])) / 2
    qx = np.copysign(np.sqrt(max(0.0, 1 + Rr[0,0] - Rr[1,1] - Rr[2,2])) / 2, Rr[2,1] - Rr[1,2])
    qy = np.copysign(np.sqrt(max(0.0, 1 - Rr[0,0] + Rr[1,1] - Rr[2,2])) / 2, Rr[0,2] - Rr[2,0])
    qz = np.copysign(np.sqrt(max(0.0, 1 - Rr[0,0] - Rr[1,1] + Rr[2,2])) / 2, Rr[1,0] - Rr[0,1])
    yaw = np.degrees(np.arctan2(Rr[1,0], Rr[0,0]))
    say(f"panda_hand → {frame}: 평행이동 {np.round(r[:3,3],4).tolist()} 쿼터니언 xyzw {np.round([qx,qy,qz,w],4).tolist()} (z축 회전 {yaw:.1f}°) R={np.round(Rr,3).tolist()}")
say(f"Lula 프레임 목록 일부: {[f for f in lula.get_all_frame_names() if 'hand' in f or 'gripper' in f or 'link8' in f]}")
app.close()
