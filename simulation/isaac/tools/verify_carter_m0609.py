"""결합 로봇(Nova Carter + M0609)이 **하나의 articulation 으로 제어되는지** 확인한다.

보는 것
    1. articulation 이 하나로 잡히는가, DOF 이름·개수
    2. 팔 관절(joint_1~6)을 지령하면 실제로 움직이는가
    3. 바퀴를 멈춰 둘 수 있는가 (시연은 로봇 정지 상태)
    4. M0609 Lula 설정으로 IK 가 풀리는가 (기존 Franka 설정 대신)

실행
    ISAAC_ENTRY=simulation/isaac/tools/verify_carter_m0609.py ./scripts/run_isaac_tool.sh \\
        --usd ~/Desktop/carter_m0609.usd
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/Desktop/carter_m0609.usd"))
ap.add_argument("--robot", default="/World/carter_m0609")
ap.add_argument("--descriptor", default=os.path.expanduser(
    "~/Isaac_Sim_b-1/src_pra/M0609/descriptor/m0609_description.yaml"))
ap.add_argument("--urdf", default=os.path.expanduser(
    "~/Isaac_Sim_b-1/src_pra/M0609/doosan-robot2/urdf/m0609.urdf"))
ap.add_argument("--ee", default="link_6", help="IK 를 풀 말단 링크 이름")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


open_stage(args.usd)
app.update()
while is_stage_loading():
    app.update()

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
world.reset()

robot = SingleArticulation(prim_path=args.robot, name="carter_m0609")
robot.initialize()
names = list(robot.dof_names)
say(f"articulation 1개 인식, DOF {len(names)}개")
say(f"  관절: {names}")

# DOF 순서는 USD 순서라 joint_1~6 이 섞여 나온다. 번호로 정렬해야 지령이 엉키지 않는다 (실측)
arm = sorted([n for n in names if n.startswith("joint_") and n.split("_")[-1].isdigit()
              and int(n.split("_")[-1]) <= 6], key=lambda n: int(n.split("_")[-1]))
wheels = [n for n in names if "wheel" in n or "caster" in n or "swing" in n]
grip = [n for n in names if "finger" in n or "knuckle" in n]
say(f"  팔 {len(arm)}개 {arm} / 바퀴·캐스터 {len(wheels)}개 / 그리퍼 {len(grip)}개 {grip}")
if len(arm) != 6:
    say("팔 관절 6개를 찾지 못했다")
    app.close()
    sys.exit(1)

idx_arm = [robot.get_dof_index(n) for n in arm]
idx_wheel = [robot.get_dof_index(n) for n in wheels]

for _ in range(60):
    world.step(render=False)

# 바퀴 고정 (시연은 정지 상태)
q = robot.get_joint_positions()
wheel_hold = q[idx_wheel].copy() if idx_wheel else None

# 팔 관절 지령 → 실제로 움직이는가
start = robot.get_joint_positions()[idx_arm].copy()
target = start + np.array([0.3, -0.2, 0.2, 0.0, 0.3, 0.0])
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
# 바퀴는 속도 구동이라 위치 지령으로는 세워지지 않는다 → 속도 0 을 준다 (실측: 위치 지령 시 1.4 rad 밀림)
if idx_wheel:
    vel = np.zeros(len(names))
    robot.apply_action(ArticulationAction(joint_velocities=vel, joint_indices=np.arange(len(names))))
for _ in range(240):
    cmd = np.full(len(names), np.nan)
    cmd[idx_arm] = target
    act = ArticulationAction(joint_positions=target, joint_indices=np.array(idx_arm))
    robot.apply_action(act)
    if idx_wheel:
        robot.apply_action(ArticulationAction(joint_velocities=np.zeros(len(idx_wheel)),
                                              joint_indices=np.array(idx_wheel)))
    world.step(render=False)
end = robot.get_joint_positions()[idx_arm]
err = float(np.max(np.abs(end - target)))
moved = float(np.max(np.abs(end - start)))
say(f"팔 지령 추종: 이동 {moved:.3f} rad, 목표 오차 {err:.3f} rad "
    f"→ {'정상' if moved > 0.1 and err < 0.15 else '확인 필요'}")
if idx_wheel:
    wheel_drift = float(np.max(np.abs(robot.get_joint_positions()[idx_wheel] - wheel_hold)))
    say(f"바퀴 고정: 편차 {wheel_drift:.4f} rad → {'정상' if wheel_drift < 0.05 else '확인 필요'}")

# M0609 Lula 설정으로 IK
say(f"Lula 설정: descriptor={os.path.exists(args.descriptor)}, urdf={os.path.exists(args.urdf)}")
if os.path.exists(args.descriptor) and os.path.exists(args.urdf):
    from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver  # noqa: E402
    try:
        solver = LulaKinematicsSolver(robot_description_path=args.descriptor, urdf_path=args.urdf)
        frames = solver.get_all_frame_names()
        say(f"  IK 프레임 {len(frames)}개: {frames[:8]}")
        ee = args.ee if args.ee in frames else frames[-1]
        q0 = robot.get_joint_positions()[idx_arm]
        pos, rot = solver.compute_forward_kinematics(ee, q0)
        say(f"  FK({ee}) 위치 {np.round(pos, 3).tolist()}")
        # Lula IK 는 자세를 **쿼터니언(w,x,y,z)** 으로 받는다. FK 가 준 3x3 행렬을 그대로 주면
        # "index 3 is out of bounds" 로 실패한다 (실측)
        m = np.asarray(rot, float)
        t = float(np.trace(m))
        if t > 0:
            sq = np.sqrt(t + 1.0) * 2
            quat = np.array([0.25 * sq, (m[2, 1] - m[1, 2]) / sq, (m[0, 2] - m[2, 0]) / sq,
                             (m[1, 0] - m[0, 1]) / sq])
        else:
            i = int(np.argmax(np.diag(m))); j, k = (i + 1) % 3, (i + 2) % 3
            sq = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
            quat = np.zeros(4)
            quat[0] = (m[k, j] - m[j, k]) / sq
            quat[i + 1] = 0.25 * sq
            quat[j + 1] = (m[j, i] + m[i, j]) / sq
            quat[k + 1] = (m[k, i] + m[i, k]) / sq
        quat = quat / np.linalg.norm(quat)
        target_pos = np.array(pos) + np.array([0.05, 0.0, -0.05])
        sol, ok = solver.compute_inverse_kinematics(ee, target_pos, quat, warm_start=q0)
        say(f"  IK 목표 {np.round(target_pos, 3).tolist()} → {'성공' if ok else '실패'}"
            + (f", 관절 변화 최대 {float(np.max(np.abs(sol - q0))):.3f} rad" if ok else ""))
    except Exception as e:                       # noqa: BLE001
        say(f"  Lula 설정 로드 실패: {str(e)[:160]}")
else:
    say("  descriptor/urdf 경로를 찾지 못했다")

app.close()
