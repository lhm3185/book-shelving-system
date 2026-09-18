"""트레이 설계용: 손을 아래로 향하게 한 상태에서 손가락 벌림별 손가락 외곽 치수를 잰다."""
import argparse, sys
ap = argparse.ArgumentParser(); ap.add_argument("--usd", required=True); args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
def say(m): sys.stderr.write(f"### {m}\n"); sys.stderr.flush()
open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
R = "/World/ridgeback_franka"
world = World(stage_units_in_meters=1.0); robot = SingleArticulation(prim_path=R, name="rf")
world.reset(); robot.initialize()
fi = [robot.get_dof_index("panda_finger_joint1"), robot.get_dof_index("panda_finger_joint2")]
cache = create_bbox_cache()
def aabb(p):
    cache.Clear(); return np.array(compute_aabb(cache, p, include_children=True), float)
for w in (0.0, 0.02, 0.025, 0.03, 0.04):
    q = robot.get_joint_positions().copy(); q[fi] = w
    for _ in range(90):
        robot.apply_action(ArticulationAction(joint_positions=q)); world.step(render=False)
    lf = aabb(R + "/panda_leftfinger"); rf = aabb(R + "/panda_rightfinger"); hand = aabb(R + "/panda_hand")
    both = np.concatenate([np.minimum(lf[:3], rf[:3]), np.maximum(lf[3:], rf[3:])])
    # 손가락은 손 좌표 y 방향으로 벌어진다. 월드에서 두 손가락 중심 간 거리로 벌림 축을 찾는다
    cl, cr = (lf[:3] + lf[3:]) / 2, (rf[:3] + rf[3:]) / 2
    axis = int(np.argmax(np.abs(cr - cl)))
    size_one = lf[3:] - lf[:3]
    say(f"벌림 {w:.3f}/손가락: 손가락쌍 외곽폭(벌림축 {'xyz'[axis]}) {both[3+axis]-both[axis]:.4f} m | 손가락 하나 크기 {np.round(size_one,4).tolist()} | 손 전체폭 {np.round(hand[3:]-hand[:3],4).tolist()}")
app.close()
