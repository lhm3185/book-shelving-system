"""주행 안전 자세(stow) 검증.

확인: ① 팔 전체가 AMR 베이스 xy 영역 안에 드는가  ② 데크 윗면·주변 물체와 겹치지 않는가
      ③ 홈↔stow 전환에서 관절 각속도가 URDF 한계의 80% 이하인가
GPU PC:
    ~/isaacsim/python.sh ~/arm/isaac/verify_stow.py --usd ~/Desktop/ing_library_env_v3.usd --out /tmp/stow
"""
import argparse, json, math, os, sys
ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True); ap.add_argument("--out", default="/tmp/stow")
ap.add_argument("--stow", type=float, nargs=7, default=None, help="검증할 stow 관절값 (기본: arm_config.yaml)")
args = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import cv2, numpy as np, yaml
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera

os.makedirs(args.out, exist_ok=True)
def say(m): sys.stderr.write(f"### {m}\n"); sys.stderr.flush()
R = "/World/ridgeback_franka"
ARM = [f"panda_joint{i}" for i in range(1, 8)]
# Franka 속도 한계 (GPU PC lula_franka_gen.urdf 실측): joint1~4 2.175, joint5~7 2.61 rad/s
VEL_LIMIT = np.array([2.175] * 4 + [2.61] * 3)
ARM_LINKS = [f"panda_link{i}" for i in range(0, 8)] + ["panda_hand", "panda_leftfinger", "panda_rightfinger"]

conf = yaml.safe_load(open(os.path.expanduser("~/arm/arm_config.yaml")))
q_home = np.array(conf["poses"]["home"], float)
q_stow = np.array(args.stow if args.stow else conf["poses"]["stow"], float)

open_stage(args.usd); app.update()
while is_stage_loading(): app.update()
stage = get_current_stage()
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
robot = SingleArticulation(prim_path=R, name="rf")
cam = Camera(prim_path="/World/stow_cam", resolution=(960, 540))
world.reset(); robot.initialize(); cam.initialize()
idx = [robot.get_dof_index(j) for j in ARM]
base_idx = [robot.get_dof_index(j) for j in robot.dof_names if j.startswith("dummy_base")]
base_hold = robot.get_joint_positions()[base_idx]
cache = create_bbox_cache()
def aabb(path):
    cache.Clear(); return np.array(compute_aabb(cache, path, include_children=True), float)

base = aabb(R + "/base_link")
say(f"AMR 베이스 AABB {np.round(base,3).tolist()}")
c = (base[:3] + base[3:]) / 2
set_camera_view(eye=c + np.array([-1.6, -1.8, 1.4]), target=c + np.array([0, 0, 0.4]), camera_prim_path="/World/stow_cam")

def goto(q_target, speed=0.8, label="", shots=4):
    """관절 공간 보간으로 이동하며 관절 각속도를 기록한다"""
    q = robot.get_joint_positions().copy()
    start = q[idx].copy(); dist = float(np.max(np.abs(q_target - start)))
    n = max(1, int(math.ceil(dist / (speed / 60))))
    prev = start.copy(); peak = np.zeros(7); over = 0
    for k in range(1, n + 120):
        t = min(1.0, k / n)
        tgt = robot.get_joint_positions().copy()
        tgt[idx] = start + (q_target - start) * t; tgt[base_idx] = base_hold
        robot.apply_action(ArticulationAction(joint_positions=tgt))
        world.step(render=(k % max(1, (n + 120) // shots) == 0))
        now = robot.get_joint_positions()[idx]
        vel = np.abs(now - prev) * 60; prev = now.copy()
        peak = np.maximum(peak, vel); over += int(np.any(vel > 0.8 * VEL_LIMIT))
        if k % max(1, (n + 120) // shots) == 0:
            rgba = cam.get_rgba()
            if rgba is not None and rgba.size:
                cv2.imwrite(f"{args.out}/{label}_{k:04d}.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))
        if t >= 1.0 and float(np.max(np.abs(now - q_target))) < 0.02:
            break
    err = float(np.max(np.abs(robot.get_joint_positions()[idx] - q_target)))
    ratio = peak / VEL_LIMIT
    say(f"{label:12s} 최대 각속도/한계 {np.round(ratio,2).tolist()}  80% 초과 스텝 {over}  도달오차 {err:.3f} rad")
    return {"peak_ratio": ratio.round(3).tolist(), "over80": over, "reach_err": err}

for _ in range(60): world.step(render=False)
res = {}
res["to_home"] = goto(q_home, label="시작→홈")
res["home_to_stow"] = goto(q_stow, label="홈→stow")
for _ in range(60): world.step(render=True)

# stow 자세 판정
links = {l: aabb(f"{R}/{l}") for l in ARM_LINKS}
arm_lo = np.min([b[:3] for b in links.values()], axis=0); arm_hi = np.max([b[3:] for b in links.values()], axis=0)
margin = 0.0
inside_xy = bool(arm_lo[0] >= base[0] - margin and arm_hi[0] <= base[3] + margin and arm_lo[1] >= base[1] - margin and arm_hi[1] <= base[4] + margin)
deck_top = base[5]
low_links = {l: round(float(b[2]), 3) for l, b in links.items() if l not in ("panda_link0", "panda_link1") and b[2] < deck_top + 0.02}
say(f"stow 팔 AABB {np.round(np.concatenate([arm_lo, arm_hi]),3).tolist()}  최고높이 {arm_hi[2]:.3f} m")
say(f"베이스 xy 안에 들어옴: {inside_xy}  (팔 x {arm_lo[0]:.3f}~{arm_hi[0]:.3f} / 베이스 x {base[0]:.3f}~{base[3]:.3f}, 팔 y {arm_lo[1]:.3f}~{arm_hi[1]:.3f} / 베이스 y {base[1]:.3f}~{base[4]:.3f})")
say(f"데크 윗면({deck_top:.3f}) 2cm 이내로 내려간 링크: {low_links or '없음'}")

# 다른 물체와 겹침 (책·선반·칸막이)
# 묶음(Scope) 단위로 박스를 잡으면 칸막이 사이 빈 공간까지 덮어 거짓 겹침이 난다 → 칸막이를 하나씩 검사
others = ["/World/books/book_encyclopedia_set_01_2k__book_encyclopedia_set_01_book15",
          "/World/bookshelves/shelf_brown__book_shelf_01"]
if stage.GetPrimAtPath("/World/fixtures").IsValid():
    others += [str(ch.GetPath()) for ch in stage.GetPrimAtPath("/World/fixtures").GetChildren()]
def overlap(a, b):
    return bool(np.all(a[:3] < b[3:]) and np.all(b[:3] < a[3:]))
hits = []
for o in others:
    if not stage.GetPrimAtPath(o).IsValid(): continue
    ob = aabb(o)
    for l, b in links.items():
        if overlap(b, ob): hits.append((l, o.split("/")[-1]))
say(f"stow 자세에서 AABB 가 겹치는 (링크, 물체): {hits or '없음'}")

res["stow_to_home"] = goto(q_home, label="stow→홈")
res.update({"stow": q_stow.tolist(), "inside_base_xy": inside_xy, "arm_aabb": np.concatenate([arm_lo, arm_hi]).round(3).tolist(),
            "base_aabb": base.round(3).tolist(), "low_links": low_links, "overlaps": hits})
ok = inside_xy and not low_links and not hits and all(res[k]["over80"] == 0 and res[k]["reach_err"] < 0.03 for k in ("home_to_stow", "stow_to_home"))
res["pass"] = bool(ok)
json.dump(res, open(f"{args.out}/stow.json", "w"), indent=1)
say(f"STOW 판정: {'통과' if ok else '불통과'}")
app.close()
