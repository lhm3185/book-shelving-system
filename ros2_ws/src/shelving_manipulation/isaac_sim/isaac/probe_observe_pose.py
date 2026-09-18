"""서가 관측 자세 찾기·검증 (웹 클로드 v7 회신 A1: 1차에 호출하지 않아도 지금 정의·충돌 검증).

가정 (카메라가 레벨에 아직 없다):
- RealSense 는 손목(panda_hand)에 달리고 광축 = 그리퍼 접근축, 가로 화각 90.5°(학습 데이터와 같음), 640×480
- 장착 오프셋은 미정이라 panda_hand 원점에 둔다. 확정되면 이 스크립트의 CAM_OFFSET 만 바꿔 다시 돌린다

순서
1. 후보(서가 앞 거리 × 높이 × 아래 기울기)마다 IK → 순간이동으로 링크-고정 충돌체 AABB 겹침 검사 → 목표 4칸 화면 안 여부
2. 통과 후보 중 홈에서 관절 거리가 가장 짧은 것을 실제로 이동 (관절 각속도·겹침 감시) → 1초 정지 → 손목 카메라 촬영 → 홈 복귀
결과: /tmp/observe/result.json, /tmp/observe/*.jpg
"""
import argparse
import json
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/Desktop/ing_library_env_v3.usd"))
ap.add_argument("--tray", default=os.path.expanduser("~/book_dataset/assets/tray/tray_v1.usdc"))
ap.add_argument("--out", default="/tmp/observe")
ap.add_argument("--camera-prim", default="",
                help="레벨 로봇에 이미 달린 카메라 prim (있으면 가정 카메라 대신 이것의 실제 장착 위치·방향을 쓴다)")
ap.add_argument("--hfov", type=float, default=90.5)
ap.add_argument("--res", type=int, nargs=2, default=[640, 480])
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from book_scene import BookScene, R, VEL_LIMIT  # noqa: E402
from arm_primitives import MoveJoint, Wait  # noqa: E402
from arm_geometry import R_from_quat  # noqa: E402

os.makedirs(args.out, exist_ok=True)
LOG = []


def say(m):
    LOG.append(m); sys.stderr.write(f"### {m}\n"); sys.stderr.flush()


PLACE_DX = [-0.51, -0.43, -0.35, -0.27]
HFOV = math.radians(args.hfov); W_PX, H_PX = args.res
FX = (W_PX / 2) / math.tan(HFOV / 2)
CAM_OFFSET = np.zeros(3)            # panda_hand 좌표계 기준 카메라 위치 (장착 확정 시 수정)

scene = BookScene(app, args.usd, args.tray, [2.36, -2.94], 6, PLACE_DX, say)
world, arm, robot, st = scene.world, scene.arm, scene.robot, scene.stage
link0 = scene.l0p
floor_z = 0.355 * 1.4
targets = [np.array([link0[0] + dx, scene.shelf_front_y + 0.024 + scene.W / 2, floor_z + scene.L / 2]) for dx in PLACE_DX]
slot_top = floor_z + 0.49          # 1번 칸 바닥~윗판 (선반 z×1.4)

# 카메라: 레벨에 실제 카메라가 있으면 그것을 쓰고(장착 위치·방향을 손 좌표계로 환산),
# 없으면 예전처럼 panda_hand 원점에 가정 카메라를 만든다
REAL_CAM = bool(args.camera_prim)
if REAL_CAM:
    CAM = args.camera_prim
    if not st.GetPrimAtPath(CAM).IsValid():
        say(f"카메라 prim 없음: {CAM}"); app.close(); sys.exit(1)
else:
    CAM = f"{R}/panda_hand/wrist_cam_probe"
    cam_prim = UsdGeom.Camera.Define(st, CAM)
    xf = UsdGeom.Xformable(cam_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*CAM_OFFSET)); xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
    cam_prim.CreateHorizontalApertureAttr(20.955)
    cam_prim.CreateVerticalApertureAttr(20.955 * H_PX / W_PX)
    cam_prim.CreateFocalLengthAttr(20.955 / (2 * math.tan(HFOV / 2)))
    cam_prim.CreateClippingRangeAttr(Gf.Vec2f(0.02, 50.0))

# 실제 카메라의 장착 위치·방향을 손 좌표계로 한 번 환산해 둔다 (이후 후보마다 FK 로 옮긴다)
CAM_REL_P = CAM_OFFSET.copy()
CAM_REL_R = np.eye(3)
if REAL_CAM:
    for _ in range(2):
        world.step(render=True)
    hp0, hq0 = SingleXFormPrim(f"{R}/panda_hand").get_world_pose()
    cp0, cq0 = SingleXFormPrim(CAM).get_world_pose()
    Rh0 = R_from_quat(np.asarray(hq0, float)); Rc0 = R_from_quat(np.asarray(cq0, float))
    CAM_REL_P = Rh0.T @ (np.asarray(cp0, float) - np.asarray(hp0, float))
    CAM_REL_R = Rh0.T @ Rc0
    say(f"실제 카메라 {CAM.rsplit('/', 1)[1]}: 손 기준 위치 {np.round(CAM_REL_P, 4).tolist()}")

links = [f"panda_link{i}" for i in range(1, 8)] + ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
colliders = []
for p in st.Traverse():
    path = str(p.GetPath())
    if p.HasAPI(UsdPhysics.CollisionAPI) and not path.startswith(R):
        colliders.append((path, scene.aabb(path)))
say(f"고정·물체 충돌체 {len(colliders)}개, 서가 앞면 y {scene.shelf_front_y:.3f}")


def overlaps(margin=0.01):
    hits = []
    for ln in links:
        lb = scene.aabb(f"{R}/{ln}")
        lb = lb + np.array([-margin] * 3 + [margin] * 3)
        for path, ob in colliders:
            if np.all(lb[:3] < ob[3:]) and np.all(ob[:3] < lb[3:]):
                hits.append((ln, path.split("/")[-1]))
    return hits


def set_arm(q):
    """순간이동 + 구동 목표도 같은 값으로 (목표가 옛 자세면 스텝 한 번에 끌려간다)"""
    from isaacsim.core.utils.types import ArticulationAction
    full = robot.get_joint_positions().copy(); full[scene.idx_arm] = q
    robot.set_joint_positions(full)
    robot.set_joint_velocities(np.zeros_like(full))
    robot.apply_action(ArticulationAction(joint_positions=full))


def project(points, cam_pos, cam_R):
    """cam_R 열: [x_cam(오른쪽), y_cam(아래), z_cam(광축)] (월드)"""
    uv = []
    for p in points:
        c = cam_R.T @ (np.asarray(p) - cam_pos)
        if c[2] <= 0.05:
            uv.append(None); continue
        uv.append((W_PX / 2 + FX * c[0] / c[2], H_PX / 2 + FX * c[1] / c[2]))
    return uv


# ------------------------------------------------------------------ 1. 후보 탐색 (순간이동, 물리 스텝 없음)
q_home = scene.q_home
cands = []
x_mid = float(np.mean([t[0] for t in targets]))
for dist in (0.22, 0.26, 0.30, 0.34):                  # 서가 앞면 ~ 카메라(손) 거리
    for z in (0.62, 0.70, 0.78, 0.86):                # 손 높이 (월드)
        for tilt_deg in (0, 15, 30):                  # 아래로 기울임
            t = math.radians(tilt_deg)
            approach = np.array([0.0, math.cos(t), -math.sin(t)])
            ori = scene.orientation(approach, [1, 0, 0])
            hand = np.array([x_mid, scene.shelf_front_y - dist, z])
            tcp = hand + 0.10 * approach              # right_gripper = hand + 0.10 접근축
            q, ok = scene.ik_joints(tcp, ori, q_home)
            rec = {"dist": dist, "z": z, "tilt": tilt_deg, "ik": ok}
            if not ok:
                cands.append(rec); continue
            set_arm(q)
            # 순간이동 직후 USD(SingleXFormPrim·AABB)는 옛 자세를 준다 (9/17 실측: 손 z 3.38 m).
            # 렌더 스텝으로 USD 를 갱신하고, 손 자세는 Lula FK(right_gripper)에서 계산한다
            for _ in range(2):
                world.step(render=True)
            ee_p, ee_R = scene.ik.compute_end_effector_pose()
            ee_p = np.asarray(ee_p, float); ee_R = np.asarray(ee_R, float)
            a_w = ee_R @ scene._a_loc                      # 접근축(월드)
            hand_p = ee_p - 0.10 * a_w                     # panda_hand = right_gripper − 0.10 접근축
            c_w = ee_R @ scene._c_loc                      # 손 y(닫힘축)
            Rh = np.stack([np.cross(c_w, a_w), c_w, a_w], axis=1)   # 손 좌표축 (x, y, z=접근)
            hp = hand_p
            cam_pos = hand_p + Rh @ CAM_REL_P
            if REAL_CAM:
                Rc = Rh @ CAM_REL_R            # USD 카메라는 −Z 를 본다 → 광축 = −Rc[:,2], 화면 아래 = −Rc[:,1]
                x_cam, y_cam, z_cam = Rc[:, 0], -Rc[:, 1], -Rc[:, 2]
            else:
                z_cam = a_w; x_cam = np.array([1.0, 0, 0]) - np.dot([1.0, 0, 0], z_cam) * z_cam
                x_cam /= np.linalg.norm(x_cam); y_cam = np.cross(z_cam, x_cam)
            pts = []
            for tg in targets:
                pts += [tg + [0, -scene.W / 2, -scene.L / 2], tg + [0, -scene.W / 2, scene.L / 2]]   # 책등 아래·위
            uv = project(pts, cam_pos, np.stack([x_cam, y_cam, z_cam], axis=1))
            inside = all(u is not None and 10 <= u[0] <= W_PX - 10 and 10 <= u[1] <= H_PX - 10 for u in uv)
            hits = overlaps()
            q_act = robot.get_joint_positions()[scene.idx_arm]
            usd_hand = np.asarray(SingleXFormPrim(f"{R}/panda_hand").get_world_pose()[0], float)
            rec.update({"hand": np.round(np.asarray(hp, float), 3).tolist(), "hand_z": np.round(Rh[:, 2], 3).tolist(),
                        "usd_hand_minus_fk": round(float(np.linalg.norm(usd_hand - hp)), 4),
                        "tcp_target": np.round(tcp, 3).tolist(), "tcp_actual": np.round(ee_p, 3).tolist(),
                        "teleport_err": round(float(np.max(np.abs(q_act - q))), 4), "approach": np.round(approach, 3).tolist()})
            rec.update({"q": np.round(q, 4).tolist(), "overlap": hits, "all_slots_in_view": bool(inside),
                        "joint_dist_from_home": round(float(np.max(np.abs(q - q_home))), 3),
                        "uv": [None if u is None else [round(u[0]), round(u[1])] for u in uv]})
            cands.append(rec)
set_arm(q_home); world.step(render=False)
for _ in range(60):
    arm.update(); world.step(render=False)

ok_c = [c for c in cands if c["ik"] and not c["overlap"] and c["all_slots_in_view"]]
say(f"후보 {len(cands)}개: IK {sum(c['ik'] for c in cands)} / 겹침 없음 {sum(1 for c in cands if c['ik'] and not c['overlap'])} / "
    f"4칸 모두 화면 안 {sum(1 for c in cands if c['ik'] and c['all_slots_in_view'])} / 모두 통과 {len(ok_c)}")
for c in cands:
    if c["ik"]:
        say(f"  d {c['dist']:.2f} z {c['z']:.2f} tilt {c['tilt']:2d}: 겹침 {len(c['overlap'])} 화면안 {c['all_slots_in_view']} "
            f"홈거리 {c['joint_dist_from_home']:.2f} USD-FK {c['usd_hand_minus_fk']:.3f}m uv0 {c['uv'][0]} {c['overlap'][:1]}")
if not ok_c:
    json.dump({"candidates": cands, "log": LOG}, open(f"{args.out}/result.json", "w"), indent=1, ensure_ascii=False)
    say("통과 후보 없음"); app.close(); sys.exit(0)

best = min(ok_c, key=lambda c: c["joint_dist_from_home"])
say(f"선택: d {best['dist']} z {best['z']} tilt {best['tilt']} q {best['q']}")

# ------------------------------------------------------------------ 2. 실제 이동 검증
cam = Camera(prim_path=CAM, resolution=(W_PX, H_PX)); cam.initialize()
say(f"손목 카메라 가로 화각 {math.degrees(cam.get_horizontal_fov()):.1f}°")
q_obs = np.array(best["q"])


def run(prims, label, capture=None):
    for p in prims:
        arm.enqueue(p)
    q_prev = robot.get_joint_positions()[scene.idx_arm].copy(); over = 0; peak = 0.0; hits = set(); steps = 0
    while not arm.idle and steps < 3000:
        s = arm.update(); world.step(render=capture is not None); steps += 1
        q_now = robot.get_joint_positions()[scene.idx_arm]
        r = np.abs(q_now - q_prev) / world.get_physics_dt() / VEL_LIMIT; q_prev = q_now.copy()
        peak = max(peak, float(r.max())); over += int(r.max() > 0.8)
        if steps % 10 == 0:
            hits.update(overlaps(margin=0.0))
        if s.name == "FAILED":
            say(f"{label} 실패 M{arm.error_code} {arm.error}"); break
    say(f"{label}: {steps} 스텝 ({steps / 60:.1f}s), 각속도 최대 {peak * 100:.0f}% 초과 {over}스텝, 이동 중 겹침 {sorted(hits) or '없음'}")
    return {"steps": steps, "peak": round(peak, 3), "over80": over, "overlap": sorted(hits)}


moves = {}
moves["home_to_observe"] = run([MoveJoint(q_obs, speed_scale=0.6, timeout_s=15), Wait(1.0)], "홈 → 서가 관측")
err = float(np.max(np.abs(robot.get_joint_positions()[scene.idx_arm] - q_obs)))
still = []
for i in range(60):
    q0 = robot.get_joint_positions()[scene.idx_arm].copy(); arm.update(); world.step(render=True)
    still.append(float(np.max(np.abs(robot.get_joint_positions()[scene.idx_arm] - q0))) * 60)
rgba = cam.get_rgba()
if rgba is not None and rgba.size:
    img = cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)
    for u in best["uv"]:
        cv2.circle(img, tuple(int(v) for v in u), 5, (0, 0, 255), -1)
    cv2.imwrite(f"{args.out}/wrist_observe_shelf.jpg", img)
say(f"관측 자세 도달 오차 {err:.4f} rad, 정지 1초 관절속도 최대 {max(still):.4f} rad/s")

moves["observe_to_home"] = run([MoveJoint(q_home, speed_scale=0.6, timeout_s=15), Wait(0.5)], "서가 관측 → 홈")
for _ in range(40):
    arm.update(); world.step(render=True)
rgba = cam.get_rgba()
if rgba is not None and rgba.size:
    cv2.imwrite(f"{args.out}/wrist_home_tray.jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))

json.dump({"assumption": {"camera": "panda_hand origin, optical axis = approach", "hfov_deg": 90.5, "res": [W_PX, H_PX]},
           "selected": best, "q_home": np.round(q_home, 4).tolist(), "reach_error_rad": err, "still_max_rad_s": max(still),
           "moves": moves, "candidates": cands, "log": LOG},
          open(f"{args.out}/result.json", "w"), indent=1, ensure_ascii=False, default=float)
app.close()
