"""로봇을 **실제 서가 앞**에 세운다.

왜 (2026-09-20):
    M0609 결합체로 파지는 되는데 삽입이 매번 빗나갔다. 원인은 팔이 아니라 배치였다 —
    로봇이 월드 x≈-5.77 에 서 있었고 서가는 x -2.97~+3.28 에 있다. 팔은 계약 좌표까지
    책을 정확히 옮긴 뒤 **허공에 놓고** 있었다. 좌표가 아니라 **서가가 없었다.**

무엇을 하나
    서가·받침판·팔 베이스를 **전부 실측**해서 로봇 루트 translate 를 옮긴다.
      y  받침판의 서가쪽 모서리가 서가 앞면에서 `--clearance` 만큼 떨어지게
         (Franka 에서 검증된 값 0.048 m. 받침판은 이 여유에 맞춰 이미 잘라 두었다)
      x  칸 x 중앙이 서가 폭 중앙에 오게
      z  건드리지 않는다 (바퀴가 바닥에 닿아 있어야 한다)

    서가의 **열린 면**은 가정하지 않고 광선으로 찾는다. 뒤판 쪽에 세우면 아무것도 안 들어간다.

자기검증 (웹 클로드 v19 회신 §1-2)
    ① 왕복 자기일치 — 옮긴 뒤 다시 재서 목표와 맞는지
    ② 오답 주입 — 여유를 음수로 주면 겹침을 잡아내는지
    ③ 0 가정 깨기 — 로봇도 서가도 원점이 아니다

실행
    ARM_ROBOT=m0609 ISAAC_ENTRY=isaac_sim/isaac/tools/place_robot_at_shelf.py \\
        ./scripts/run_isaac_tool.sh --usd <레벨>.usd --out <출력>.usd \\
        --shelf /World/.../shelf_brown__book_shelf_01 --slot-center-x -0.3497
"""
import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", default="", help="비우면 저장하지 않고 계산만 한다")
ap.add_argument("--shelf", required=True, help="서가 prim 경로")
ap.add_argument("--clearance", type=float, default=0.048,
                help="받침판 모서리와 서가 앞면 사이 여유 (m). Franka 에서 검증된 값")
ap.add_argument("--slot-center-x", type=float, default=-0.3497,
                help="서가 칸 x 들의 중앙 (팔 기준). 이 점이 서가 폭 중앙에 오게 맞춘다")
ap.add_argument("--row-z", type=float, default=0.0,
                help="0 이 아니면 그 선반판 윗면(월드 z)에 대한 팔 기준 삽입 z 를 같이 계산한다")
args = ap.parse_args()

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac"))
sys.path.insert(0, os.path.join(REPO, "simulation", "isaac", "config"))

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402

from robot_profiles import profile  # noqa: E402

BOT = profile()


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


open_stage(os.path.expanduser(args.usd))
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()


def aabb(path):
    cache.Clear()
    return np.array(compute_aabb(cache, path, include_children=True), float)


def base_pose():
    p = stage.GetPrimAtPath(f"{BOT.root}/{BOT.base_link}")
    if not p.IsValid():
        say(f"팔 베이스를 못 찾음: {BOT.root}/{BOT.base_link}")
        app.close()
        sys.exit(1)
    m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
    R = np.array([[m[0][0], m[1][0], m[2][0]],
                  [m[0][1], m[1][1], m[2][1]],
                  [m[0][2], m[1][2], m[2][2]]], float)
    return np.array(m.ExtractTranslation(), float), math.atan2(float(R[1, 0]), float(R[0, 0]))


shelf = aabb(args.shelf)
say(f"서가 {args.shelf}")
say(f"  x {shelf[0]:+.3f}~{shelf[3]:+.3f}  y {shelf[1]:+.3f}~{shelf[4]:+.3f}  z {shelf[2]:+.3f}~{shelf[5]:+.3f}")

P0, yaw0 = base_pose()
say(f"팔 베이스 {np.round(P0, 3).tolist()}  yaw {math.degrees(yaw0):+.1f}°")
if abs(math.degrees(yaw0)) > 1.0:
    say("**팔 yaw 가 0 이 아니다** — 경로 계산이 월드 축을 쓰므로 set_robot_yaw.py 를 먼저 돌릴 것")
    app.close()
    sys.exit(1)

plate = aabb(f"{BOT.root}/Cube")
say(f"받침판  x {plate[0]:+.3f}~{plate[3]:+.3f}  y {plate[1]:+.3f}~{plate[4]:+.3f}  윗면 z {plate[5]:+.3f}")
say(f"  팔 기준 모서리  -y {plate[1]-P0[1]:+.3f}  +y {plate[4]-P0[1]:+.3f}  "
    f"-x {plate[0]-P0[0]:+.3f}  +x {plate[3]-P0[0]:+.3f}")

# --- 열린 면 찾기 (가정하지 않는다) -------------------------------------------
# 서가 **바깥에서 안으로** 쏜다. 안에서 쏘면 뒤판 뒷면을 맞히게 되어 양쪽 다 뚫린 것처럼 나온다.
# 열린 쪽으로 쏜 광선은 서가를 가로질러 뒤판까지 들어가고, 뒤판 쪽으로 쏜 광선은 즉시 멈춘다.
World(stage_units_in_meters=1.0).reset()
from omni.physx import get_physx_scene_query_interface  # noqa: E402
sq = get_physx_scene_query_interface()
cx = float((shelf[0] + shelf[3]) / 2)
# 선반판 사이의 빈 높이에서 본다 (판 위 책에 막히지 않게 판 바로 아래)
probe_z = float(args.row_z - 0.06) if args.row_z else float((shelf[2] + shelf[5]) / 2)
depth = float(shelf[4] - shelf[1])
pen = {}
for name, face, d in (("-y", float(shelf[1]), +1.0), ("+y", float(shelf[4]), -1.0)):
    h = sq.raycast_closest([cx, face - d * 0.30, probe_z], [0.0, d, 0.0], 0.90)
    pen[name] = abs(float(h["position"][1]) - face) if h.get("hit") else depth
    say(f"  {name} 바깥에서 안으로: {'벽까지 ' + format(pen[name], '.3f') + ' m' if h.get('hit') else '끝까지 뚫림'}")
# 이 레벨의 서가는 **뒤판이 없다** — 양쪽이 똑같이 뚫린다. 그러면 어느 쪽이든 꽂히므로
# 팔 +y 가 서가를 향하는 -y 쪽을 쓴다. 대신 그쪽에 통로가 있는 서가를 골라야 한다.
if abs(pen["-y"] - pen["+y"]) < 0.02:
    say("  양쪽이 같다 — **뒤판 없는 개방형 서가**. 팔 +y 가 향하는 -y 면을 쓴다")
    near_is_min = True
else:
    near_is_min = pen["-y"] > pen["+y"]
face_y = float(shelf[1]) if near_is_min else float(shelf[4])
say(f"**열린 면 = y {'최소' if near_is_min else '최대'} 쪽 (y {face_y:+.3f})** — 로봇은 그 바깥에 선다")

if not near_is_min:
    say("**팔 +y 가 서가를 향하지 않는다** — 삽입 방향(+Y)과 어긋난다. 서가 반대편을 쓰거나 yaw 를 180° 돌릴 것")
    app.close()
    sys.exit(1)

# 물리를 돌리면 로봇이 미세하게 주저앉아 측정이 흔들린다 (yaw 0.6°, 10 mm 를 실제로 겪었다).
# 광선은 여기까지만 쓰고, **물리가 안 붙은 새 스테이지**에서 재고 옮긴다.
open_stage(os.path.expanduser(args.usd))
app.update()
while is_stage_loading():
    app.update()
stage = get_current_stage()
cache = create_bbox_cache()
P0, yaw0 = base_pose()
plate = aabb(f"{BOT.root}/Cube")

# --- 목표 위치 계산 -----------------------------------------------------------
edge_rel_y = float(plate[4] - P0[1])          # 받침판의 서가쪽(+y) 모서리, 팔 기준
want_base_y = face_y - args.clearance - edge_rel_y
want_base_x = float((shelf[0] + shelf[3]) / 2) - args.slot_center_x
delta = np.array([want_base_x - P0[0], want_base_y - P0[1], 0.0])
say("")
say(f"목표 팔 베이스 x {want_base_x:+.3f}  y {want_base_y:+.3f}  (z 는 그대로 {P0[2]:+.3f})")
say(f"이동량 {np.round(delta, 3).tolist()}")

say("")
say("=== 자기검증 ===")
say(f"③ 0 가정 깨기: 로봇 {np.linalg.norm(P0[:2]):.2f} m, 서가 {np.linalg.norm([cx, (shelf[1]+shelf[4])/2]):.2f} m — 둘 다 원점이 아니다")
if args.clearance < 0:
    say("② 오답 주입 생략 (여유가 이미 음수)")
else:
    bad = face_y - (-0.05) - edge_rel_y      # 여유 -5 cm → 받침판이 서가를 5 cm 파고든다
    ov = (bad + edge_rel_y) - face_y
    say(f"② 오답 주입: 여유 -0.05 면 받침판이 서가를 {ov*100:+.1f} cm 파고든다 "
        f"{'OK (겹침을 잡아낸다)' if ov > 0 else '**실패**'}")

# --- 적용 --------------------------------------------------------------------
root = stage.GetPrimAtPath(BOT.root)
xf = UsdGeom.Xformable(root)
tr_op = next((o for o in xf.GetOrderedXformOps() if "translate" in o.GetOpName()), None)
if tr_op is None:
    say("로봇 루트에 translate op 가 없다")
    app.close()
    sys.exit(1)
tr_op.Set(Gf.Vec3d(*[float(v) for v in (np.array(tr_op.Get(), float) + delta)]))
# app.update() 를 부르지 않는다 — 물리가 한 스텝 돌아 로봇이 주저앉는다. USD 변환은 즉시 반영된다

P1, yaw1 = base_pose()
plate1 = aabb(f"{BOT.root}/Cube")
err = float(np.linalg.norm(P1[:2] - np.array([want_base_x, want_base_y])))
gap = face_y - float(plate1[4])
say(f"① 왕복 자기일치: 옮긴 뒤 팔 베이스 {np.round(P1, 3).tolist()} 오차 {err*1000:.1f} mm "
    f"{'OK' if err < 1e-3 else '**실패**'}")
say(f"   받침판 서가쪽 모서리 y {plate1[4]:+.3f}, 서가 앞면 {face_y:+.3f} → 여유 {gap*100:+.1f} cm")
if gap < 0:
    say("   **받침판이 서가와 겹친다** — clearance 를 키울 것")

say("")
say("=== 결과 ===")
say(f"팔 베이스 월드 {np.round(P1, 3).tolist()}  yaw {math.degrees(yaw1):+.1f}°")
if args.row_z:
    bh = yaml.safe_load(open(os.path.join(
        REPO, "ros2_ws", "src", "shelving_manipulation",
        "config", "book_profiles.yaml")))["profiles"]["default"]["height"]
    say(f"선반판 윗면 {args.row_z:+.3f} → **팔 기준 삽입 z {args.row_z + bh/2 - P1[2]:+.4f}** "
        f"(책 높이 {bh:.4f} 의 절반을 더한다)")
say(f"칸 x 중앙 {args.slot_center_x:+.4f} → 월드 x {P1[0] + args.slot_center_x:+.3f} "
    f"(서가 폭 {shelf[0]:+.3f}~{shelf[3]:+.3f})")
say(f"칸 y {0.5495:+.4f} → 월드 y {P1[1] + 0.5495:+.3f} "
    f"(서가 깊이 {shelf[1]:+.3f}~{shelf[4]:+.3f}, 앞면에서 {abs(P1[1] + 0.5495 - face_y)*100:.1f} cm 안쪽)")

if args.out:
    out = os.path.expanduser(args.out)
    stage.Export(out)
    say(f"저장 {out}")
else:
    say("(--out 이 없어 저장하지 않았다)")
app.close()
