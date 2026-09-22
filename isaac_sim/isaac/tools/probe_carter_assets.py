"""Nova Carter · M0609 에셋이 어디에 있는지, 붙일 수 있는 구조인지 확인한다 (로봇 교체 준비).

확인하는 것
    1. Isaac 에셋 루트 (클라우드/로컬)에서 Nova Carter USD 경로
    2. GPU PC 에 있는 M0609(+RG2) USD 의 관절·링크 구조
    3. 두 로봇을 한 USD 에 붙일 때 필요한 것 (articulation root, 고정 조인트 자리)

실행
    ISAAC_ENTRY=isaac_sim/isaac/tools/probe_carter_assets.py ./scripts/run_isaac_tool.sh
"""
import os
import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


root = get_assets_root_path()
say(f"Isaac 에셋 루트: {root}")

CARTER_CANDIDATES = [
    "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    "/Isaac/Robots/Carter/nova_carter.usd",
    "/Isaac/Robots/Carter/carter_v2.usd",
    "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter_sensors.usd",
]
found_carter = None
for rel in CARTER_CANDIDATES:
    url = root + rel
    try:
        stage = Usd.Stage.Open(url)
    except Exception as e:                     # noqa: BLE001
        say(f"  열기 실패 {rel}: {str(e)[:60]}")
        continue
    if stage is None:
        say(f"  없음 {rel}")
        continue
    found_carter = url
    say(f"  **찾음** {rel}")
    arts = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    say(f"    articulation root {len(arts)}개: {[str(a.GetPath()) for a in arts][:2]}")
    say(f"    관절 {len(joints)}개: {[p.GetName() for p in joints][:8]}")
    links = [p for p in stage.Traverse() if p.GetTypeName() == 'Xform' and p.GetPath().pathString.count('/') == 3]
    say(f"    상위 링크: {[p.GetName() for p in links][:10]}")
    break

if not found_carter:
    say("Nova Carter USD 를 찾지 못했다 — 위 후보 경로를 확인할 것")

m0609 = os.path.expanduser("~/Desktop/Collected_m0609_gripper.usd")
if not os.path.exists(m0609):
    m0609 = os.path.expanduser("~/Isaac_Sim_b-1/src_pra/M0609/Collected_m0609_gripper/m0609_gripper.usd")
say(f"M0609 USD: {m0609} (있음={os.path.exists(m0609)})")
if os.path.exists(m0609):
    st = Usd.Stage.Open(m0609)
    arts = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = [(p.GetName(), p.GetTypeName()) for p in st.Traverse() if p.IsA(UsdPhysics.Joint)]
    say(f"  articulation root: {[str(a.GetPath()) for a in arts]}")
    say(f"  관절 {len(joints)}개")
    for n, t in joints[:14]:
        say(f"    {n} ({t})")
    default = st.GetDefaultPrim()
    say(f"  defaultPrim: {default.GetPath() if default else '없음'}")
    top = [p for p in st.Traverse() if p.GetPath().pathString.count('/') == 2]
    say(f"  상위 prim: {[p.GetName() for p in top][:10]}")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    if default:
        r = cache.ComputeWorldBound(default).ComputeAlignedRange()
        if not r.IsEmpty():
            say(f"  크기 {[round(v, 3) for v in r.GetSize()]}, 최소z {round(r.GetMin()[2], 3)}")

app.close()
