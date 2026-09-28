#!/usr/bin/env python3
"""레벨 USD 를 **자기완결 폴더**로 모은다 (GUI 의 File ▸ Collect As 와 같은 일).

    ~/isaacsim/python.sh simulation/isaac/tools/collect_level.py <레벨.usd> <나갈폴더>

왜 Isaac 안에서 해야 하나: 레벨의 로봇은 `https://omniverse-content-production…` 를 참조한다.
그 주소를 풀 수 있는 것은 **옴니버스 리졸버뿐**이고, 그건 Kit 이 떠 있을 때만 있다.
맨 python 에서 `UsdUtils.ComputeAllDependencies` 를 돌리면 그 셋이 "미해결" 로 나온다.

끝나면 같은 검사를 꾸러미에 다시 돌려 **미해결 0 · 원격 0 · 폴더 밖 0** 인지 찍는다.
"""
import argparse, asyncio, os, sys

ap = argparse.ArgumentParser()
ap.add_argument("usd")
ap.add_argument("out")
ap.add_argument("--renderer", default="RayTracedLighting")
args = ap.parse_args()

usd = os.path.abspath(os.path.expanduser(args.usd))
out = os.path.abspath(os.path.expanduser(args.out))
if not os.path.exists(usd):
    sys.exit(f"레벨이 없다: {usd}")
os.makedirs(out, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True, "renderer": args.renderer})

import omni.usd  # noqa: E402
from omni.kit.usd.collect import Collector  # noqa: E402
from pxr import UsdUtils  # noqa: E402

print(f"[모으기] {usd}\n[모으기] → {out}", flush=True)
ctx = omni.usd.get_context()
ok = ctx.open_stage(usd)
print(f"[모으기] 스테이지 열기: {ok}", flush=True)
for _ in range(120):
    app.update()

done = {"v": None}


def _progress(step, total):
    if total and step % max(1, total // 20) == 0:
        print(f"[모으기] {step}/{total}", flush=True)


async def _run():
    c = Collector(usd, out, usd_only=False, flat_collection=False,
                  material_only=False, skip_existing=False)
    done["v"] = await c.collect(_progress, None)

loop = asyncio.get_event_loop()
task = loop.create_task(_run())
while not task.done():
    app.update()
    loop.run_until_complete(asyncio.sleep(0.0))
print(f"[모으기] 끝: {done['v']}", flush=True)

# --- 꾸러미를 바로 검사한다
# collect() 는 (성공여부, 모은 루트 USD 경로) 를 준다
root = ""
if isinstance(done["v"], (tuple, list)) and len(done["v"]) > 1 and done["v"][1]:
    root = os.path.abspath(str(done["v"][1]))
if not root or not os.path.exists(root):
    root = os.path.join(out, os.path.basename(usd))
if not os.path.exists(root):
    cands = [f for f in os.listdir(out) if f.endswith((".usd", ".usdc", ".usda"))]
    root = os.path.join(out, cands[0]) if cands else ""
if root and os.path.exists(root):
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(root)
    remote = [a for a in assets if a.startswith(("http:", "https:", "omniverse:"))]
    outside = [a for a in assets if not os.path.abspath(a).startswith(out)]
    print(f"[검사] {root}")
    print(f"[검사] 레이어 {len(layers)} · 에셋 {len(assets)} · "
          f"미해결 {len(unresolved)} · 원격 {len(remote)} · 폴더 밖 {len(outside)}", flush=True)
    for x in list(unresolved)[:10]:
        print("   미해결:", x)
    for x in remote[:10]:
        print("   원격  :", x)
    for x in outside[:10]:
        print("   밖    :", x)
else:
    print("[검사] 꾸러미 안에서 루트 USD 를 못 찾았다 — 손으로 확인할 것", flush=True)

app.close()
