"""트레이 주변(로봇 데크 위)에 무엇이 있는지 확인한다 — 학습 데이터의 라벨 대상 점검용."""
import os
import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402
usd = sys.argv[1] if len(sys.argv) > 1 else _paths.default_usd()
center = np.array([2.36, -2.94, 0.35])
open_stage(usd); app.update()
while is_stage_loading():
    app.update()
st = get_current_stage()
cache = create_bbox_cache()
rows = []
for p in st.Traverse():
    if not p.IsA(UsdGeom.Xformable) or p.GetPath().pathString.count("/") > 3:
        continue
    for c in p.GetChildren():
        try:
            cache.Clear()
            b = np.array(compute_aabb(cache, str(c.GetPath()), include_children=True), float)
        except Exception:
            continue
        if not np.all(np.isfinite(b)):
            continue
        mid = (b[:3] + b[3:]) / 2
        if np.linalg.norm(mid[:2] - center[:2]) < 0.6 and 0.2 < mid[2] < 0.8:
            rows.append((float(np.linalg.norm(mid[:2] - center[:2])), str(c.GetPath()),
                         np.round(mid, 3).tolist(), np.round(b[3:] - b[:3], 3).tolist()))
for d, path, mid, size in sorted(rows)[:40]:
    print(f"### {d:.2f}m {path} 중심 {mid} 크기 {size}", flush=True)
print(f"### 트레이 주변 prim {len(rows)}개", flush=True)
app.close()
