#!/usr/bin/env python3
"""우리 레벨(로봇·트레이가 한 파일에 든 것)을 통합 브랜치용 **환경 전용** 파일로 만든다. 원본은 건드리지 않는다.

    python3 make_integration_env.py <우리 레벨.usdc> <통합 브랜치의 tray_set.usd> <낼 파일.usd>

통합 브랜치(integration/manipulation-standalone)는 로봇 · 트레이 · 환경을 세 파일로 겹쳐 쓴다. 그래서:
  1. /World/ridgeback_franka 를 뺀다          (franka_nav.usda 가 같은 경로에 얹는다)
  2. /World/tray_books 를 뺀다                (tray_set.usd 가 같은 경로에 얹는다)
  3. tray_set.usd 와 이름이 겹치는 머티리얼을 뺀다 — 환경에서 쓰는 곳이 없을 때만
  4. /World/env_Base/Cube_002 의 충돌체를 convexDecomposition → none 으로
     (통합 월드와 같은 값. 안 하면 로봇이 출발 자리에서 떨며 못 움직인다 — 2026-09-29 10.10.0.1 에서 확인)

낸 파일은 env_base_collider.usd · textures/ 와 같은 폴더에 둔다.

md5 는 저장 횟수에 따라 달라진다(usdc 는 같은 내용도 바이트가 다를 수 있다). 넘긴 파일(40dae43e…)과 같은지는
usda 로 풀어 비교한다 — 2026-09-29 에 이 도구로 다시 만들어 내용이 같음을 확인했다.
"""
import hashlib
import shutil
import sys

from pxr import Sdf, Usd, UsdShade


def main(src, tray_usd, dst):
    shutil.copy(src, dst)
    layer = Sdf.Layer.FindOrOpen(dst)

    def remove(path):
        spec = layer.GetPrimAtPath(path)
        if spec:
            del spec.nameParent.nameChildren[spec.name]
            print("뺌", path)

    remove("/World/ridgeback_franka")
    remove("/World/tray_books")

    tray = Sdf.Layer.FindOrOpen(tray_usd)
    names = [c.name for c in tray.GetPrimAtPath("/_materials").nameChildren]
    stage = Usd.Stage.Open(layer, load=Usd.Stage.LoadNone)
    used = set()
    for prim in stage.Traverse():
        bound = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath()
        if bound:
            used.add(str(bound))
        for rel in prim.GetRelationships():
            used.update(str(t) for t in rel.GetTargets())
    for name in names:
        path = "/_materials/" + name
        if any(u == path or u.startswith(path + "/") for u in used):
            print("쓰는 곳이 있어 남김", path)
        else:
            remove(path)

    shell = Sdf.CreatePrimInLayer(layer, "/World/env_Base/Cube_002")
    attr = shell.attributes.get("physics:approximation") or Sdf.AttributeSpec(
        shell, "physics:approximation", Sdf.ValueTypeNames.Token, Sdf.VariabilityUniform)
    attr.default = "none"
    print("충돌체", "/World/env_Base/Cube_002", "→ none")

    layer.Save()
    print("md5:", hashlib.md5(open(dst, "rb").read()).hexdigest())


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
