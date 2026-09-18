"""폴더 안 USD 들의 **절대경로 참조를 상대경로로** 바꿔 어느 PC 에서나 열리게 만든다.

왜: 결합 로봇 USD 는 팔을 `/home/rokey/carter_m0609_handoff/m0609/m0609_gripper.usd` 로,
    그 팔 USD 는 다시 `SubUSDs/...` 를 절대경로로 참조한다. 다른 PC 로 옮기면 그 경로가 없어
    **팔이 통째로 빠지거나 일부만 들어온다.** 폴더째 복사해도 경로가 다르면 마찬가지다.

무엇을 바꾸나: 같은 폴더(루트) 안을 가리키는 절대경로만 상대경로로 바꾼다.
             클라우드 URL(Nova Carter)이나 폴더 밖 경로는 건드리지 않는다.

실행
    ISAAC_ENTRY=simulation/isaac/tools/relativize_usd_refs.py ./scripts/run_isaac_tool.sh \\
        --root ~/carter_m0609_handoff
    # 확인만 하려면 --dry-run
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--root", required=True, help="USD 들이 들어 있는 폴더 (이 안이 상대경로 기준)")
ap.add_argument("--dry-run", action="store_true")
args = ap.parse_args()

# pxr 은 Isaac 런타임이 올라와야 import 된다 (헤드리스로 최소만 띄운다)
from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

from pxr import Sdf  # noqa: E402


def say(m):
    sys.stderr.write(f"### {m}\n")
    sys.stderr.flush()


root = os.path.realpath(os.path.expanduser(args.root))
layers = []
for base, _dirs, files in os.walk(root):
    for f in sorted(files):
        if f.endswith((".usd", ".usda", ".usdc")):
            layers.append(os.path.join(base, f))
say(f"대상 {len(layers)}개 (루트 {root})")

changed_total = 0
for path in layers:
    layer = Sdf.Layer.FindOrOpen(path)
    if layer is None:
        say(f"  열 수 없음: {path}")
        continue
    here = os.path.dirname(os.path.realpath(path))
    changed = []
    # GetCompositionAssetDependencies 는 reference·payload·sublayer 를 모두 준다
    for ref in list(layer.GetCompositionAssetDependencies()):
        if not ref.startswith("/"):
            continue                       # 이미 상대경로거나 URL
        target = os.path.realpath(ref)
        if not target.startswith(root + os.sep):
            say(f"  건너뜀(루트 밖): {os.path.basename(path)} → {ref}")
            continue
        rel = os.path.relpath(target, here)
        if args.dry_run:
            changed.append((ref, rel))
            continue
        # 참조·페이로드·서브레이어 전부에서 경로 문자열을 바꿔 준다
        if layer.UpdateExternalReference(ref, rel):
            changed.append((ref, rel))
    if changed:
        for old, new in changed:
            say(f"  {os.path.basename(path)}: {old} → {new}")
        if not args.dry_run:
            layer.Save()
        changed_total += len(changed)

say(f"{'바꿀 것' if args.dry_run else '바꾼 것'} {changed_total}곳")
app.close()
