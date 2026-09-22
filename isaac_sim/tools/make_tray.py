"""책 트레이를 치수 매개변수로 생성해 USD 로 내보낸다 (Blender 5.2, GPU PC 에서 실행).

사용자가 만든 book_tray.usdc 의 디자인(바닥판 + 좌우 두 줄 칸막이, 둥근 윗면)을 유지하고
치수만 실측 근거로 정한다.

치수 근거 (Isaac Sim 5.1.0 실측, 2026-09-17)
- Franka 손가락 두께(벌림축) 2.64cm. 책 두께 3.5cm 를 여유 1cm 로 잡으면 손가락쌍 바깥폭 9.8cm
- 옆 칸 책과 5mm 여유 → 칸 간격(pitch) ≥ 7.2cm
- 손은 책 가운데(길이 방향)를 잡고 칸막이는 책 양끝에만 있으므로 칸막이 높이는 손과 간섭하지 않는다

    blender -b --factory-startup -P make_tray.py -- --out tray.usdc [--pitch 0.075 ...]
"""
import argparse, sys, math
import bpy, bmesh

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--slots", type=int, default=6)
ap.add_argument("--pitch", type=float, default=0.075, help="칸 간격 (칸막이 중심 사이)")
ap.add_argument("--div-thick", type=float, default=0.01)
ap.add_argument("--div-height", type=float, default=0.10, help="바닥판 윗면에서 칸막이 꼭대기까지")
ap.add_argument("--row-inner", type=float, default=0.08, help="트레이 중심에서 칸막이 줄 안쪽 끝까지")
ap.add_argument("--row-len", type=float, default=0.10, help="칸막이 한 줄의 길이(책 길이 방향)")
ap.add_argument("--floor", type=float, default=0.02, help="바닥판 두께")
ap.add_argument("--margin", type=float, default=0.01, help="바깥 칸막이 바깥쪽 바닥판 여유")
ap.add_argument("--round", type=float, default=0.012, help="칸막이 윗모서리 둥글림 반경")
args = ap.parse_args(argv)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

# 좌표: 칸 간격 방향 = X (손가락이 닫히는 방향과 맞춘다), 책 길이 방향 = Y, 위 = Z. 원점은 바닥판 아랫면 중심
n_div = args.slots + 1
span_x = (n_div - 1) * args.pitch + args.div_thick + 2 * args.margin
span_y = 2 * (args.row_inner + args.row_len) + 2 * args.margin
mat = bpy.data.materials.new("tray_color")
mat.use_nodes = True
bsdf = mat.node_tree.nodes.get("Principled BSDF")
bsdf.inputs["Base Color"].default_value = (0.55, 0.55, 0.57, 1.0)
bsdf.inputs["Roughness"].default_value = 0.6

def box(name, cx, cy, cz, sx, sy, sz, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(cx, cy, cz))
    o = bpy.context.active_object; o.name = name
    o.scale = (sx, sy, sz)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0:
        # 윗모서리만 둥글린다 (사용자 디자인). 칸막이 두께보다 크게 잡으면 형상이 깨지므로 제한
        me = o.data; bm = bmesh.new(); bm.from_mesh(me)
        top_edges = [e for e in bm.edges if all(v.co.z > sz / 2 - 1e-6 for v in e.verts)
                     and abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6]
        bmesh.ops.bevel(bm, geom=top_edges, offset=min(bevel, sx * 0.49, sy * 0.49), segments=6, affect="EDGES")
        bm.to_mesh(me); bm.free()
    o.data.materials.append(mat)
    return o

parts = [box("floor", 0, 0, args.floor / 2, span_x, span_y, args.floor)]
x0 = -(n_div - 1) * args.pitch / 2
for i in range(n_div):
    x = x0 + i * args.pitch
    for side in (-1, 1):
        cy = side * (args.row_inner + args.row_len / 2)
        # 칸막이 윗모서리를 책 길이 방향(Y) 으로 둥글린다
        parts.append(box(f"div_{i}_{'L' if side < 0 else 'R'}", x, cy, args.floor + args.div_height / 2,
                         args.div_thick, args.row_len, args.div_height, bevel=args.round))

bpy.ops.object.select_all(action="DESELECT")
for o in parts: o.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
tray = bpy.context.active_object; tray.name = "book_tray"
bpy.ops.object.shade_smooth()

# 칸 중심 좌표를 커스텀 속성으로 남긴다 (Isaac 스크립트가 읽는다)
slot_x = [x0 + (i + 0.5) * args.pitch for i in range(args.slots)]
# 배열 속성은 USD 로 내보내지지 않아(실측) 숫자로 남긴다. 칸 i 중심 x = (i + 0.5 - slots/2) * pitch
tray["tray_pitch"] = args.pitch
tray["tray_slots"] = args.slots
tray["slot_gap"] = args.pitch - args.div_thick
tray["floor_top_z"] = args.floor

bpy.ops.wm.usd_export(filepath=args.out, selected_objects_only=False, export_materials=True,
                      export_lights=False, export_cameras=False, convert_scene_units="METERS",
                      export_custom_properties=True, root_prim_path="/root")
print(f"TRAY size x {span_x:.3f} y {span_y:.3f} z {args.floor + args.div_height:.3f}  slots {args.slots} "
      f"pitch {args.pitch} gap {args.pitch - args.div_thick:.3f}  slot_x {[round(v,4) for v in slot_x]}")
