"""
Blender 批处理脚本：FBX → 上色 → 减面 → 导出 DAE + OBJ
用法：blender --background --python fbx_to_dae.py -- <input.fbx>

输出：
  - <name>_blender.dae  (Collada，带材质颜色)
  - <name>_blender.obj  (Wavefront OBJ，备用)
"""
import bpy
import sys
import os

filepath = sys.argv[-1]
output_dir = os.path.dirname(os.path.abspath(filepath))
basename = os.path.splitext(os.path.basename(filepath))[0]
output_dae = os.path.join(output_dir, f"{basename}_blender.dae")
output_obj = os.path.join(output_dir, f"{basename}_blender.obj")

# ── 导入 FBX ──
try:
    bpy.ops.import_scene.fbx(filepath=filepath, use_image_search=True, global_scale=1.0)
    print(f"Imported: {filepath}")
except Exception as e:
    print(f"Import error: {e}")
    sys.exit(1)

# ── 交通沙盘颜色规范 ──
color_map = {
    'road': (0.15, 0.15, 0.18, 1.0),
    'street': (0.15, 0.15, 0.18, 1.0),
    'asphalt': (0.15, 0.15, 0.18, 1.0),
    '马路': (0.15, 0.15, 0.18, 1.0),
    '柏油': (0.15, 0.15, 0.18, 1.0),
    '路面': (0.15, 0.15, 0.18, 1.0),
    '道路': (0.15, 0.15, 0.18, 1.0),

    'grass': (0.25, 0.55, 0.15, 1.0),
    'green': (0.25, 0.55, 0.15, 1.0),
    'vegetation': (0.25, 0.55, 0.15, 1.0),
    '草地': (0.25, 0.55, 0.15, 1.0),
    '绿化': (0.25, 0.55, 0.15, 1.0),
    '植被': (0.25, 0.55, 0.15, 1.0),
    '草坪': (0.25, 0.55, 0.15, 1.0),

    'building': (0.75, 0.75, 0.72, 1.0),
    '建筑': (0.75, 0.75, 0.72, 1.0),
    '楼板': (0.75, 0.75, 0.72, 1.0),
    '墙体': (0.7, 0.7, 0.68, 1.0),
    'wall': (0.7, 0.7, 0.68, 1.0),

    'glass': (0.6, 0.75, 0.85, 1.0),
    '玻璃': (0.6, 0.75, 0.85, 1.0),
    '透明': (0.6, 0.75, 0.85, 1.0),

    'marking': (1.0, 1.0, 1.0, 1.0),
    'white': (1.0, 1.0, 1.0, 1.0),
    'line': (1.0, 1.0, 1.0, 1.0),
    '标线': (1.0, 1.0, 1.0, 1.0),
    '白色': (1.0, 1.0, 1.0, 1.0),
    '车道': (1.0, 1.0, 1.0, 1.0),
    '箭头': (1.0, 1.0, 1.0, 1.0),
    '斑马': (1.0, 1.0, 1.0, 1.0),

    'yellow': (0.95, 0.85, 0.1, 1.0),
    '黄色': (0.95, 0.85, 0.1, 1.0),
    '中心线': (0.95, 0.85, 0.1, 1.0),

    'track': (0.75, 0.25, 0.15, 1.0),
    'stadium': (0.75, 0.25, 0.15, 1.0),
    '跑道': (0.75, 0.25, 0.15, 1.0),
    '操场': (0.75, 0.25, 0.15, 1.0),
    '运动': (0.75, 0.25, 0.15, 1.0),

    'field': (0.3, 0.65, 0.2, 1.0),
    '足球': (0.3, 0.65, 0.2, 1.0),
    '球场': (0.3, 0.65, 0.2, 1.0),

    'sign': (0.9, 0.85, 0.15, 1.0),
    '标志': (0.9, 0.85, 0.15, 1.0),
    '路牌': (0.9, 0.85, 0.15, 1.0),
    '交通': (0.9, 0.85, 0.15, 1.0),
    '警告': (0.9, 0.85, 0.15, 1.0),

    'prohibition': (0.9, 0.1, 0.1, 1.0),
    '禁令': (0.9, 0.1, 0.1, 1.0),
    '禁止': (0.9, 0.1, 0.1, 1.0),

    'info': (0.1, 0.4, 0.8, 1.0),
    '指示': (0.1, 0.4, 0.8, 1.0),

    'metal': (0.5, 0.5, 0.5, 1.0),
    '金属': (0.5, 0.5, 0.5, 1.0),
    '银灰': (0.5, 0.5, 0.5, 1.0),
    '路灯': (0.5, 0.5, 0.5, 1.0),
    '灯杆': (0.5, 0.5, 0.5, 1.0),

    'wood': (0.55, 0.35, 0.2, 1.0),
    '木质': (0.55, 0.35, 0.2, 1.0),
    '百叶': (0.55, 0.35, 0.2, 1.0),
    '木': (0.55, 0.35, 0.2, 1.0),

    'sidewalk': (0.65, 0.65, 0.63, 1.0),
    '人行': (0.65, 0.65, 0.63, 1.0),
    '步道': (0.65, 0.65, 0.63, 1.0),

    'curb': (0.8, 0.8, 0.78, 1.0),
    '路缘': (0.8, 0.8, 0.78, 1.0),
    '缘石': (0.8, 0.8, 0.78, 1.0),

    'parking': (0.25, 0.25, 0.27, 1.0),
    '停车': (0.25, 0.25, 0.27, 1.0),
    '车位': (0.25, 0.25, 0.27, 1.0),
}

def get_color_for_material(mat_name):
    name_lower = mat_name.lower()
    for key, color in color_map.items():
        if key in name_lower:
            return color
    return (0.7, 0.7, 0.7, 1.0)

# ── 材质上色 ──
print("\n=== 材质颜色分配 ===")
for mat in bpy.data.materials:
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()

    color = get_color_for_material(mat.name)
    print(f"  {mat.name} -> RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")

    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = 0.8

    output = nodes.new('ShaderNodeOutputMaterial')
    links = mat.node_tree.links
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

# ── 减面 ──
try:
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.context.object.modifiers['Decimate'].ratio = 0.1
    bpy.ops.object.modifier_apply(modifier='Decimate')
    print("\nDecimate applied (ratio=0.1)")
except Exception as e:
    print(f"Decimate warning: {e}")

# ── 导出 DAE ──
try:
    bpy.ops.wm.collada_export(
        filepath=output_dae,
        use_selection=True,
        apply_modifiers=True,
        include_texture=False,
    )
    print(f"\nExported DAE: {output_dae}")
    if os.path.exists(output_dae):
        size = os.path.getsize(output_dae)
        print(f"  Size: {size/1024/1024:.2f} MB")
    else:
        print("ERROR: DAE file not created!")
except Exception as e:
    print(f"DAE export error: {e}")

# ── 导出 OBJ（备用）──
try:
    bpy.ops.export_scene.obj(filepath=output_obj, use_selection=True, use_materials=True)
    print(f"Exported OBJ: {output_obj}")
    if os.path.exists(output_obj):
        size = os.path.getsize(output_obj)
        print(f"  Size: {size/1024/1024:.2f} MB")
except Exception as e:
    print(f"OBJ export warning: {e}")

print("\n=== 完成 ===")
