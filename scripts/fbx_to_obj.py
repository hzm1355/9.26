import bpy
import sys
import os

filepath = sys.argv[-1]
output_dir = os.path.dirname(filepath)
basename = os.path.splitext(os.path.basename(filepath))[0]
output_obj = os.path.join(output_dir, f"{basename}_blender.obj")

try:
    bpy.ops.import_scene.fbx(filepath=filepath, use_image_search=True, global_scale=1.0)
    print(f"Imported: {filepath}")
except Exception as e:
    print(f"Import error: {e}")
    sys.exit(1)

# 交通沙盘完整颜色规范
color_map = {
    # 道路系统 - 深灰黑色
    'road': (0.15, 0.15, 0.18, 1.0),
    'street': (0.15, 0.15, 0.18, 1.0),
    'asphalt': (0.15, 0.15, 0.18, 1.0),
    '马路': (0.15, 0.15, 0.18, 1.0),
    '柏油': (0.15, 0.15, 0.18, 1.0),
    '路面': (0.15, 0.15, 0.18, 1.0),
    '道路': (0.15, 0.15, 0.18, 1.0),
    
    # 草地/绿化 - 深绿色
    'grass': (0.25, 0.55, 0.15, 1.0),
    'green': (0.25, 0.55, 0.15, 1.0),
    'vegetation': (0.25, 0.55, 0.15, 1.0),
    '草地': (0.25, 0.55, 0.15, 1.0),
    '绿化': (0.25, 0.55, 0.15, 1.0),
    '植被': (0.25, 0.55, 0.15, 1.0),
    '草坪': (0.25, 0.55, 0.15, 1.0),
    
    # 建筑主体 - 浅灰白色
    'building': (0.75, 0.75, 0.72, 1.0),
    '建筑': (0.75, 0.75, 0.72, 1.0),
    '楼板': (0.75, 0.75, 0.72, 1.0),
    '墙体': (0.7, 0.7, 0.68, 1.0),
    'wall': (0.7, 0.7, 0.68, 1.0),
    
    # 玻璃 - 淡蓝色
    'glass': (0.6, 0.75, 0.85, 1.0),
    '玻璃': (0.6, 0.75, 0.85, 1.0),
    '透明': (0.6, 0.75, 0.85, 1.0),
    
    # 标线 - 纯白色
    'marking': (1.0, 1.0, 1.0, 1.0),
    'white': (1.0, 1.0, 1.0, 1.0),
    'line': (1.0, 1.0, 1.0, 1.0),
    '标线': (1.0, 1.0, 1.0, 1.0),
    '白色': (1.0, 1.0, 1.0, 1.0),
    '车道': (1.0, 1.0, 1.0, 1.0),
    '箭头': (1.0, 1.0, 1.0, 1.0),
    '斑马': (1.0, 1.0, 1.0, 1.0),
    
    # 黄色标线
    'yellow': (0.95, 0.85, 0.1, 1.0),
    '黄色': (0.95, 0.85, 0.1, 1.0),
    '中心线': (0.95, 0.85, 0.1, 1.0),
    
    # 跑道 - 红棕色
    'track': (0.75, 0.25, 0.15, 1.0),
    'stadium': (0.75, 0.25, 0.15, 1.0),
    '跑道': (0.75, 0.25, 0.15, 1.0),
    '操场': (0.75, 0.25, 0.15, 1.0),
    '运动': (0.75, 0.25, 0.15, 1.0),
    
    # 运动场绿地
    'field': (0.3, 0.65, 0.2, 1.0),
    '足球': (0.3, 0.65, 0.2, 1.0),
    '球场': (0.3, 0.65, 0.2, 1.0),
    
    # 交通标志 - 黄色
    'sign': (0.9, 0.85, 0.15, 1.0),
    '标志': (0.9, 0.85, 0.15, 1.0),
    '路牌': (0.9, 0.85, 0.15, 1.0),
    '交通': (0.9, 0.85, 0.15, 1.0),
    '警告': (0.9, 0.85, 0.15, 1.0),
    
    # 禁令标志 - 红色
    'prohibition': (0.9, 0.1, 0.1, 1.0),
    '禁令': (0.9, 0.1, 0.1, 1.0),
    '禁止': (0.9, 0.1, 0.1, 1.0),
    
    # 指示标志 - 蓝色
    'info': (0.1, 0.4, 0.8, 1.0),
    '指示': (0.1, 0.4, 0.8, 1.0),
    
    # 金属/路灯 - 中灰色
    'metal': (0.5, 0.5, 0.5, 1.0),
    '金属': (0.5, 0.5, 0.5, 1.0),
    '银灰': (0.5, 0.5, 0.5, 1.0),
    '路灯': (0.5, 0.5, 0.5, 1.0),
    '灯杆': (0.5, 0.5, 0.5, 1.0),
    
    # 木质 - 棕色
    'wood': (0.55, 0.35, 0.2, 1.0),
    '木质': (0.55, 0.35, 0.2, 1.0),
    '百叶': (0.55, 0.35, 0.2, 1.0),
    '木': (0.55, 0.35, 0.2, 1.0),
    
    # 人行道 - 浅灰色
    'sidewalk': (0.65, 0.65, 0.63, 1.0),
    '人行': (0.65, 0.65, 0.63, 1.0),
    '步道': (0.65, 0.65, 0.63, 1.0),
    
    # 路缘石 - 近白色
    'curb': (0.8, 0.8, 0.78, 1.0),
    '路缘': (0.8, 0.8, 0.78, 1.0),
    '缘石': (0.8, 0.8, 0.78, 1.0),
    
    # 停车场 - 深灰色
    'parking': (0.25, 0.25, 0.27, 1.0),
    '停车': (0.25, 0.25, 0.27, 1.0),
    '车位': (0.25, 0.25, 0.27, 1.0),
}

def get_color_for_material(mat_name):
    name_lower = mat_name.lower()
    for key, color in color_map.items():
        if key in name_lower:
            return color
    # 默认颜色：浅灰色
    return (0.7, 0.7, 0.7, 1.0)

# 为每个材质分配颜色
print("\n=== 材质颜色分配 ===")
for mat in bpy.data.materials:
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()

    color = get_color_for_material(mat.name)
    print(f"{mat.name} -> RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")

    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = 0.8

    output = nodes.new('ShaderNodeOutputMaterial')
    links = mat.node_tree.links
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

# 减面
try:
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.context.object.modifiers['Decimate'].ratio = 0.1
    bpy.ops.object.modifier_apply(modifier='Decimate')
    print("\nDecimate applied (ratio=0.1)")
except Exception as e:
    print(f"Decimate warning: {e}")

# 导出 OBJ
try:
    bpy.ops.export_scene.obj(filepath=output_obj, use_selection=True, use_materials=True)
    print(f"\nExported: {output_obj}")
    if os.path.exists(output_obj):
        size = os.path.getsize(output_obj)
        print(f"File size: {size/1024/1024:.2f} MB")
    else:
        print("ERROR: Export file not created!")
except Exception as e:
    print(f"Export error: {e}")
    sys.exit(1)

print("\n=== 完成 ===")
