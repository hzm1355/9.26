"""
Blender FBX → OBJ 转换脚本（面向 Gazebo Classic 11）

用法（在 WSL / Blender 中）：
    blender --background --python fbx_to_gazebo.py -- <in.fbx> <out.obj> [target_faces]

关键设计：
  - 遍历场景中【所有】网格对象逐个减面（旧脚本只对 context.object 生效，
    多对象 FBX 几乎不减面，导致 OBJ 高达数百 MB，Gazebo 加载卡死）。
  - 按【目标总面数】统一计算每个对象的 decimate ratio，精确控制输出体积。
  - 依据材质名做交通沙盘配色（复用既有 color_map）。
  - 导出前对每个对象应用变换，保证 OBJ 世界坐标正确。

target_faces 默认 500000（约 25~30MB OBJ），Gazebo 加载流畅。
"""
import bpy
import sys
import os


def patch_colorspace_fallback():
    """WSL Blender 3.0.1 缺少 OCIO config，色彩空间只有 Linear/sRGB。
    3dsMax/Corona 导出的 FBX 带 alpha 贴图，导入器给贴图设 'Non-Color'
    会抛 TypeError 并中断整个导入。

    本流程按材质名重建纯色、不使用任何贴图（assign_colors 会 nodes.clear()
    并只保留 Principled BSDF），因此跳过贴图的色彩空间赋值是安全的。
    """
    try:
        import bpy_extras.node_shader_utils as nsu
        Wrapper = nsu.ShaderImageTextureWrapper
        orig_fget = Wrapper.image.fget
        orig_fset = Wrapper.image.fset

        def safe_fset(self, image):
            saved = self.colorspace_name
            self.colorspace_name = ...  # 令 image_set 跳过 Non-Color 赋值分支
            try:
                orig_fset(self, image)
            finally:
                self.colorspace_name = saved

        Wrapper.image = property(orig_fget, safe_fset)
        print("[patch] 色彩空间 fallback 补丁已应用（跳过贴图 Non-Color 赋值）")
    except Exception as e:
        print(f"[patch] 警告: 补丁未应用 ({e})")


def parse_args():
    argv = sys.argv
    if '--' in argv:
        argv = argv[argv.index('--') + 1:]
    else:
        argv = []
    if len(argv) < 2:
        print("用法: blender --background --python fbx_to_gazebo.py -- <in.fbx> <out.obj> [target_faces]")
        sys.exit(1)
    in_fbx = argv[0]
    out_obj = argv[1]
    target_faces = int(argv[2]) if len(argv) > 2 else 500000
    return in_fbx, out_obj, target_faces


# 交通沙盘完整颜色规范
color_map = {
    'road': (0.15, 0.15, 0.18, 1.0), 'street': (0.15, 0.15, 0.18, 1.0),
    'asphalt': (0.15, 0.15, 0.18, 1.0), '马路': (0.15, 0.15, 0.18, 1.0),
    '柏油': (0.15, 0.15, 0.18, 1.0), '路面': (0.15, 0.15, 0.18, 1.0),
    '道路': (0.15, 0.15, 0.18, 1.0),
    'grass': (0.25, 0.55, 0.15, 1.0), 'green': (0.25, 0.55, 0.15, 1.0),
    'vegetation': (0.25, 0.55, 0.15, 1.0), '草地': (0.25, 0.55, 0.15, 1.0),
    '绿化': (0.25, 0.55, 0.15, 1.0), '植被': (0.25, 0.55, 0.15, 1.0),
    '草坪': (0.25, 0.55, 0.15, 1.0),
    'building': (0.75, 0.75, 0.72, 1.0), '建筑': (0.75, 0.75, 0.72, 1.0),
    '楼板': (0.75, 0.75, 0.72, 1.0), '墙体': (0.7, 0.7, 0.68, 1.0),
    'wall': (0.7, 0.7, 0.68, 1.0),
    'glass': (0.6, 0.75, 0.85, 1.0), '玻璃': (0.6, 0.75, 0.85, 1.0),
    '透明': (0.6, 0.75, 0.85, 1.0),
    'marking': (1.0, 1.0, 1.0, 1.0), 'white': (1.0, 1.0, 1.0, 1.0),
    'line': (1.0, 1.0, 1.0, 1.0), '标线': (1.0, 1.0, 1.0, 1.0),
    '白色': (1.0, 1.0, 1.0, 1.0), '车道': (1.0, 1.0, 1.0, 1.0),
    '箭头': (1.0, 1.0, 1.0, 1.0), '斑马': (1.0, 1.0, 1.0, 1.0),
    'yellow': (0.95, 0.85, 0.1, 1.0), '黄色': (0.95, 0.85, 0.1, 1.0),
    '中心线': (0.95, 0.85, 0.1, 1.0),
    'track': (0.75, 0.25, 0.15, 1.0), 'stadium': (0.75, 0.25, 0.15, 1.0),
    '跑道': (0.75, 0.25, 0.15, 1.0), '操场': (0.75, 0.25, 0.15, 1.0),
    '运动': (0.75, 0.25, 0.15, 1.0),
    'field': (0.3, 0.65, 0.2, 1.0), '足球': (0.3, 0.65, 0.2, 1.0),
    '球场': (0.3, 0.65, 0.2, 1.0),
    'sign': (0.9, 0.85, 0.15, 1.0), '标志': (0.9, 0.85, 0.15, 1.0),
    '路牌': (0.9, 0.85, 0.15, 1.0), '交通': (0.9, 0.85, 0.15, 1.0),
    '警告': (0.9, 0.85, 0.15, 1.0),
    'prohibition': (0.9, 0.1, 0.1, 1.0), '禁令': (0.9, 0.1, 0.1, 1.0),
    '禁止': (0.9, 0.1, 0.1, 1.0),
    'info': (0.1, 0.4, 0.8, 1.0), '指示': (0.1, 0.4, 0.8, 1.0),
    'metal': (0.5, 0.5, 0.5, 1.0), '金属': (0.5, 0.5, 0.5, 1.0),
    '银灰': (0.5, 0.5, 0.5, 1.0), '路灯': (0.5, 0.5, 0.5, 1.0),
    '灯杆': (0.5, 0.5, 0.5, 1.0),
    'wood': (0.55, 0.35, 0.2, 1.0), '木质': (0.55, 0.35, 0.2, 1.0),
    '百叶': (0.55, 0.35, 0.2, 1.0), '木': (0.55, 0.35, 0.2, 1.0),
    'sidewalk': (0.65, 0.65, 0.63, 1.0), '人行': (0.65, 0.65, 0.63, 1.0),
    '步道': (0.65, 0.65, 0.63, 1.0),
    'curb': (0.8, 0.8, 0.78, 1.0), '路缘': (0.8, 0.8, 0.78, 1.0),
    '缘石': (0.8, 0.8, 0.78, 1.0),
    'parking': (0.25, 0.25, 0.27, 1.0), '停车': (0.25, 0.25, 0.27, 1.0),
    '车位': (0.25, 0.25, 0.27, 1.0),
}


def get_color_for_material(mat_name):
    name_lower = mat_name.lower()
    for key, color in color_map.items():
        if key in name_lower:
            return color
    return (0.7, 0.7, 0.7, 1.0)


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in list(block):
            try:
                block.remove(item)
            except Exception:
                pass


def assign_colors():
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
        mat.node_tree.links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])


def join_and_decimate(target_faces):
    """把所有网格对象 join 成单个网格，再一次性精确减面到目标面数。

    旧实现对每个对象单独 modifier_apply：上千对象时极慢（~27min），
    且因 ratio 下限/小对象跳过导致总面数失控。join 后只需一次 apply，
    面数 = ratio * 总面数，可精确命中目标。
    join 会把各对象的世界变换烘焙进活动对象的局部空间，故 join 后
    再 transform_apply 一次即可得到正确世界坐标。
    """
    mesh_objs = [o for o in bpy.data.objects
                 if o.type == 'MESH' and o.data is not None and len(o.data.polygons) > 0]
    if not mesh_objs:
        print("警告: 场景中没有有效网格对象")
        return

    total_faces = sum(len(o.data.polygons) for o in mesh_objs)
    print(f"\n=== 减面 ===")
    print(f"网格对象数: {len(mesh_objs)}  原始总面数: {total_faces}")

    # join 成单个对象
    bpy.ops.object.select_all(action='DESELECT')
    for o in mesh_objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objs[0]
    if len(mesh_objs) > 1:
        bpy.ops.object.join()
        print("已 join 为单个网格")

    joined = bpy.context.view_layer.objects.active

    # join 后统一应用世界变换
    try:
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    except Exception as e:
        print(f"  transform_apply 警告: {e}")

    cur_faces = len(joined.data.polygons)
    if cur_faces <= target_faces:
        print(f"当前面数 {cur_faces} 已 <= 目标 {target_faces}，跳过减面")
        return

    ratio = max(0.001, target_faces / cur_faces)
    print(f"目标面数: {target_faces}  ratio: {ratio:.4f}")
    try:
        mod = joined.modifiers.new(name='Decimate', type='DECIMATE')
        mod.ratio = ratio
        bpy.ops.object.modifier_apply(modifier=mod.name)
    except Exception as e:
        print(f"  减面失败: {e}")
        return

    print(f"减面后面数: {len(joined.data.polygons)}")



def main():
    in_fbx, out_obj, target_faces = parse_args()

    if not os.path.exists(in_fbx):
        print(f"错误: 输入文件不存在 {in_fbx}")
        sys.exit(1)

    patch_colorspace_fallback()
    clear_scene()

    print(f"导入 FBX: {in_fbx}")
    try:
        bpy.ops.import_scene.fbx(filepath=in_fbx, use_image_search=True, global_scale=1.0)
    except Exception as e:
        print(f"导入错误: {e}")
        sys.exit(1)

    assign_colors()
    join_and_decimate(target_faces)

    bpy.ops.object.select_all(action='SELECT')

    os.makedirs(os.path.dirname(out_obj), exist_ok=True)
    print(f"\n导出 OBJ: {out_obj}")
    try:
        bpy.ops.export_scene.obj(
            filepath=out_obj, use_selection=True, use_materials=True,
            use_normals=True, use_triangles=False, path_mode='COPY')
    except Exception as e:
        print(f"导出错误: {e}")
        sys.exit(1)

    if os.path.exists(out_obj):
        size_mb = os.path.getsize(out_obj) / 1024 / 1024
        print(f"OBJ 大小: {size_mb:.2f} MB")
        if size_mb > 50:
            print(f"警告: 仍超过 50MB，建议降低 target_faces 重新运行")
    else:
        print("错误: OBJ 未生成")
        sys.exit(1)

    print("\n=== 完成 ===")


main()
