#!/usr/bin/env python3
"""
Blender 减面脚本 - 将 OBJ 模型减小到目标大小以下
使用 Decimate modifier 减少面数，处理所有网格对象
"""
import bpy
import os
import sys

def reduce_mesh(input_obj, output_obj, target_size_mb=45):
    """
    减少 OBJ 模型的面数直到文件大小小于目标值
    """
    # 清除场景
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # 导入 OBJ
    bpy.ops.import_scene.obj(filepath=input_obj)

    # 获取所有网格对象
    mesh_objects = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    print(f"Found {len(mesh_objects)} mesh objects")

    if not mesh_objects:
        print(f"Error: No mesh objects found in {input_obj}")
        return False

    # 计算总面数
    total_faces = sum(len(o.data.polygons) for o in mesh_objects)
    print(f"Total original faces: {total_faces}")

    # 对所有对象应用 Decimate modifier
    ratio = 0.3
    modifiers = []
    for obj in mesh_objects:
        mod = obj.modifiers.new(name="Decimate", type='DECIMATE')
        mod.ratio = ratio
        modifiers.append((obj, mod))

    # 迭代调整直到文件大小满足要求
    max_iterations = 10
    for i in range(max_iterations):
        # 应用所有 modifiers
        for obj, mod in modifiers:
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_apply(modifier=mod.name)

        # 导出临时文件检查大小
        temp_output = output_obj + ".temp"
        bpy.ops.export_scene.obj(filepath=temp_output, use_selection=False, use_mesh_modifiers=True)

        # 检查文件大小
        file_size_mb = os.path.getsize(temp_output) / (1024 * 1024)
        new_total_faces = sum(len(o.data.polygons) for o in mesh_objects)
        print(f"Iteration {i+1}: ratio={ratio:.3f}, faces={new_total_faces}, size={file_size_mb:.1f}MB")

        if file_size_mb <= target_size_mb:
            # 满足要求，重命名临时文件
            os.rename(temp_output, output_obj)
            print(f"Success: Final size {file_size_mb:.1f}MB <= {target_size_mb}MB")
            return True

        # 文件仍然太大，继续减少
        os.remove(temp_output)
        ratio *= 0.5  # 进一步减少
        if ratio < 0.05:
            ratio = 0.05

        # 重新添加 modifiers
        modifiers = []
        for obj in mesh_objects:
            mod = obj.modifiers.new(name="Decimate", type='DECIMATE')
            mod.ratio = ratio
            modifiers.append((obj, mod))

    # 最后一次尝试
    bpy.ops.export_scene.obj(filepath=output_obj, use_selection=False, use_mesh_modifiers=True)
    final_size_mb = os.path.getsize(output_obj) / (1024 * 1024)
    print(f"Final: size={final_size_mb:.1f}MB")
    return final_size_mb <= target_size_mb

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: blender --background --python reduce_mesh.py -- <input.obj> <output.obj> [target_size_mb]")
        sys.exit(1)

    # Blender 参数后的参数
    argv = sys.argv
    if "--" in argv:
        idx = argv.index("--")
        args = argv[idx+1:]
    else:
        args = argv[1:]

    input_obj = args[0]
    output_obj = args[1]
    target_size_mb = float(args[2]) if len(args) > 2 else 45

    print(f"Reducing {input_obj} -> {output_obj} (target: {target_size_mb}MB)")
    success = reduce_mesh(input_obj, output_obj, target_size_mb)
    sys.exit(0 if success else 1)
