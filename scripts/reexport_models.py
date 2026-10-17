#!/usr/bin/env python3
"""
Re-export OBJ models with proper materials for Gazebo
- Read MTL Kd colors
- Assign distinct colors to each material
- Export as DAE (better material support than OBJ)
"""
import bpy
import os
import sys

def patch_colorspace_fallback():
    """Monkey-patch Blender 3.0.1 OCIO fallback"""
    try:
        import bpy.types
        orig_assign = bpy.types.ImageTextureNode.color_space.fset
        def safe_assign(self, value):
            try:
                orig_assign(self, value)
            except TypeError:
                pass
        bpy.types.ImageTextureNode.color_space.fset = safe_assign
    except:
        pass

def process_model(obj_path, output_dir):
    """Process one OBJ model"""
    # Clear scene
    bpy.ops.wm.read_factory_settings(use_empty=True)
    
    # Import OBJ
    bpy.ops.wm.obj_import(
        filepath=obj_path,
        forward_axis='Y',
        up_axis='Z'
    )
    
    # Get all mesh objects
    mesh_objs = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    print(f"Imported {len(mesh_objs)} mesh objects")
    
    # Join all meshes
    if len(mesh_objs) > 1:
        bpy.ops.object.select_all(action='DESELECT')
        for obj in mesh_objs:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = mesh_objs[0]
        bpy.ops.object.join()
        print("Joined all meshes")
    
    # Get the joined object
    obj = bpy.context.active_object
    if obj is None:
        print("ERROR: No active object")
        return False
    
    # Apply transforms
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    
    # Create materials based on MTL or assign distinct colors
    # For now, assign a simple gray material
    mat = bpy.data.materials.new(name="Default")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs['Base Color'].default_value = (0.7, 0.7, 0.7, 1.0)
    obj.data.materials.append(mat)
    
    # Export as DAE
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "model.dae")
    bpy.ops.wm.collada_export(
        filepath=output_path,
        apply_modifiers=True,
        export_selected=False,
        include_children=True,
        include_armatures=False,
        include_shapekeys=False,
        deform_bones_only=False,
        export_animations=False,
        export_texture_include=False,
        sampling_rate=1.0,
        keep_bind_info=False
    )
    print(f"Exported to {output_path}")
    return True

def main():
    patch_colorspace_fallback()
    
    base_dir = "/home/hzm/v2x_ws/src/smart_car_sim/models"
    
    for room in ["room_33", "room_20", "room_12"]:
        obj_path = os.path.join(base_dir, room, "meshes", f"{room}.obj")
        output_dir = os.path.join(base_dir, room, "meshes")
        
        if os.path.exists(obj_path):
            print(f"\n=== Processing {room} ===")
            if process_model(obj_path, output_dir):
                print(f"✓ {room} done")
            else:
                print(f"✗ {room} failed")
        else:
            print(f"SKIP {obj_path} not found")

if __name__ == "__main__":
    main()
