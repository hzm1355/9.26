"""
为 Gazebo 模型生成/更新 SDF 文件（引用 DAE 网格）
用法：python gen_sdf.py <model_name> [<model_name2> ...]
      python gen_sdf.py --all          # 处理 models/ 下所有模型

示例：
  python gen_sdf.py room_20
  python gen_sdf.py --all
"""
import os
import sys
import glob

MODELS_DIR = os.path.join(os.path.dirname(__file__), '..', 'src', 'smart_car_sim', 'models')

SDF_TEMPLATE = '''\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/{mesh_file}</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/{mesh_file}</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'''

CONFIG_TEMPLATE = '''\
<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>V2X Traffic Sim</name>
  </author>
  <description>
    {name} 户型模型（从FBX转换）
  </description>
</model>
'''


def find_mesh_file(meshes_dir, name):
    """在 meshes 目录中查找可用的网格文件，优先 DAE"""
    for ext in ['.dae', '.obj']:
        candidate = os.path.join(meshes_dir, f"{name}{ext}")
        if os.path.exists(candidate):
            return f"{name}{ext}"
        candidate_blender = os.path.join(meshes_dir, f"{name}_blender{ext}")
        if os.path.exists(candidate_blender):
            return f"{name}_blender{ext}"
    return f"{name}.dae"


def process_model(name):
    model_dir = os.path.join(MODELS_DIR, name)
    meshes_dir = os.path.join(model_dir, 'meshes')

    os.makedirs(meshes_dir, exist_ok=True)

    mesh_file = find_mesh_file(meshes_dir, name)

    sdf_path = os.path.join(model_dir, 'model.sdf')
    sdf_content = SDF_TEMPLATE.format(name=name, mesh_file=mesh_file)
    with open(sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)
    print(f"  SDF: {sdf_path} (mesh: {mesh_file})")

    config_path = os.path.join(model_dir, 'model.config')
    if not os.path.exists(config_path):
        config_content = CONFIG_TEMPLATE.format(name=name)
        with open(config_path, 'w', encoding='utf-8') as f:
            f.write(config_content)
        print(f"  Config: {config_path} (新建)")
    else:
        print(f"  Config: {config_path} (已存在，跳过)")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    if sys.argv[1] == '--all':
        models = []
        for entry in sorted(os.listdir(MODELS_DIR)):
            full = os.path.join(MODELS_DIR, entry)
            if os.path.isdir(full) and entry.startswith('room_'):
                models.append(entry)
        if not models:
            print("未找到任何 room_* 模型目录")
            sys.exit(1)
        print(f"发现 {len(models)} 个模型: {', '.join(models)}")
    else:
        models = sys.argv[1:]

    for name in models:
        print(f"\n处理: {name}")
        process_model(name)

    print("\n=== 全部完成 ===")


if __name__ == '__main__':
    main()
