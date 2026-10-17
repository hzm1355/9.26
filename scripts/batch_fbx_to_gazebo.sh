#!/bin/bash
# FBX → Gazebo 模型一键转换脚本
# 用法：./batch_fbx_to_gazebo.sh <fbx文件> [fbx文件2 ...]
#
# 流程：
#   1. Blender 导入 FBX → 上色 → 减面 → 导出 DAE + OBJ
#   2. 复制网格文件到 Gazebo 模型目录
#   3. 生成/更新 SDF 文件
#
# 前置条件：
#   - blender 在 PATH 中（或修改下方 BLENDER 变量）
#   - 模型目录已存在于 src/smart_car_sim/models/room_*/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$PROJECT_DIR/src/smart_car_sim/models"
BLENDER_SCRIPT="$SCRIPT_DIR/fbx_to_dae.py"

BLENDER="blender"

if [ $# -eq 0 ]; then
    echo "用法: $0 <file1.fbx> [file2.fbx ...]"
    echo ""
    echo "示例:"
    echo "  $0 ~/fbx_work/room_20.fbx"
    echo "  $0 ~/fbx_work/room_*.fbx"
    exit 1
fi

for fbx_file in "$@"; do
    echo ""
    echo "=========================================="
    echo "处理: $fbx_file"
    echo "=========================================="

    if [ ! -f "$fbx_file" ]; then
        echo "错误: 文件不存在 $fbx_file"
        continue
    fi

    fbx_basename="$(basename "$fbx_file" .fbx)"
    fbx_dir="$(cd "$(dirname "$fbx_file")" && pwd)"

    model_name="$fbx_basename"
    model_dir="$MODELS_DIR/$model_name"
    meshes_dir="$model_dir/meshes"

    if [ ! -d "$model_dir" ]; then
        echo "模型目录不存在，创建: $model_dir"
        mkdir -p "$model_dir"
    fi
    mkdir -p "$meshes_dir"

    echo ""
    echo "[1/3] Blender 转换 (FBX → DAE + OBJ)..."
    "$BLENDER" --background --python "$BLENDER_SCRIPT" -- "$fbx_file"

    dae_file="$fbx_dir/${fbx_basename}_blender.dae"
    obj_file="$fbx_dir/${fbx_basename}_blender.obj"

    echo ""
    echo "[2/3] 复制网格文件到模型目录..."
    if [ -f "$dae_file" ]; then
        cp "$dae_file" "$meshes_dir/${model_name}.dae"
        echo "  已复制: ${model_name}.dae"
    else
        echo "  警告: DAE 文件未生成"
    fi
    if [ -f "$obj_file" ]; then
        cp "$obj_file" "$meshes_dir/${model_name}.obj"
        echo "  已复制: ${model_name}.obj"
    fi

    # 复制可能存在的纹理文件
    for tex_ext in png jpg jpeg tga bmp; do
        for tex in "$fbx_dir"/*."$tex_ext"; do
            if [ -f "$tex" ]; then
                cp "$tex" "$meshes_dir/"
                echo "  已复制纹理: $(basename "$tex")"
            fi
        done
    done

    echo ""
    echo "[3/3] 生成 SDF..."
    python "$SCRIPT_DIR/gen_sdf.py" "$model_name"

    echo ""
    echo "✓ $model_name 处理完成"
done

echo ""
echo "=========================================="
echo "全部完成！"
echo "=========================================="
echo ""
echo "模型目录结构:"
echo "  $MODELS_DIR/"
echo "  ├── room_20/"
echo "  │   ├── model.config"
echo "  │   ├── model.sdf"
echo "  │   └── meshes/"
echo "  │       ├── room_20.dae"
echo "  │       └── room_20.obj"
echo "  └── ..."
