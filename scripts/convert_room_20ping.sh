#!/bin/bash
# room_20ping: 20平.fbx → OBJ (激进减面) → Gazebo 模型目录
set -e

FBX="/mnt/d/desktop1/9.26.1/20平.fbx"
SCRIPT="/mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/fbx_to_gazebo.py"
OUTDIR="/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/models/room_20ping/meshes"
TARGET_FACES="${1:-500000}"

mkdir -p "$OUTDIR"
echo "FBX存在: $(test -f "$FBX" && echo yes || echo NO)"
echo "目标面数: $TARGET_FACES"
echo "开始 Blender 转换..."

blender --background --python "$SCRIPT" -- "$FBX" "$OUTDIR/room_20ping.obj" "$TARGET_FACES" 2>&1 | tail -80

echo "=== 产物 ==="
ls -la "$OUTDIR"
