#!/bin/bash
# 应用材质：MTL 的 map_ 行改为裸文件名 + 复制贴图到 meshes 目录
TEX_SRC=/mnt/d/desktop1/9.26.1/2
BASE=/home/hzm/v2x_ws/src/smart_car_sim/models

for m in room_33 room_20 room_12; do
  d="$BASE/$m/meshes"
  [ -d "$d" ] || continue
  echo "=== $m ==="
  for mtl in "$d"/*.mtl; do
    # 备份一次
    [ -f "$mtl.bak_orig" ] || cp "$mtl" "$mtl.bak_orig"
    # map_ 行: 取最后一个字段并去目录前缀
    awk '/^map_/{n=$NF; sub(/.*[/\\]/,"",n); print $1, n; next} {print}' "$mtl.bak_orig" > "$mtl"
    echo "  重写: $(basename "$mtl")"
  done
  # 复制贴图（不覆盖已有）
  cp -n "$TEX_SRC"/* "$d"/ 2>/dev/null
  echo "  贴图文件数: $(ls "$d" | grep -ciE '\.(jpg|jpeg|png)$')"
done
echo "=== 完成 ==="
