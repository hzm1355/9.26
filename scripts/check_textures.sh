#!/bin/bash
# 给 room_33/room_20/room_12 应用材质：MTL 贴图路径改为裸文件名 + 复制贴图到 meshes 目录
TEX_SRC=/mnt/d/desktop1/9.26.1/2
BASE=/home/hzm/v2x_ws/src/smart_car_sim/models

for m in room_33 room_20 room_12; do
  d="$BASE/$m/meshes"
  echo "=== $m ==="
  if [ ! -d "$d" ]; then echo "(无 meshes 目录, 跳过)"; continue; fi
  ls "$d"
  for obj in "$d"/*.obj; do
    grep -m1 mtllib "$obj" 2>/dev/null
  done
done

echo "=== 贴图匹配检查 (room_33) ==="
d="$BASE/room_33/meshes"
grep -oh 'map_Kd .*' "$d"/*.mtl 2>/dev/null | sed 's/map_Kd //' | sed -E 's|([^/\\ ]+[/\\])+||' | sort -u > /tmp/refs.txt
while read -r f; do
  if [ -f "$TEX_SRC/$f" ]; then echo "有: $f"; else echo "缺: $f"; fi
done < /tmp/refs.txt
