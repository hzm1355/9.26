#!/bin/bash
# Fix MTL: remove invalid texture refs, keep valid ones
BASE=/home/hzm/v2x_ws/src/smart_car_sim/models
TEX_DIR="/mnt/d/desktop1/9.26.1/2"

for m in room_33 room_20 room_12; do
  mtl="$BASE/$m/meshes/${m}_blender.mtl"
  echo "=== Fixing $m ==="
  
  if [ ! -f "$mtl" ]; then
    echo "  MTL not found"
    continue
  fi
  
  # Backup
  cp "$mtl" "$mtl.bak_before_fix"
  
  # Use Python to process MTL
  python3 - "$mtl" "$TEX_DIR" << 'PYEOF'
import sys
import os

mtl_path = sys.argv[1]
tex_dir = sys.argv[2]

with open(mtl_path, 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

valid_extensions = {'.jpg', '.JPG', '.jpeg', '.png', '.bmp', '.tga', '.tif', '.tiff'}

new_lines = []
removed = 0
for line in lines:
    stripped = line.strip()
    if stripped.startswith('map_Kd') or stripped.startswith('map_Bump') or stripped.startswith('map_Ke'):
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            removed += 1
            continue
        
        fname = parts[1].strip()
        _, ext = os.path.splitext(fname)
        
        if ext.lower() not in valid_extensions:
            removed += 1
            continue
        
        if os.path.exists(os.path.join(tex_dir, fname)):
            new_lines.append(line)
        else:
            removed += 1
    else:
        new_lines.append(line)

with open(mtl_path, 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print(f"  Original: {len(lines)} lines, Kept: {len(new_lines)} lines, Removed: {removed}")
PYEOF
  
  echo ""
done
