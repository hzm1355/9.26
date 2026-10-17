#!/bin/bash
# Check texture matching for all room models
BASE=/home/hzm/v2x_ws/src/smart_car_sim/models

for m in room_33 room_20 room_12; do
  dir="$BASE/$m/meshes"
  echo "=== $m ==="
  
  # Find the MTL referenced by OBJ
  obj=$(ls "$dir"/*.obj 2>/dev/null | head -1)
  if [ -z "$obj" ]; then
    echo "  No OBJ found"
    continue
  fi
  mtl_ref=$(grep '^mtllib' "$obj" | head -1 | awk '{print $2}')
  echo "  OBJ: $(basename $obj)"
  echo "  MTL ref: $mtl_ref"
  mtl="$dir/$mtl_ref"
  
  if [ ! -f "$mtl" ]; then
    echo "  MTL file NOT FOUND!"
    continue
  fi
  
  # Count materials with and without map_Kd
  total_mats=$(grep -c '^newmtl' "$mtl")
  mats_with_map=$(grep -c '^map_Kd' "$mtl")
  echo "  Materials: $total_mats, with map_Kd: $mats_with_map"
  
  # Check each map_Kd target
  found=0
  missing=0
  while IFS= read -r line; do
    # Extract filename after map_Kd
    fname=$(echo "$line" | sed 's/^map_Kd //')
    if [ -f "$dir/$fname" ]; then
      found=$((found+1))
    else
      missing=$((missing+1))
      if [ $missing -le 10 ]; then
        echo "  MISSING: [$fname]"
      fi
    fi
  done < <(grep '^map_Kd' "$mtl")
  echo "  Textures found: $found, missing: $missing"
  
  # List texture files in dir
  tex_count=$(ls "$dir"/*.jpg "$dir"/*.JPG "$dir"/*.jpeg "$dir"/*.png 2>/dev/null | wc -l)
  echo "  Texture files in dir: $tex_count"
done
