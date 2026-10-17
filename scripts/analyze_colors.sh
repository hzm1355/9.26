#!/bin/bash
# Analyze Kd color distribution in MTL files
BASE=/home/hzm/v2x_ws/src/smart_car_sim/models

for m in room_33 room_20 room_12; do
  mtl="$BASE/$m/meshes/${m}_blender.mtl"
  echo "=== $m ==="
  
  if [ ! -f "$mtl" ]; then
    echo "  MTL not found"
    continue
  fi
  
  # Count materials by Kd brightness
  awk '
    /^newmtl/ { mat_count++ }
    /^Kd/ {
      r = $2; g = $3; b = $4
      brightness = (r + g + b) / 3
      if (brightness > 0.7) light++
      else if (brightness < 0.3) dark++
      else mid++
    }
    END {
      print "  Total materials:", mat_count+0
      print "  Light (>0.7):", light+0
      print "  Mid (0.3-0.7):", mid+0
      print "  Dark (<0.3):", dark+0
    }
  ' "$mtl"
  
  # Show first 5 materials with their Kd and map_Kd
  echo "  Sample materials:"
  awk '
    /^newmtl/ { name=$2; has_map=0 }
    /^Kd/ { kd=$2" "$3" "$4 }
    /^map_Kd/ { has_map=1; map=$2 }
    /^$/ || /^newmtl/ {
      if (name != "" && NR > 1) {
        if (has_map) print "    " name " (Kd:" kd " map:" map ")"
        else print "    " name " (Kd:" kd " no-map)"
      }
    }
  ' "$mtl" | head -10
done
