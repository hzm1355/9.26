#!/bin/bash
# Match MTL texture refs to files in "2" folder
TEX_DIR="/mnt/d/desktop1/9.26.1/2"
MTL="/home/hzm/v2x_ws/src/smart_car_sim/models/room_33/meshes/room_33_blender.mtl"

echo "=== MTL map_Kd refs vs available textures ==="
echo ""

# Get list of available texture files (basenames)
avail_files=$(ls "$TEX_DIR" | grep -E '\.(jpg|JPG|jpeg|png)$' | sort)

# Check each map_Kd in MTL
while IFS= read -r line; do
    # Extract filename
    fname=$(echo "$line" | sed 's/^map_Kd //')
    
    # Check if file exists in texture dir
    if [ -f "$TEX_DIR/$fname" ]; then
        echo "✓ FOUND: $fname"
    else
        echo "✗ MISSING: $fname"
    fi
done < <(grep '^map_Kd' "$MTL")

echo ""
echo "=== Available textures in '2' folder ==="
echo "$avail_files" | wc -l
echo "files total"
