#!/usr/bin/env bash
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
SRC=$SIM/src/v2x_demo
cd "$SIM"
python3 scene_builder.py v2x_scene.yaml "$SRC/worlds/v2x_city.world" "$SRC/config/scene_devices.yaml"
echo "=== 校验生成的信号灯（内联 + 插件 + 组话题） ==="
grep -c "libv2x_traffic_light.so" "$SRC/worlds/v2x_city.world"
grep -o "traffic_light/[A-Za-z0-9_]*/state" "$SRC/worlds/v2x_city.world" | sort | uniq -c
echo "=== 设备表信号灯分组 ==="
grep -A2 "group:" "$SRC/config/scene_devices.yaml" | head -30
echo "=== 场景相机 ==="
grep -c "libgazebo_ros_camera.so" "$SRC/worlds/v2x_city.world"
grep -o "v2x_scene_cam" "$SRC/worlds/v2x_city.world" | head -2
echo "REGEN_DONE"
