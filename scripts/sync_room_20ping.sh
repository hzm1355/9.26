#!/bin/bash
# 同步 room_20ping 相关文件到 WSL 工作区并编译
set -e

SRC="/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim"
DST="/home/hzm/v2x_ws/src/smart_car_sim"

echo "=== [1/4] 同步 model 目录 ==="
mkdir -p "$DST/models/room_20ping/meshes"
cp -v "$SRC/models/room_20ping/model.sdf"    "$DST/models/room_20ping/"
cp -v "$SRC/models/room_20ping/model.config" "$DST/models/room_20ping/"
cp -v "$SRC/models/room_20ping/meshes/room_20ping.obj" "$DST/models/room_20ping/meshes/"
cp -v "$SRC/models/room_20ping/meshes/room_20ping.mtl" "$DST/models/room_20ping/meshes/"

echo "=== [2/4] 同步 world 文件 ==="
cp -v "$SRC/worlds/room_20ping.world" "$DST/worlds/"

echo "=== [3/4] 同步 launch 文件 ==="
cp -v "$SRC/launch/room_20ping.launch.py" "$DST/launch/"

echo "=== [4/4] colcon build ==="
cd /home/hzm/v2x_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_car_sim --allow-overriding smart_car_sim 2>&1 | tail -20

echo "=== 验证安装产物 ==="
ls -la /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models/room_20ping/meshes/ 2>/dev/null
ls -la /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_20ping.world 2>/dev/null
ls -la /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/launch/room_20ping.launch.py 2>/dev/null
echo "=== 同步完成 ==="
