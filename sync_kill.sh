#!/usr/bin/env bash
# 清理所有仿真/导航进程 + 同步 Windows 侧源码到 WSL 工作区
set -e
echo "===== 杀掉旧进程 ====="
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f spawn_entity 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
pkill -9 -f 'traffic_light_' 2>/dev/null || true
pkill -9 -f 'region_trigge[r]' 2>/dev/null || true
sleep 2
echo "剩余相关进程（应为空）："
pgrep -af 'gzserver|slam_toolbox|component_container|rviz2|static_transform' || echo "(已全部清理)"

echo ""
echo "===== 同步 launch 源码 ====="
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
DST=~/v2x_ws/src/v2x_demo
cp "$SRC/launch/v2x_bringup.launch.py" "$DST/launch/v2x_bringup.launch.py"
# 清除 Windows CRLF
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r$//' {} +
echo "同步完成，base_footprint_broadcaster 是否已进 install(symlink)："
grep -c base_footprint_broadcaster ~/v2x_ws/install/v2x_demo/share/v2x_demo/launch/v2x_bringup.launch.py
