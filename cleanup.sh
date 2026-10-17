#!/usr/bin/env bash
# 彻底清理仿真/导航进程 + Gazebo 残留共享内存/socket
pkill -9 -f 'ros2 launch'      2>/dev/null
pkill -9 -f gzserver           2>/dev/null
pkill -9 -f gzclient           2>/dev/null
pkill -9 -f static_transform   2>/dev/null
pkill -9 -f traffic_light      2>/dev/null
pkill -9 -f 'region_trigge[r]' 2>/dev/null
pkill -9 -f spawn_entity       2>/dev/null
pkill -9 -f slam_toolbox       2>/dev/null
pkill -9 -f component_container 2>/dev/null
pkill -9 -f rviz2              2>/dev/null
sleep 2
rm -rf /tmp/gazebo-* 2>/dev/null
rm -rf /dev/shm/* 2>/dev/null
echo "===== 清理完成，剩余相关进程 ====="
pgrep -af 'gzserver|gzclient|slam_toolbox|rviz2|ros2 launch' | grep -v 'bash -c' || echo "(空)"
