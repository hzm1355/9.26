#!/usr/bin/env bash
# 广义排查：所有 gz/ros 进程 + /dev/shm 残留
echo "===== 所有 gazebo/ros 相关进程 ====="
ps -eo pid,etimes,comm,args | grep -iE 'gzserver|gzclient|gzmaster|slam|rviz|component_container|ros2|_node' | grep -v grep | grep -v 'bash -c' | cut -c1-140
echo ""
echo "===== /dev/shm 内容 (FastDDS SHM 残留?) ====="
ls -la /dev/shm/ 2>/dev/null | head -40
echo ""
echo "===== 强制清理所有 gz + shm ====="
pkill -9 -f gzserver   2>/dev/null
pkill -9 -f gzclient   2>/dev/null
pkill -9 -f gzmaster   2>/dev/null
pkill -9 -f 'ros2 launch' 2>/dev/null
pkill -9 -f static_transform 2>/dev/null
pkill -9 -f traffic_light 2>/dev/null
sleep 1
rm -rf /dev/shm/* 2>/dev/null
rm -rf /tmp/gazebo-* 2>/dev/null
echo "清理后 /dev/shm:"
ls -la /dev/shm/ 2>/dev/null | head
echo "清理后 gz 进程:"
pgrep -af 'gzserver|gzmaster' | grep -v 'bash -c' || echo "(空)"
