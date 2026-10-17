#!/usr/bin/env bash
# 只清理 Nav2 栈（slam + nav2 各节点），保留 gzserver/bringup/静态TF/信号灯。
# 关键词用 [x] 正则防自匹配误杀调用方 shell。
pkill -9 -f 'ros2 launc[h]'
pkill -9 -f 'slam_toolbo[x]'
pkill -9 -f 'controller_serve[r]'
pkill -9 -f 'planner_serve[r]'
pkill -9 -f 'bt_navigato[r]'
pkill -9 -f 'lifecycle_manage[r]'
pkill -9 -f 'smoother_serve[r]'
pkill -9 -f 'behavior_serve[r]'
pkill -9 -f 'waypoint_followe[r]'
pkill -9 -f 'velocity_smoothe[r]'
pkill -9 -f 'component_containe[r]'
sleep 1
echo "===== nav2 栈已清理（bringup 保留） ====="
