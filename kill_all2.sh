#!/usr/bin/env bash
# 全量清理 V2X 仿真/导航进程（从脚本调用，关键词用 [x] 正则防自匹配，
# 避免此前 cleanup.sh 被 "pkill gzserver 杀掉含同关键词的父shell后 HUP 自身" 的连锁坑）
pkill -9 -f 'start_all.s[h]'
pkill -9 -f 'gzserve[r]'
pkill -9 -f 'gzclien[t]'
pkill -9 -f 'gzmaste[r]'
pkill -9 -f 'static_transform_publishe[r]'
pkill -9 -f 'traffic_light_[cm]'
pkill -9 -f 'region_trigge[r]'
pkill -9 -f 'slam_toolbo[x]'
pkill -9 -f 'ros2 launc[h]'
pkill -9 -f 'controller_serve[r]'
pkill -9 -f 'planner_serve[r]'
pkill -9 -f 'bt_navigato[r]'
pkill -9 -f 'lifecycle_manage[r]'
pkill -9 -f 'smoother_serve[r]'
pkill -9 -f 'behavior_serve[r]'
pkill -9 -f 'waypoint_followe[r]'
pkill -9 -f 'velocity_smoothe[r]'
pkill -9 -f 'component_containe[r]'
pkill -9 -f 'spawn_entit[y]'
pkill -9 -f 'tf2_ech[o]'
pkill -9 -f 'ros2 topi[c]'
pkill -9 -f 'ros2 servic[e]'
sleep 2
rm -rf /tmp/gazebo-* /dev/shm/* 2>/dev/null
echo "===== kill_all2 清理完成 ====="
