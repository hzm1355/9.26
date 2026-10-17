#!/bin/bash
# 在不重启 gzserver 的前提下运行时注入 gate_j3_north，验证 v2x_gate 抓拍插件。
set -e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
mkdir -p /tmp/v2x_gates

ros2 run gazebo_ros spawn_entity.py -entity gate_j3_north -file /tmp/gate_j3_north.sdf 2>&1 | tail -3

echo "--- 话题 ---"
sleep 4
ros2 topic list | grep -E 'gate_j3_north|plate/capture' || echo "(无)"
