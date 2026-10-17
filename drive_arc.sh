#!/usr/bin/env bash
# 手动驱动 prius（发布 cmd_vel 弧线行驶，用于建图补扫/微调车位）
# 用法: bash drive_arc.sh <linear_x> <angular_z> <秒数>
#   例: bash drive_arc.sh 0.6 0.35 4   → 前进0.6m/s 左转0.35rad/s 持续4s
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
V=${1:-0.6}
W=${2:-0.35}
T=${3:-4}
echo "[drive_arc] v=$V m/s, w=$W rad/s, ${T}s"
timeout "$T" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $V}, angular: {z: $W}}" > /dev/null 2>&1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1
echo "[drive_arc] done"
