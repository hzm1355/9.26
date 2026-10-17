#!/usr/bin/env bash
# ============================================================================
# 智能小车避障启动脚本
# 用法: bash run_obstacle_avoidance.sh [car_name] [max_speed] [stop_distance]
#   例: bash run_obstacle_avoidance.sh smart_car_1 0.5 0.8
# ============================================================================
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

CAR_NAME=${1:-smart_car_1}
MAX_SPEED=${2:-0.5}
STOP_DISTANCE=${3:-0.8}

echo "[避障] 启动避障节点: car=$CAR_NAME speed=$MAX_SPEED stop_dist=$STOP_DISTANCE"
echo "[避障] 话题: /$CAR_NAME/scan -> /$CAR_NAME/cmd_vel"
echo "[避障] 按 Ctrl+C 停止"
echo ""

ros2 launch v2x_demo obstacle_avoidance.launch.py \
  car_name:=$CAR_NAME \
  max_speed:=$MAX_SPEED \
  stop_distance:=$STOP_DISTANCE
