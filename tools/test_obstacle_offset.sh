#!/usr/bin/env bash
# obstacle_offset 离线自测：需要 ROS 环境（节点模块 import rclpy）
# 注意不要写 set -u —— ROS 的 setup.bash 会引用未定义变量。
set -e
source /opt/ros/humble/setup.bash
source "$HOME/v2x_ws/install/setup.bash"
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
python3 tools/test_obstacle_offset.py
