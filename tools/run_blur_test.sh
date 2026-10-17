#!/bin/bash
# 离线跑运动模糊容忍上限表（需要 source ROS 才能 import rclpy——识别器节点模块带着它）。
# 不占用仿真实时性的 CPU 活，但还是 nice 到最低，免得跟速度扫参抢渲染线程。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM"
nice -n 19 python3 -u tools/speed_blur_test.py "${1:-680}"
