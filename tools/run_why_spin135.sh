#!/bin/bash
# 跑 why_spin135.py：必须先 source 两份 setup（/opt/ros/humble 给 rclpy，
# ~/v2x_ws/install 给 v2x_msgs），一次性命令里直接 python3 是找不到 rclpy 的。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
timeout -k 5 200 python3 -u tools/why_spin135.py
echo "EXIT=$?"
