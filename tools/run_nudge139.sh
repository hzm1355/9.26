#!/bin/bash
# nudge_out_of_circle139.py 的运行环境包装：rclpy 只有在 source 过两个 setup.bash
# 之后才 import 得到（裸跑报 ModuleNotFoundError: No module named 'rclpy'）。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
timeout -k 5 240 python3 -u nudge_out_of_circle139.py
echo "[nudge] 退出码 $?"
gz model -m car_6 -p 2>&1 | head -2
