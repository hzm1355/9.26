#!/bin/bash
# measure_pivot139.py 的运行环境包装（rclpy 要 source 两个 setup.bash 才 import 得到）。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
timeout -k 5 200 python3 -u measure_pivot139.py
echo "[probe] 退出码 $?"
gz model -m car_6 -p 2>&1 | head -2
exit 0
