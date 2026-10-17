#!/bin/bash
# 跑 seat_guard126.py（恢复 car_6 到等待位并闭环校验就位）。
# 环境必须两份 setup 都 source：/opt/ros/humble 给 rclpy，~/v2x_ws/install 给 v2x_msgs。
# 用 setsid 前台跑：WSL 一次性命令里 nohup & 的长驻进程会被回收（本轮探针就是这么没跑起来的）。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
timeout -k 5 200 python3 -u tools/seat_guard126.py
echo "EXIT=$?"
