#!/bin/bash
# 跑 wheel_target132.py（摆位机制三相实验）。
# 环境：/opt/ros/humble 给 rclpy，~/v2x_ws/install 给 v2x_msgs；gz CLI 要 GAZEBO_MODEL_PATH。
# 前台跑：WSL 一次性命令里 nohup & 的长驻进程会被回收。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
timeout -k 5 240 python3 -u tools/wheel_target132.py 2>&1 | tee /tmp/_wt132.txt
echo "EXIT=$?"
