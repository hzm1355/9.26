#!/bin/bash
# 跑 cmd_authority135.py（cmd_vel 通路裁决，全程约 55 s）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
timeout -k 5 180 python3 -u tools/cmd_authority135.py
echo "EXIT=$?"
