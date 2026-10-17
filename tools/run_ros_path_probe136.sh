#!/bin/bash
# 跑 ros_path_probe136.py：必须先 source 两套环境，否则 rclpy / v2x_msgs 导不到（#135 踩过）。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
python3 -u tools/ros_path_probe136.py
echo "EXIT=$?"
