#!/bin/bash
# 跑 parking_shuttle 的离线自测（不需要 Gazebo）。
#
# 为什么要包一层：rclpy 与 v2x_msgs 只在 source 过工作区之后才 import 得到，而
# D 盘的文件是 CRLF，python 直接执行会栽在 \r 上 —— 所以先 sed 去 CR 再跑。
# 退出码自己判（不把 python 输出接进 tail，否则失败被吞）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM"
if python3 tools/parking_shuttle_selftest.py; then
  echo "SHUTTLE_SELFTEST_OK"
else
  echo "SHUTTLE_SELFTEST_FAILED"
  exit 1
fi
