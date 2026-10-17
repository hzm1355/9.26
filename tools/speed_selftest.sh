#!/bin/bash
# 测速模块离线自测：合成位姿流 + 合成 10 Hz 时钟喂进回调，逐条断言五种行为。
# 不依赖 gzserver，所以能在拉起真车队之前先卡住回归（本轮 `copy_pose` 未定义
# 这个 NameError 被逐点位 try/except 咽掉，区间事件会静默变零 —— 只有自测能抓）。
# 前提：先跑 build_speed.sh，自测读的是 install 里的 speed_zones.yaml 与生成码。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd ~/v2x_ws
python3 src/v2x_demo/v2x_demo/speed_selftest.py
