#!/usr/bin/env bash
# 单独直跑 gzserver（headless，不带 ROS 插件），隔离崩溃原因
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
echo "===== gzserver 直跑测试 (verbose, 最多 20s) ====="
timeout 20 gzserver --verbose "$WORLD" 2>&1 | tail -40
echo "===== 退出码: ${PIPESTATUS[0]} ====="
