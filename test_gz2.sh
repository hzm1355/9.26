#!/usr/bin/env bash
# 带 ROS 插件直跑 gzserver，复现并定位段错误
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
echo "===== 是否有残留 gzmaster ====="
pgrep -af gzmaster || echo "(无 gzmaster)"
echo ""
echo "===== gzserver 带三插件直跑 (最多 20s) ====="
timeout 20 gzserver --verbose "$WORLD" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so 2>&1 | tail -50
echo "===== 退出码: ${PIPESTATUS[0]} (124=存活被timeout, 139=SIGSEGV) ====="
