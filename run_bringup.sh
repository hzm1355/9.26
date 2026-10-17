#!/usr/bin/env bash
# 启动 bringup（Gazebo + prius + 信号灯 + TF 广播）
# 用法: bash run_bringup.sh [true|false]   参数=gui，默认 false(headless)
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
GUI=${1:-false}
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
export LIBGL_ALWAYS_INDIRECT=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
echo "[run_bringup] gui=$GUI DISPLAY=$DISPLAY"
exec ros2 launch v2x_demo v2x_bringup.launch.py gui:=$GUI
