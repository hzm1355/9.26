#!/usr/bin/env bash
# 启动 SLAM + Nav2（可选 RViz，输出重定向 /tmp/nav2.log 防刷屏）
# 用法: bash run_nav2.sh [true|false]   参数=use_rviz，默认 false
#       VcXsrv 不可用时必须 false，否则 rviz 连不上 X 会 SIGABRT 连带关掉整个 Nav2 栈
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
RVIZ=${1:-false}
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
export LIBGL_ALWAYS_INDIRECT=1
echo "[run_nav2] use_rviz=$RVIZ DISPLAY=$DISPLAY，日志输出到 /tmp/nav2.log"
exec ros2 launch v2x_demo v2x_nav2.launch.py use_rviz:=$RVIZ > /tmp/nav2.log 2>&1
