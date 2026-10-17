#!/usr/bin/env bash
# ============================================================================
# 启动 V2X 可视化操作界面（PyQt5 大屏界面）。
# 前置：先 bash start_all.sh 起仿真（gzserver + 信号灯 + 多车 + 记录器）。
# 显示走 WSLg 或 VcXsrv（与 gzclient 同一 DISPLAY）。
# 用法: bash run_dashboard.sh
# ============================================================================
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
# Windows 10 用 VcXsrv/Xming；DISPLAY=:0 是标准配置
export DISPLAY=:0
unset LIBGL_ALWAYS_INDIRECT

echo "[dashboard] 启动可视化界面（F11 全屏）..."
ros2 run v2x_demo v2x_dashboard.py --ros-args -p use_sim_time:=true
