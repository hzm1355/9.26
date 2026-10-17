#!/usr/bin/env bash
# 可视化界面数据层无头自测（QT offscreen，不需显示）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
python3 "$D/dashboard_selftest.py"
echo "SELFTEST_EXIT=$?"
