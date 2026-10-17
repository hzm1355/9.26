#!/usr/bin/env bash
# 对当前正在运行的仿真抓一张界面实时快照（offscreen，不影响已弹出的 GUI 窗口）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
OUT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/dashboard_live_preview.png
rm -f "$OUT"
V2X_DASH_LIVE=1 V2X_DASH_LIVE_SECS=6 python3 "$D/dashboard_gui_smoke.py" "$OUT" 2>&1 | grep -iE 'status bar|saved|SMOKE'
ls -la "$OUT"
