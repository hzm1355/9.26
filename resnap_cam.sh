#!/usr/bin/env bash
# 对当前已运行的仿真抓一张实景3D 快照（不重启 gzserver）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
OUT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/dashboard_live_preview_cam.png
rm -f "$OUT" "${OUT%.png}_scene_cam.png"
V2X_DASH_LIVE=1 V2X_DASH_LIVE_SECS=10 python3 "$D/dashboard_gui_smoke.py" "$OUT" 2>&1 | grep -iE 'status bar|scene_cam saved|SMOKE'
ls -la "$OUT" "${OUT%.png}_scene_cam.png" 2>/dev/null
echo RESNAP_DONE
