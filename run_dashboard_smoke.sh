#!/usr/bin/env bash
# 可视化界面 GUI 冒烟测试（offscreen 渲染出图，不需显示器/Gazebo）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
OUT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/_smoke_dashboard.png
rm -f "$OUT" "${OUT%.png}_scene.png" "${OUT%.png}_scene3d.png" "${OUT%.png}_scene_cam.png"
python3 "$D/dashboard_gui_smoke.py" "$OUT"
echo "SMOKE_EXIT=$?"
ls -la "$OUT" "${OUT%.png}_scene.png" "${OUT%.png}_scene3d.png" "${OUT%.png}_scene_cam.png" 2>/dev/null
