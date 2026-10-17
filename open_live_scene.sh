#!/usr/bin/env bash
# 打开实景3D画面：确保仿真在跑 + 启动 dashboard GUI
set -e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1

# 1) 确保仿真在跑（没有 gzserver 就起一套最小的）
if ! pgrep -f '[g]zserver' >/dev/null 2>&1; then
  echo "[open_live] gzserver 未运行，启动最小仿真..."
  source /opt/ros/humble/setup.bash
  export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
  setsid nohup gzserver --verbose worlds/v2x_city.world \
    -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >/tmp/gzserver_live.log 2>&1 &
  sleep 6
fi

# 2) 启动 dashboard GUI（在用户桌面显示）
echo "[open_live] 启动 dashboard..."
source /opt/ros/humble/setup.bash
exec bash run_dashboard.sh
