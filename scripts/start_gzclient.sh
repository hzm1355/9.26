#!/bin/bash
# 在 WSL 中启动 Gazebo GUI 客户端（VcXsrv 显示）
# 用法（WSL 内）: bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/start_gzclient.sh

source /opt/ros/humble/setup.bash

WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
export GAZEBO_MODEL_PATH="$HOME/v2x_models:$WS/src/smart_car_sim/models:$GAZEBO_MODEL_PATH"

export DISPLAY=$(ip route show default | awk '{print $3}'):1
unset LIBGL_ALWAYS_INDIRECT

killall -q gzclient 2>/dev/null
sleep 1

setsid nohup gzclient > /tmp/gzclient.log 2>&1 < /dev/null &
echo "gzclient started, pid $!"

sleep 8
if pgrep -x gzclient > /dev/null; then
    echo "gzclient alive: $(pgrep -x gzclient | head -1)"
else
    echo "ERROR: gzclient died. Last log lines:"
    tail -20 /tmp/gzclient.log
    exit 1
fi
