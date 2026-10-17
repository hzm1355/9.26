#!/bin/bash
# 在 WSL 中启动 v2x_city_scene 世界（沙盘 + car_1 + 摄像头）
# 用法（WSL 内）: bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/start_gazebo_city.sh

killall -q gzserver gzclient 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash 2>/dev/null

WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
# WSL 本地盘模型优先（9P 读取 470MB OBJ 太慢）
export GAZEBO_MODEL_PATH="$HOME/v2x_models:$WS/src/smart_car_sim/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="/home/hzm/v2x_ws/install/v2x_gazebo_plugins/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:$GAZEBO_PLUGIN_PATH"

# 相机传感器需要 GL 渲染上下文（VcXsrv 在 Windows 侧 :1）
export DISPLAY=$(ip route show default | awk '{print $3}'):1
unset LIBGL_ALWAYS_INDIRECT

cd "$WS" || exit 1

setsid nohup gzserver src/smart_car_sim/worlds/v2x_city_scene.world \
    -s libgazebo_ros_init.so -s libgazebo_ros_factory.so --verbose \
    > /tmp/gzserver.log 2>&1 < /dev/null &
echo "gzserver started, pid $!"

# 等待 gzserver 存活并开始加载世界
sleep 10
if ! pgrep -x gzserver > /dev/null; then
    echo "ERROR: gzserver died. Last log lines:"
    tail -20 /tmp/gzserver.log
    exit 1
fi

echo "gzserver alive: $(pgrep -x gzserver | head -1)"
tail -5 /tmp/gzserver.log
