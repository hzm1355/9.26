#!/bin/bash
# 干净启动 room_layout：gzserver + gzclient
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_PATH=/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:$LD_LIBRARY_PATH
export DISPLAY=:0
unset WAYLAND_DISPLAY
unset LIBGL_ALWAYS_INDIRECT

WORLD=/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_layout.world

# 清理
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
sleep 2

# 启动 gzserver
gzserver --verbose "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
GZSERVER_PID=$!
echo "gzserver pid=$GZSERVER_PID"

# 等 gzserver 就绪
for i in $(seq 1 30); do
  if gz stats -d 0 >/dev/null 2>&1; then echo "gzserver ready after ${i}s"; break; fi
  sleep 1
done

# 启动 gzclient
gzclient &
GZCLIENT_PID=$!
echo "gzclient pid=$GZCLIENT_PID"

sleep 3
ps -eo pid,args | grep -E 'gzserver|gzclient' | grep -v grep
echo "=== 启动完成 ==="
