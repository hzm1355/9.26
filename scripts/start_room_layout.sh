#!/bin/bash
# 启动 room_layout：gzserver + gzclient
export GAZEBO_MODEL_PATH=/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:$LD_LIBRARY_PATH
unset LIBGL_ALWAYS_INDIRECT

WORLD=/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_layout.world

# 清理旧会话
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
sleep 1

# 启动 gzserver
setsid nohup gzserver --verbose "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so >/tmp/gzserver_room_layout.log 2>&1 &
echo "gzserver pid=$!"

# 等 gzserver 就绪
for i in $(seq 1 20); do
  if gz stats -d 0 >/dev/null 2>&1; then echo "gzserver ready"; break; fi
  sleep 1
done

# 启动 gzclient（需要 DISPLAY）
export DISPLAY=:0
setsid nohup gzclient >/tmp/gzclient_room_layout.log 2>&1 &
echo "gzclient pid=$!"
echo "=== 启动完成 ==="
