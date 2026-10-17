#!/bin/bash
# Start gzserver with logging for texture debugging
export GAZEBO_MODEL_PATH=/home/hzm/v2x_ws/src/smart_car_sim/models:/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models
export GAZEBO_RESOURCE_PATH=/opt/ros/humble/share/gazebo_ros:/opt/ros/humble/share/gazebo_plugins
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

LOG=/tmp/gzserver_texture.log
echo "Starting gzserver at $(date)" > "$LOG"

gzserver --verbose \
  /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_layout.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  >> "$LOG" 2>&1 &

echo "gzserver PID: $!"
echo "Waiting 10s for models to load..."
sleep 10

echo "" >> "$LOG"
echo "=== Texture/Material related messages ===" >> "$LOG"
grep -i -E 'texture|material|mtl|map_kd|unable to find' /tmp/gzserver_texture.log >> "$LOG" 2>&1

echo "Log saved to $LOG"
echo "Last 50 lines:"
tail -50 "$LOG"
