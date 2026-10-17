#!/bin/bash
# Start gzserver with correct environment
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

export GAZEBO_MODEL_PATH=/home/hzm/v2x_ws/src/smart_car_sim/models:/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models:$GAZEBO_MODEL_PATH

LOG=/tmp/gzserver_texture.log
echo "Starting gzserver at $(date)" > "$LOG"
echo "GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH" >> "$LOG"
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH" >> "$LOG"

gzserver --verbose \
  /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_layout.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  >> "$LOG" 2>&1 &

echo "gzserver PID: $!"
echo "Waiting 15s for models to load..."
sleep 15

echo ""
echo "=== Texture/Material related messages ==="
grep -i -E 'texture|material|mtl|unable to find' "$LOG" | head -30

echo ""
echo "=== Last 30 lines of log ==="
tail -30 "$LOG"
