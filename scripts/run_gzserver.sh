#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=/home/hzm/v2x_ws/src/smart_car_sim/models:/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models:$GAZEBO_MODEL_PATH

gzserver --verbose \
  /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/room_layout.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  2>&1 | tee /tmp/gzserver3.log
