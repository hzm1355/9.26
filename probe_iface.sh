#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
for i in gazebo_msgs/srv/SpawnEntity gazebo_msgs/srv/SetEntityState gazebo_msgs/msg/EntityState gazebo_msgs/msg/ModelState; do
  echo "=== $i ==="
  ros2 interface show "$i" 2>/dev/null
done
echo "IFACE_DONE"
