#!/bin/bash
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
echo "--- 12 次采样（1s 间隔）：J1_ns / J1_ew / J2_ns / J1_ns 行人 ---"
for k in $(seq 1 12); do
  A=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  B=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  C=$(timeout 2 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  P=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  echo "t$k: J1_ns=$A J1_ew=$B J2_ns=$C ped=$P"
  sleep 1
done
echo "--- state_detail 一帧 ---"
timeout 3 ros2 topic echo --once /traffic_light/J1_ns/state_detail 2>/dev/null | head -16
echo "--- stats 一帧 ---"
timeout 3 ros2 topic echo --once /traffic_light/J1_ns/stats 2>/dev/null | head -14
