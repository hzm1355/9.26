#!/bin/bash
# 回放取证：两帧路口画面 + 对应灯态文字
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

snap() {
  tag=$1
  ns=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  nsp=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  ew=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  j2=$(timeout 2 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  echo "${tag}: J1_ns=${ns} ped=${nsp} J1_ew=${ew} J2_ns=${j2}" > "/tmp/replay_state_${tag}.txt"
  curl -s -o "/mnt/d/desktop1/9.26.1/cam_replay_${tag}.png" "http://localhost:8080/cam/inter"
}

snap A
sleep 7
snap B
cat /tmp/replay_state_A.txt /tmp/replay_state_B.txt
