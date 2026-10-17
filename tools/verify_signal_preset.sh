#!/bin/bash
# 条目3 复核：预设切换（手动优先）+ 自适应系数
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

detail() {
  timeout 4 ros2 topic echo --once /traffic_light/J1_ns/state_detail 2>/dev/null \
    | grep -E 'green_duration|yellow_duration|red_duration' | tr -d ' ' | tr '\n' ' '
  echo
}

for p in peak offpeak night; do
  ros2 topic pub --once /traffic_light/preset std_msgs/msg/String "{data: $p}" > /dev/null
  sleep 3
  echo "preset=$p -> $(detail)"
done

echo "--- J2_ns adaptive_factor / stats 采样（8s 超时）---"
for k in 1 2 3; do
  F=$(timeout 8 ros2 topic echo --once /traffic_light/J2_ns/adaptive_factor std_msgs/msg/String 2>/dev/null | grep '^data:' | head -1)
  Q=$(timeout 8 ros2 topic echo --once /traffic_light/J2_ns/stats 2>/dev/null | grep -E '^queue_length:' | head -1)
  T=$(timeout 8 ros2 topic echo --once /traffic_light/J2_ns/stats 2>/dev/null | grep -E '^throughput:' | head -1)
  echo "  sample$k: $F | $Q | $T"
  sleep 2
done
echo "--- adaptive_stats 全文 ---"
timeout 8 ros2 topic echo --once /traffic_light/adaptive_stats std_msgs/msg/String 2>/dev/null | grep '^data:' | head -2
