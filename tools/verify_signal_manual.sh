#!/bin/bash
# 条目2 手动配时 + 条目3 预设/自适应 验证
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

detail() {
  timeout 4 ros2 topic echo --once /traffic_light/J1_ns/state_detail 2>/dev/null \
    | grep -E 'green_duration|yellow_duration|red_duration' | tr -d ' ' | tr '\n' ' '
  echo
}

echo "== 当前（夜间方案自动）=="; detail

echo "== 手动配时 green=8 yellow=2 red=8 =="
ros2 param set /traffic_light_manager green_duration 8.0 > /dev/null
ros2 param set /traffic_light_manager yellow_duration 2.0 > /dev/null
ros2 param set /traffic_light_manager red_duration 8.0 > /dev/null
sleep 2; detail

for p in peak offpeak night; do
  echo "== 切换预设 $p =="
  ros2 topic pub --once /traffic_light/preset std_msgs/msg/String "{data: $p}" > /dev/null
  sleep 2; detail
done

echo "== 自适应：门槛降到 1 辆车并启动巡线车 =="
ros2 param set /adaptive_timing_node min_vehicles_for_adapt 1 > /dev/null
pkill -f '^python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py' 2>/dev/null
sleep 1
setsid nohup python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py > /tmp/loop.log 2>&1 &
sleep 30
echo "--- J2_ns adaptive_factor 采样 ---"
for k in 1 2 3 4; do
  F=$(timeout 3 ros2 topic echo --once /traffic_light/J2_ns/adaptive_factor std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  Q=$(timeout 3 ros2 topic echo --once /traffic_light/J2_ns/stats 2>/dev/null | grep '^queue_length:' | awk '{print $2}')
  echo "  sample$k: factor=$F queue=$Q"
  sleep 3
done
echo "--- adaptive_stats ---"
timeout 4 ros2 topic echo --once /traffic_light/adaptive_stats std_msgs/msg/String 2>/dev/null | grep '^data:' | head -2
