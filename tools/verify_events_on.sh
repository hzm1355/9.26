#!/bin/bash
# 条目4 验证（上）：通行统计 + 急救优先全绿
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

pkill -f '^python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py' 2>/dev/null
sleep 1
setsid nohup python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py > /tmp/loop.log 2>&1 &
sleep 35

echo "== 通行统计（J1_ns / J2_ns）=="
for g in J1_ns J2_ns; do
  timeout 6 ros2 topic echo --once /traffic_light/$g/stats 2>/dev/null \
    | grep -E 'vehicle_count|throughput|queue_length|cycle_count' | tr -d ' ' | tr '\n' ' '
  echo "   <- $g"
done

echo "== 急救优先：emergency ON =="
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: true}" 2>/dev/null | grep -E 'success|message'
sleep 3
for g in J1_ns J1_ew J2_ns; do
  S=$(timeout 4 ros2 topic echo --once /traffic_light/$g/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  M=$(timeout 4 ros2 topic echo --once /traffic_light/$g/state_detail 2>/dev/null | grep '^control_mode:' | awk '{print $2}')
  echo "  $g = $S ($M)"
done
