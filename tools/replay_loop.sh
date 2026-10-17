#!/bin/bash
# 循环回放模式：让 Gazebo 灯球跟随回放时序持续变化（用于截图取证）
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

pkill -f 'traffic_light_recorder.py player' || true
sleep 2
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=/tmp/tl_log.json -p loop:=true > /tmp/player.log 2>&1 &
sleep 6
ros2 service call /traffic_light_player/load std_srvs/srv/Trigger
ros2 service call /traffic_light_player/play std_srvs/srv/Trigger
sleep 4
echo "=== 回放中灯态 ==="
for i in $(seq 1 4); do
  ns=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  nsp=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  ew=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  j2=$(timeout 2 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  printf "sample %d  J1_ns=%-7s ped=%-10s J1_ew=%-7s J2_ns=%-7s\n" "$i" "${ns:-?}" "${nsp:-?}" "${ew:-?}" "${j2:-?}"
done
echo "=== player log ==="
tail -n 5 /tmp/player.log
