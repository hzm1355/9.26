#!/bin/bash
# 回放验证：确保 player 在运行 -> load -> play -> 采样复现灯态序列
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

if ! pgrep -f 'traffic_light_recorder.py player' > /dev/null; then
  echo "=== 启动回放器 ==="
  setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
    --ros-args -p input_file:=/tmp/tl_log.json > /tmp/player.log 2>&1 &
  sleep 6
fi
cat /tmp/player.log

echo "=== 载入日志 ==="
ros2 service call /traffic_light_player/load std_srvs/srv/Trigger

echo "=== 开始回放（~/play, Trigger）==="
ros2 service call /traffic_light_player/play std_srvs/srv/Trigger

echo "=== 采样回放中的灯态序列 ==="
for i in $(seq 1 12); do
  ns=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  nsp=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  ew=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  j2=$(timeout 2 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  printf "sample %2d  J1_ns=%-7s ped=%-10s J1_ew=%-7s J2_ns=%-7s\n" "$i" "${ns:-?}" "${nsp:-?}" "${ew:-?}" "${j2:-?}"
done

echo "=== 回放器日志 ==="
tail -n 10 /tmp/player.log
