#!/bin/bash
# 回放验证（含车辆）：停实时栈 -> player 循环回放 -> 采样车位+灯态+路口画面
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
source /usr/share/gazebo-11/setup.sh

pkill -f '/install/v2x_demo/lib/v2x_demo/' || true
pkill -f 'ros2 launch v2x_demo traffic_signal_full' || true
pkill -f 'traffic_light_recorder.py player' || true
sleep 3

setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=/tmp/tl_log.json -p loop:=true > /tmp/player.log 2>&1 &
sleep 6
ros2 service call /traffic_light_player/load std_srvs/srv/Trigger
ros2 service call /traffic_light_player/play std_srvs/srv/Trigger

for i in 1 2 3 4; do
  sleep 6
  pose=$(timeout 3 gz model -m car_1 -p 2>/dev/null | tr '\n' ' ')
  ns=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  nsp=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  echo "sample $i  car_1: $pose  J1_ns=${ns} ped=${nsp}" > "/tmp/replay_car_$i.txt"
  curl -s -o "/mnt/d/desktop1/9.26.1/cam_replayC_$i.png" "http://localhost:8080/cam/inter"
  cat "/tmp/replay_car_$i.txt"
done
tail -n 3 /tmp/player.log
