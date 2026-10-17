#!/bin/bash
# 条目4（下）强制绿通 + 条目5 录制回放
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "== emergency OFF，恢复定时方案 =="
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: false}" 2>/dev/null | grep message
sleep 2
S=$(timeout 4 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
M=$(timeout 4 ros2 topic echo --once /traffic_light/J1_ns/state_detail 2>/dev/null | grep '^control_mode:' | awk '{print $2}')
echo "  J1_ns=$S control=$M"

echo "== 强制绿通 forced_green ON =="
ros2 service call /traffic_light_manager/forced_green std_srvs/srv/SetBool "{data: true}" 2>/dev/null | grep message
sleep 2
A=$(timeout 4 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
B=$(timeout 4 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
echo "  J1_ns=$A J1_ew=$B（强制绿通只放行默认组）"
ros2 service call /traffic_light_manager/forced_green std_srvs/srv/SetBool "{data: false}" 2>/dev/null | grep message

echo "== 停车-起步演示（制造排队与通过事件）=="
pkill -f '^python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py' 2>/dev/null
sleep 1
setsid nohup python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/park_demo.py > /tmp/park.log 2>&1 &
for k in 1 2 3 4 5 6; do
  sleep 12
  P=$(timeout 5 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  Q=$(timeout 5 ros2 topic echo --once /traffic_light/J2_ns/stats 2>/dev/null | grep '^queue_length:' | awk '{print $2}')
  echo "  t+$((k*12))s: J2_ns=$P queue=$Q"
done
sleep 15
echo "== 演示后统计 =="
for g in J1_ns J2_ns; do
  timeout 6 ros2 topic echo --once /traffic_light/$g/stats 2>/dev/null \
    | grep -E 'vehicle_count|throughput|max_wait_time|cycle_count' | tr -d ' ' | tr '\n' ' '
  echo "   <- $g"
done

echo "== 录制保存 =="
ros2 service call /traffic_light_recorder/save std_srvs/srv/Trigger 2>/dev/null | grep -E 'success|message'
python3 - <<'PY'
import json
d = json.load(open('/tmp/tl_log.json'))
m = d['meta']
print('  meta: groups=%s events=%d duration=%.1fs' % (m.get('groups'), m.get('event_count'), m.get('record_duration')))
print('  first:', d['events'][0])
print('  last :', d['events'][-1])
PY

echo "== 回放（停掉管理器，由 player 重放灯态）=="
pkill -f '/install/v2x_demo/lib/v2x_demo/' 2>/dev/null
pkill -f 'ros2 launch v2x_demo traffic_signal_full' 2>/dev/null
sleep 2
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player --ros-args \
  -p input_file:=/tmp/tl_log.json -p use_sim_time:=true > /tmp/player.log 2>&1 &
sleep 6
ros2 service call /traffic_light_player/load std_srvs/srv/Trigger 2>/dev/null | grep message
ros2 service call /traffic_light_player/play std_srvs/srv/Trigger 2>/dev/null | grep message
for k in 1 2 3 4 5 6; do
  S=$(timeout 3 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  P=$(timeout 3 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  echo "  replay $k: J1_ns=$S ped=$P"
  sleep 2
done
pkill -f '/install/v2x_demo/lib/v2x_demo/' 2>/dev/null
sleep 1
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/start_signal.sh > /dev/null 2>&1
echo "== 回放结束，信号栈已恢复运行 =="
