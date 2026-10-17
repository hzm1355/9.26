#!/bin/bash
# 录制回放验证：停实时栈 -> 起 player -> load+play -> 采样复现的灯态序列
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 1) 停止实时信号栈 ==="
pkill -f '/install/v2x_demo/lib/v2x_demo/' || true
pkill -f 'ros2 launch v2x_demo traffic_signal_full' || true
sleep 3
pgrep -af 'traffic_light' | head -n 5 || echo "(实时节点已全部退出)"

echo "=== 2) 启动回放器 ==="
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=/tmp/tl_log.json > /tmp/player.log 2>&1 &
sleep 6
cat /tmp/player.log

echo "=== 3) 载入日志 ==="
ros2 service call /traffic_light_player/load std_srvs/srv/Trigger

echo "=== 4) 开始回放 ==="
ros2 service call /traffic_light_player/play std_srvs/srv/SetBool "{data: true}"

echo "=== 5) 采样回放中的灯态序列（每 1s 采一次，共 14 次）==="
for i in $(seq 1 14); do
  ns=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  nsp=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  ew=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  j2=$(timeout 2 ros2 topic echo --once /traffic_light/J2_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  printf "t=%2ds  J1_ns=%-7s ped=%-10s J1_ew=%-7s J2_ns=%-7s\n" "$i" "${ns:-?}" "${nsp:-?}" "${ew:-?}" "${j2:-?}"
done

echo "=== 6) 回放器状态 ==="
tail -n 8 /tmp/player.log
