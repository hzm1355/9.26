#!/bin/bash
# 任务二 条目5 取证：全流程录制 -> 回放（复现信号灯时序 + 车辆通行过程）
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 1) 使用已归档的录制文件（586 s，含 6 组灯态 + 5 车位姿）==='
cp -f /mnt/d/desktop1/9.26.1/tl_log_5cars.json /tmp/tl_log_replay.json
python3 - <<'PY'
import json
d = json.load(open('/tmp/tl_log_replay.json'))
m = d['meta']
ev = d['events']
veh = [e for e in ev if e.get('type') == 'vehicle']
light = [e for e in ev if e.get('type') != 'vehicle']
print(f'  录制时长={m["record_duration"]:.1f}s 事件={m["event_count"]} '
      f'信号组={len(m["groups"])} 车辆={m.get("vehicles")}')
print(f'  灯态事件={len(light)} 车辆位姿样本={len(veh)}')
print(f'  灯态事件样例={light[0] if light else None}')
PY

echo '=== 2) 归档实时轨迹（与录制时间窗对齐）==='
cp -f /tmp/signal_drive_path.csv /mnt/d/desktop1/9.26.1/signal_drive_path_live.csv
ls -l /mnt/d/desktop1/9.26.1/tl_log_5cars.json /mnt/d/desktop1/9.26.1/signal_drive_path_live.csv | awk '{print "  "$5" "$9}'

echo '=== 3) 停止实时配时栈（回放期间由 player 接管灯态与车位）==='
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/stop_signal.sh
sleep 2
pgrep -f '/install/v2x_demo/lib/v2x_demo/' | wc -l | xargs -I{} echo '  剩余实时节点数={}'

echo '=== 4) 启动回放器并 load + play ==='
pkill -f 'traffic_light_recorder.py player' 2>/dev/null || true
sleep 1
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=/tmp/tl_log_replay.json -p loop:=true -p playback_speed:=2.0 \
  > /tmp/player.log 2>&1 &
sleep 7
timeout -k 3 15 ros2 service call /traffic_light_player/load std_srvs/srv/Trigger 2>&1 | tail -1
timeout -k 3 15 ros2 service call /traffic_light_player/play std_srvs/srv/Trigger 2>&1 | tail -1

echo '=== 5) 回放中的灯态时序 + 车辆位姿采样（每 6s 一次，共 6 次）==='
for i in 1 2 3 4 5 6; do
  line="  t+$((i*6))s "
  for g in J1_ns J1_ew J2_ns J3_ew; do
    v=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/$g/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
    p=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/$g/pedestrian std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
    line="$line $g=${v:-?}/${p:-?}"
  done
  echo "$line"
  timeout -k 3 20 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/pose_probe_node.py 2>/dev/null | head -1 | sed 's/^/    /'
  sleep 4
done

echo '=== 6) 回放器日志 ==='
tail -n 6 /tmp/player.log | sed 's/^/  /'
curl -s -m 4 -o /mnt/d/desktop1/9.26.1/cam_replay_inter.png http://localhost:8080/cam/inter
curl -s -m 4 -o /mnt/d/desktop1/9.26.1/cam_replay_monitor.png http://localhost:8080/cam/monitor
echo '  已抓取回放画面 cam_replay_inter.png / cam_replay_monitor.png'
