#!/bin/bash
# 只读点验：car_3 掉出台面之后，其它车/台面/闸机/节点是不是还健康。
# 目的：判断"重 spawn car_3 + 只重起 shuttle"够不够，还是必须整栈重启（后者要另外批准）。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
echo '=== 各车位姿（台面 z≈6.7；z≈0 就是台外地面）'
for M in car_1 car_2 car_3 car_4 car_5 car_6 car_7; do
  printf '  %-6s %s\n' "$M" "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
done
echo '=== gzserver / gzclient 存活与起量'
ps -o pid=,etime=,cmd= -p "$(pgrep -x gzserver | head -1)" 2>/dev/null | cut -c1-90 | sed 's/^/  /'
echo "  gzclient 实例：$(pgrep -cx gzclient 2>/dev/null; true)"
echo '=== 停车场三节点实例（要求各 1）'
for N in parking_manager_node parking_recorder_node parking_shuttle_node; do
  echo "  $N $(pgrep -cf "lib/v2x[_]demo/$N"; true)"
done
echo '=== 摆渡车日志尾 6 行'
tail -6 /tmp/shuttle_car_3.log 2>/dev/null | cut -c1-160 | sed 's/^/  /'
echo '=== 台账行数（JSONL 是追加全量）'
for F in /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl; do
  echo "  $F $(wc -l < "$F" 2>/dev/null; true) 行  mtime=$(stat -c %y "$F" 2>/dev/null | cut -c1-19)"
done
echo '=== 两道闸当前状态'
timeout -k 3 15 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 \
  | tr -d '\r' | grep -aE 'gate_id|state:|angle_rad|fault' | head -8 | sed 's/^/  /'
