#!/bin/bash
# 盯 car_3 的进场周期：每 10 s 落一行"位姿 + 两闸角度 + 账本行数 + 节点日志尾"。
# 只读，不改任何状态。用法：bash tools/pk_watch308.sh [轮数] [模型名]
set +e
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash
N=${1:-12}
M=${2:-car_3}
PK=/tmp/v2x_parking
for i in $(seq 1 "$N"); do
  T=$(date +%H:%M:%S)
  P=$(timeout 6 gz model -m "$M" -p 2>&1 | tr -d '\n' | cut -c1-46)
  G=$(timeout 6 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 \
      | tr -d '\r' | grep -aE '^gate_id|^angle_rad|^state|^loop_occupied' | tr '\n' ' ' | cut -c1-120)
  NP=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
  NA=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
  echo "[$T] $M 位姿=$P"
  echo "      闸机回包: $G"
  echo "      账本: 通行 $NP 行 / 动作 $NA 行"
  tail -1 /tmp/shuttle_$M.log 2>/dev/null | cut -c1-160 | sed 's/^/      shuttle| /'
  tail -1 /tmp/parking_manager_node.log 2>/dev/null | cut -c1-160 | sed 's/^/      mgr   | /'
  sleep 10
done
echo "WATCH308DONE"
