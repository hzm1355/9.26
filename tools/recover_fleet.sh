#!/bin/bash
# 车队现场恢复：杀掉驱动 → 逐辆"先等净空、摆完验停稳" → 重启驱动 → 跑 110 s 后用
# fleet_peek 复核每辆车确实沿着自己车道前进（驱动活着 ≠ 车在动）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[recover] 1) 停掉车队驱动"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal_drive_node' || true); do
  kill "$PID" 2>/dev/null || true
  sleep 1
  kill -9 "$PID" 2>/dev/null || true
done
sleep 1
echo "  残留实例 $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)"

echo "[recover] 2) 逐辆摆位（先递归挪开堵路车 + 摆后验停稳）"
cd $SIM/tools
timeout -k 10 900 python3 -u recover_fleet.py || echo "  !! 有车未恢复干净（见上面异常清单）"

echo "[recover] 3) 重启车队驱动"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p config_file:=$CFG \
             -p path_csv:=/tmp/signal_drive_path.csv \
  > /tmp/signal_drive_sweep.log 2>&1 < /dev/null &
sleep 8
echo "  drive: $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' | tr '\n' ' ')"

echo "[recover] 4) 让车队跑 110 s 再复核前进情况"
sleep 110
timeout -k 5 90 python3 -u fleet_peek.py 20
