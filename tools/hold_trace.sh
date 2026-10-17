#!/bin/bash
# 诊断包装：先断掉车队驱动（否则车是被开着的，轨迹没法判），跑 hold_trace.py，
# 完事再 reset_fleet.sh 恢复车队。用法：
#   bash hold_trace.sh --car car_4 --x 2 --y -14.6 --hold 1
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal_drive_node' || true); do
  echo "杀掉车队驱动 pid $PID"
  kill "$PID" 2>/dev/null || true
  sleep 1
  kill -9 "$PID" 2>/dev/null || true
done
sleep 1
echo "车队驱动残留：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)"

cd "$SIM/tools"
timeout -k 5 120 python3 -u "${SCRIPT:-hold_trace.py}" "$@" || echo "!! ${SCRIPT:-hold_trace.py} 退出码非 0"

bash "$SIM/tools/reset_fleet.sh"
