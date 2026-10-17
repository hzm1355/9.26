#!/bin/bash
# 摆位生效性自检：杀驱动 → park 车辆到候选避让位 → 回读偏差 → 再等 5 s 看它会不会自己漂走。
# 用法：bash park_test.sh [车名:x,y ...]（不传参数用南环车道默认位）
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
for PID in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true); do kill "$PID" 2>/dev/null || true; done
sleep 2
for PID in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true); do kill -9 "$PID" 2>/dev/null || true; done
echo "drive 残留 $(pgrep -cf 'lib/v2x[_]demo/signal[_]drive[_]node' || true)"
if [ $# -ge 1 ]; then
  SPECS="$*"
else
  # 南环西行车道 y=-14.6：沙盘里少数的平直沥青面，离北环走廊 y=4.6 有 19 m
  SPECS="car_1:-15.0,-14.6 car_3:-8.0,-14.6 car_4:0.0,-14.6 car_5:10.0,-14.6"
fi
# shellcheck disable=SC2086
python3 -u $D/tools/park_and_verify.py $SPECS 2>&1 | cut -c1-160
