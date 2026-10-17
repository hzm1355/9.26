#!/bin/bash
# 停掉车队驱动 → 同一点位跑两次自由滚动判别（车头朝 +x / 朝 +y）。
# 用法：bash free_roll_probe.sh [车名] [x] [y] [z]
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[probe] 1) 停掉车队驱动（它会抢 cmd_vel）"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal_drive_node' || true); do
  kill "$PID" 2>/dev/null || true; sleep 1; kill -9 "$PID" 2>/dev/null || true
done
sleep 1
echo "  残留 drive 实例 $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)"

cd "$SIM/tools"
CAR=${1:-car_3}; X=${2:--0.8}; Y=${3:--11.6}; Z=${4:-6.70}
for YAW in 0 1.5708; do
  echo
  echo "[probe] ===== yaw=$YAW ====="
  timeout -k 5 200 python3 -u free_roll_probe.py --car "$CAR" --x "$X" --y "$Y" --z "$Z" --yaw "$YAW" \
    || echo "!! yaw=$YAW 退出码非 0"
done
