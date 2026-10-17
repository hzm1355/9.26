#!/usr/bin/env bash
# 一次性：先确认 signal_drive_node 已被杀干净，再"刹轮+摆位"恢复五车现场，
# 最后 dump 一次位姿校验落点。跑之前不要启动驱动节点（零速由 place_cars 自己发）。
set -e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

LEFT=$(pgrep -c -f 'signal_drive_node' || true)
echo "驱动节点实例数=${LEFT}（应为 0，非 0 说明还有僵尸实例在抢 cmd_vel）"
if [ "${LEFT}" != "0" ]; then
  echo "ABORT：先 kill -9 再跑"
  exit 1
fi

python3 tools/place_cars.py
python3 tools/dump_model_states.py
