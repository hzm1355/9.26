#!/bin/bash
# 用**当前活位姿**跑 OBB 净空审计：脱困指令发了却几乎不动，先钉死"到底还有没有东西贴着车"。
set +e
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
W=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world
P=$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)
echo "  gz 位姿行: $P"
set -- $P
if [ "$#" -lt 6 ]; then
  echo "FAIL 位姿读不到 6 个数（gz 回的不是位姿行）"
  exit 1
fi
echo "  取 x=$1 y=$2 yaw=$6"
python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/car_vs_colliders.py "$W" "$1" "$2" "$6"
echo "  （只读审计：不在此处下任何 cmd_vel —— shuttle 正在发脱困指令，另开驱动者会互相污染读数）"
echo DONE
