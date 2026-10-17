#!/usr/bin/env bash
# 只读：六车位姿 + 是否在西腿/车道上 + ETC 闸角 + /etc/heartbeat 是否在跳
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
for C in car_1 car_2 car_3 car_4 car_5 car_6 car_7; do
  printf "%-7s %s\n" "$C" "$(timeout -k 3 12 python3 $SIM/tools/pose155.py $C 2>&1 | grep -a 当前位姿 | head -1)"
done
echo "--- 道闸状态 ---"
timeout -k 3 12 ros2 topic echo /etc/gate/etc_gate_1/state --no-daemon --once 2>&1 | grep -aE 'angle_rad|occupied|fault|target' | head
echo "--- 心跳（8 s 采样）---"
timeout -k 3 8 ros2 topic echo /etc/heartbeat --no-daemon 2>&1 | head -4
echo "--- signal_drive 最近 60 s 复位次数 ---"
awk -v now=$(date +%s) '$0 ~ /复位/ {print}' /tmp/signal_drive_fix301.log | tail -4 | cut -c1-140
echo "--- 台账现量 ---"
for f in etc_identify etc_pass etc_actions; do printf "%s=%s " $f $(wc -l < /tmp/v2x_etc/$f.jsonl 2>/dev/null||echo 0); done; echo
echo "DONE"
