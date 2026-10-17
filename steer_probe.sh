#!/usr/bin/env bash
# 转向探针实验：respawn 解除锁死态 -> 立即以给定 (v, ω) 驱动 -> 采样转向节角度
# 用法: bash steer_probe.sh <v> <w> [t] [采样次数]
V=${1:-0.26}; W=${2:-0.3}; T=${3:-8}; N=${4:-6}
SHARE=~/v2x_ws/install/v2x_demo/share/v2x_demo
export GAZEBO_MASTER_URI=http://localhost:11345

gz model -m v2x_prius -d 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
timeout 30 ros2 run gazebo_ros spawn_entity.py -entity v2x_prius \
  -file "$SHARE/models/v2x_prius/model.sdf" -x 0.0 -y -15.0 -z 0.4 -Y 3.14159 > /tmp/spawn.log 2>&1
sleep 2
echo "[probe] respawn 完成，开始驱动 v=$V w=$W"

timeout "$T" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $V}, angular: {z: $W}}" > /dev/null 2>&1 &
PUBPID=$!
sleep 1
for i in $(seq 1 "$N"); do
  A=$(gz model -m v2x_prius -i 2>/dev/null | grep -E 'name: "v2x_prius::front_(left|right)_steer_joint"' -A2 | grep 'angle:' | tr -d ' ' | tr '\n' ' ')
  echo "[probe $i] $A"
  sleep 1
done
wait $PUBPID 2>/dev/null
# 可靠停车：--once 有 QoS 握手竞态会丢，last_cmd 残留 -> 车不停
# （实测：车被持续驱动 15m 后物理崩坏，/distance 出 inf）
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1
echo "[probe] done"
