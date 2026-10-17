#!/usr/bin/env bash
# ============================================================================
# 隔离实验：respawn 车到裸地 (15,-15)（无路面 box，纯 ground_plane）
#   裸地行驶正常 -> 问题锁定路面 box 的碰撞体；
#   裸地也异常   -> 问题在车模型/物理 HEAD。
# 用法: bash isolate_probe.sh <x> <y> <yaw_deg> [v] [t]
# ============================================================================
X=${1:-15}; Y=${2:--15}; YAW=${3:-90}; V=${4:-0.2}; T=${5:-8}
SHARE=~/v2x_ws/install/v2x_demo/share/v2x_demo
export GAZEBO_MASTER_URI=http://localhost:11345
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

pose() {
  timeout 3 ros2 topic echo /odom --once --field pose.pose 2>/dev/null \
    | grep -E 'x:|y:|z:|w:' | tr -d ' ' | tr '\n' ' '
}

echo "[iso] 删旧车..."
gz model -m v2x_prius -d 2>/dev/null
sleep 1
echo "[iso] respawn 到 ($X,$Y) yaw=${YAW}deg..."
YR=$(echo "$YAW * 3.14159 / 180" | bc -l)
timeout 30 ros2 run gazebo_ros spawn_entity.py -entity v2x_prius \
  -file "$SHARE/models/v2x_prius/model.sdf" -x "$X" -y "$Y" -z 0.4 -Y "$YR" > /tmp/spawn.log 2>&1
sleep 4
echo "[iso] 落地姿态: $(pose)"

echo "[iso] 驱动 v=$V t=${T}s..."
timeout "$T" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $V}, angular: {z: 0.0}}" > /dev/null 2>&1
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1
sleep 2
echo "[iso] 终点姿态: $(pose)"
D=$(timeout 3 ros2 topic echo /distance --once --field data 2>/dev/null | tr -d ' ')
echo "[iso] 插件里程 distance: $D"
echo "[iso] done（对比 distance 与车体位移即得空转率）"
