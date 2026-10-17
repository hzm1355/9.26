#!/usr/bin/env bash
# 同步修改后的 SDF/Nav2 参数到 install，并 respawn prius（解除转向节锁死吸收态）。
# 背景：转向节一旦被压死在 ±0.6 限位，车速趋零后轮胎 cone_model 剪切自锁，
# PID 900 N·m 回正力也拉不回，必须删除模型重新生成才能恢复。
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
SHARE=~/v2x_ws/install/v2x_demo/share/v2x_demo

cp -f "$SRC/models/v2x_prius/model.sdf" "$SHARE/models/v2x_prius/model.sdf"
cp -f "$SRC/config/v2x_nav2_params.yaml" "$SHARE/config/v2x_nav2_params.yaml"
echo "[respawn] SDF + nav2 params 已同步到 install"

export GAZEBO_MASTER_URI=http://localhost:11345
gz model -m v2x_prius -d
sleep 1
echo "[respawn] 旧车已删除"

source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
timeout 30 ros2 run gazebo_ros spawn_entity.py -entity v2x_prius \
  -file "$SHARE/models/v2x_prius/model.sdf" -x 0.0 -y -15.0 -z 0.4 -Y 3.14159
sleep 3
echo "[respawn] 话题检查："
ros2 topic list 2>/dev/null | grep -E '^/(odom|cmd_vel|scan|imu)' | head -5
