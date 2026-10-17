#!/usr/bin/env bash
# 发送 Nav2 自主导航目标点
# 用法: bash send_goal.sh <x> <y> [quat_z] [quat_w]   （map 帧坐标，默认朝向 z=0 w=1 即 +X）
# 提示：目标朝向尽量设成与车头一致的四元数，避免 Ackermann 车到点后原地转向。
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
X=${1:?用法: bash send_goal.sh <x> <y> [quat_z] [quat_w]}
Y=${2:?缺 y}
Z=${3:-0.0}
W=${4:-1.0}
echo "[send_goal] 目标点 map($X, $Y) z=$Z w=$W，下发 NavigateToPose ..."
timeout 240 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $Z, w: $W}}}}"
echo "[send_goal] 动作返回（goal 状态见上）。"
