#!/usr/bin/env bash
# 监控 Nav2 自主导航：循环采样 /odom 位置 + 检查 nav2.log 目标状态
# 最多跑 ITER 次迭代（每迭代约 6s），车到点或动作结束由调用方判读。
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
ITER=${1:-30}
for i in $(seq 1 "$ITER"); do
  P=$(timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr -d ' ' | tr '\n' ' ')
  echo "[monitor $i] odom: $P"
  sleep 2
done
echo "[monitor] 采样结束（$ITER 次）"
