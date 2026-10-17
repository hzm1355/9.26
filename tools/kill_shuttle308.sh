#!/bin/bash
# 杀掉 parking_shuttle_node 的同名**所有**实例（rclpy 接管 SIGTERM，主线程不在 spin 里的
# 常驻节点 TERM 杀不掉，必须补 KILL），然后给每辆摆渡车补发一次零速 —— 旧实例最后一拍的
# 巡航速度会一直作用到差速轮的默认 1 s 超时之后（这套底盘的 cmd_vel_timeout 实测不生效，
# 静默等于保持上一条）。
# 用法：bash tools/kill_shuttle308.sh [模型名...]（默认 car_3 car_2）
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash

MODELS="${*:-car_3 car_2}"
for PID in $(pgrep -f "lib/v2x[_]demo/parking_shuttle_node"); do
  echo "杀掉 shuttle 实例 pid $PID"
  kill "$PID" 2>/dev/null
  sleep 1
  kill -9 "$PID" 2>/dev/null
done
sleep 1
N=$(pgrep -cf "lib/v2x[_]demo/parking_shuttle_node")
echo "杀后 shuttle 实例数：${N:-0}（应为 0）"

for M in $MODELS; do
  timeout -k 3 15 ros2 topic pub --once "/$M/cmd_vel" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
  echo "  已给 $M 补发零速"
done
