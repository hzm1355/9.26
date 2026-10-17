#!/usr/bin/env bash
# 拉取 gazebo_ros_pkgs 各分支/标签的 ackermann 插件源码并对比转向计算关键行
for ref in 3.9.0 humble; do
  curl -sL --max-time 25 "https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/${ref}/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp" -o "/tmp/ack_${ref}.cpp"
  echo "== ref=${ref} size=$(wc -c < /tmp/ack_${ref}.cpp 2>/dev/null)"
done
echo '---- 3.9.0 关键行 ----'
grep -n 'steering_diff\|tanSteer\|copysign\|target_rot\|atan\|clamp' /tmp/ack_3.9.0.cpp 2>/dev/null
echo '---- humble 关键行 ----'
grep -n 'steering_diff\|tanSteer\|copysign\|target_rot\|atan\|clamp' /tmp/ack_humble.cpp 2>/dev/null
