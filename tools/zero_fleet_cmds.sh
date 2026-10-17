#!/bin/bash
# 给车队每辆车补发一次零速 cmd_vel。
#
# 为什么必须单独有这一步：signal_drive_node 是这些 cmd_vel 的发布者，按 memory
# 「重启常驻节点必须杀光同名所有实例」杀掉它之后，差速插件里最后一条速度会一直留着
# ——车不会自己停，而是带着 0.7 m/s 的巡航速度继续无人管地跑，接下来任何测量都不可信。
#
# 用法：bash tools/zero_fleet_cmds.sh [车数]
set -e
N=${1:-5}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
for i in $(seq 1 $N); do
  # --once 要等匹配到订阅者才发，DDS 发现要几秒，所以 timeout 给足；
  # 只发一条会因 QoS/发现竞态丢，这里连发三条。
  for k in 1 2 3; do
    timeout -k 2 12 ros2 topic pub --once /car_$i/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      > /dev/null 2>&1 || true
  done
  echo "  car_$i 零速已补发"
done
