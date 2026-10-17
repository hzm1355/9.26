#!/bin/bash
# 车牌链路诊断：插件是否收到 /plate/set、材质是否注册、面片是否存在
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

echo "=== gzserver.log 里所有 v2x_plate 行 ==="
grep -a 'v2x_plate' /tmp/gzserver.log | tail -15 || echo "(无)"

echo
echo "=== gzserver.log 里的材质/面片报错 ==="
grep -a -E 'Unable to get Material|plate_carrier|Wrn.*Visual' /tmp/gzserver.log | tail -10 || echo "(无)"

echo
echo "=== 载体模型是否被生成 ==="
timeout -k 2 20 python3 -c "
import rclpy
from gazebo_msgs.srv import GetWorldNames, GetModelList
" 2>/dev/null || true
gz model -l 2>/dev/null | tr -d '\000' | grep -c plate_carrier || echo "gz CLI 不可用"

echo
echo "=== /plate 相关话题与订阅者 ==="
ros2 topic list 2>/dev/null | grep plate || echo "(无)"
for t in /plate/set /plate/number /plate/request; do
  echo "-- $t --"
  ros2 topic info $t 2>/dev/null || true
done

echo
echo "=== p00001 目录内容 ==="
ls -la /tmp/v2x_plates/p00001/ 2>/dev/null || echo "(无)"
cat /tmp/v2x_plates/p00001/p00001.material 2>/dev/null || true
