#!/bin/bash
# 查 user_camera 那几个话题谁是发布方、谁是订阅方 —— 决定"能不能用命令把镜头挪到 car_6 上方"。
#
# 为什么要单独成文件：上一条内联 for 循环里的 $T 又被 Git Bash 吃掉了（memory「bash -lc 里
# 不要嵌 $()/$VAR」），三次都打印成空话题名。
set +e
for T in \
  /gazebo/v2x_city_scene/user_camera/pose \
  /gazebo/v2x_city_scene/user_camera/joy_pose \
  /gazebo/v2x_city_scene/user_camera/joy_twist \
  /gazebo/v2x_city_scene/gzclient_camera/cmd \
  /gazebo/v2x_city_scene/pose/modify
do
  echo "===== $T ====="
  gz topic -i -t "$T" 2>&1 | tr -d '\r'
done
