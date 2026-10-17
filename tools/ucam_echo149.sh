#!/bin/bash
# 读 gzclient 自己发布的 user_camera/pose —— 拿到它才能把世界坐标投影成屏幕像素。
#
# 为什么改走投影而不是继续折腾镜头：三种 CLI 手段（gz camera -f、发 user_camera/pose、
# 发 joy_pose）全都没让画面动一下，而 WSL 里没有 xdotool 可以模拟鼠标滚轮。既然挪不动
# 镜头，就反过来：用客户端报出来的相机位姿把 car_6 投影到当前这张截图上，直接圈给用户看。
set +e
export DISPLAY=172.26.32.1:1
echo "--- user_camera/pose（最多等 8 s）---"
timeout -k 3 20 gz topic -e -t /gazebo/v2x_city_scene/user_camera/pose -d 8 2>&1 |
  tr -d '\r' | head -20
echo "--- gzclient_camera/cmd（顺便看一眼，这是传感器相机的控制口）---"
timeout -k 3 20 gz topic -e -t /gazebo/v2x_city_scene/gzclient_camera/cmd -d 6 2>&1 |
  tr -d '\r' | head -12
