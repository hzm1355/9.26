#!/bin/bash
# 把 GUI 镜头挪到 car_6 上方，让它在画面里看得见。
#
# 为什么走 user_camera/pose：`gz camera -f` 在这套环境是空操作（见 ucam_pose146.py 顶部
# 注释），而 WSL 里没有 xdotool/wmctrl，模拟鼠标也不可行。gz topic -l 里确实有
# /gazebo/v2x_city_scene/user_camera/pose，所以先 echo 一次看这个方向有没有人发，再试着发。
# 这一步只动镜头，不改任何模型、不动物理，最坏情况是没反应。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
OUT=/mnt/d/desktop1/9.26.1/_live142/ucam_after.png
export DISPLAY=172.26.32.1:1

echo "--- 1) car_6 现在在哪 ---"
P=$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)
echo "  $P"
CX=$(echo "$P" | awk '{print $1}')
CY=$(echo "$P" | awk '{print $2}')
CZ=$(echo "$P" | awk '{print $3}')

echo "--- 2) 反向确认：user_camera/pose 上现在有没有人在发（发的是客户端自己的镜头位姿）---"
timeout -k 3 12 gz topic -e -t /gazebo/v2x_city_scene/user_camera/pose -d 10 2>&1 |
  tr -d '\r' | head -14 | sed 's/^/  /'

echo "--- 3) 算一个「站在车南侧 12 m、上方 8 m、俯视它」的相机位姿 ---"
python3 $TOOLS/ucam_pose146.py 12.0 -24.0 14.5 "$CX" "$CY" "$CZ" /tmp/ucam.pose 2>&1 | sed 's/^/  /'

echo "--- 4) 发出去（先试 gz_msgs.Pose，不行再试 gz.msgs.Pose）---"
gz topic -p /gazebo/v2x_city_scene/user_camera/pose -m gz_msgs.Pose -f /tmp/ucam.pose 2>&1 |
  tr -d '\r' | sed 's/^/  /'
sleep 3
python3 $TOOLS/xshot142.py "$OUT" 2>&1 | sed 's/^/  /'
echo "--- 5) 复读一次相机位姿，看有没有被客户端接走 ---"
timeout -k 3 12 gz topic -e -t /gazebo/v2x_city_scene/user_camera/pose -d 10 2>&1 |
  tr -d '\r' | head -14 | sed 's/^/  /'
