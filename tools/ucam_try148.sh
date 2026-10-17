#!/bin/bash
# 第三次尝试挪镜头 —— 这次是**发对了**：`gz topic -p` 的 `-m` 是"消息内容"，不是类型名。
#
# 前两发都白发的原因就在这儿（报错原文）：
#   Error parsing text-format gazebo.msgs.Pose: 1:8: Message type "gazebo.msgs.Pose"
#   has no field named "gz_msgs"
# 也就是 CLI 把 `-m gz_msgs.Pose` 当成消息文本去解析了；类型它自己会从话题上认
# （joy_pose→gazebo.msgs.Pose，joy_twist→gazebo.msgs.Joystick，报错里已经写出来了）。
# 所以只给 `-f 文件` 就行。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
export DISPLAY=172.26.32.1:1

echo "--- 1) car_6 位姿（当镜头目标）---"
P=$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)
echo "  $P"
X=$(echo "$P" | awk '{print $1}')
Y=$(echo "$P" | awk '{print $2}')
Z=$(echo "$P" | awk '{print $3}')

echo "--- 2) 算相机位姿：站在车南侧 7 m、上方 4.6 m 俯视 ---"
python3 $TOOLS/ucam_pose146.py 12.0 -19.5 11.5 "$X" "$Y" "$Z" /tmp/ucam.pose 2>&1 | sed 's/^/  /'

echo "--- 3) 发到 joy_pose（只给 -f，不给 -m）---"
timeout -k 3 25 gz topic -p /gazebo/v2x_city_scene/user_camera/joy_pose \
  -f /tmp/ucam.pose 2>&1 | tr -d '\r' | head -6 | sed 's/^/  /'
sleep 4

echo "--- 4) 截图看效果 ---"
timeout -k 3 40 python3 $TOOLS/xshot142.py /mnt/d/desktop1/9.26.1/_live142/ucam_joy148.png 2>&1 |
  tr -d '\r' | sed 's/^/  /'
