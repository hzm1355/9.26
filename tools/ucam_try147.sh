#!/bin/bash
# 第二次尝试挪镜头：试 user_camera/joy_pose 和 joy_twist。
#
# 上一步发 /user_camera/pose 没反应（那是客户端自己发布出去的状态话题，不是输入口）。
# gz topic -l 里还挂着 joy_pose / joy_twist 两个，名字上像是"外部驱动镜头"的输入路。
# 每一步都只截图看效果，不做任何模型/物理改动；全部命令带 timeout，避免像上一发那样挂住。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
export DISPLAY=172.26.32.1:1

shot() {
  timeout -k 3 40 python3 $TOOLS/xshot142.py "$1" 2>&1 | tr -d '\r' | sed 's/^/  /'
}

echo "--- 1) joy_pose：发一个「车南侧俯视」的位姿 ---"
python3 $TOOLS/ucam_pose146.py 12.0 -24.0 14.5 12.0 -12.36 6.7 /tmp/ucam.pose 2>&1 | sed 's/^/  /'
timeout -k 3 20 gz topic -p /gazebo/v2x_city_scene/user_camera/joy_pose \
  -m gz_msgs.Pose -f /tmp/ucam.pose 2>&1 | tr -d '\r' | head -5 | sed 's/^/  /'
sleep 3
shot /mnt/d/desktop1/9.26.1/_live142/ucam_joy_pose.png

echo "--- 2) joy_twist：给一个前进速度，看镜头会不会飞 ---"
printf 'linear: {x: 2.0, y: 0.0, z: 0.0}\nangular: {x: 0.0, y: 0.0, z: 0.0}\n' > /tmp/ucam.twist
timeout -k 3 20 gz topic -p /gazebo/v2x_city_scene/user_camera/joy_twist \
  -m gz_msgs.Twist -f /tmp/ucam.twist 2>&1 | tr -d '\r' | head -5 | sed 's/^/  /'
sleep 3
shot /mnt/d/desktop1/9.26.1/_live142/ucam_joy_twist.png
echo "--- 完成 ---"
