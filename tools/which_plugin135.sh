#!/bin/bash
# 只读体检：car_6 的 cmd_vel 订阅者到底是不是 libgazebo_ros_diff_drive.so。
# 为什么查这个：反向 ω=+3.0 连发 14 s、零速连发 14 s，实测角速度四段全是 -2.3 rad/s，
# 说明"发给 /car_6/cmd_vel 的东西没被这条订阅处理"。如果订阅者是 v2x 自己的插件，
# 那"插件残值/摆位污染"这一整套结论都指向错了对象。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID

echo '== 1) /car_6/cmd_vel 的订阅端点（含类型）=='
timeout -k 3 20 ros2 topic info /car_6/cmd_vel -v 2>&1 | tr -d '\r' | head -25

echo '== 2) gzserver 里加载了哪些 v2x 插件 =='
strings /proc/27983/maps 2>/dev/null | head -1
grep -ao '/[^ ]*\.so' /proc/27983/maps 2>/dev/null | sort -u | grep -aE 'v2x|diff_drive' | head -20

echo '== 3) 运行期 world 里 car_6 的插件原文（含 filename）=='
awk '/<model name="car_6">/,/<\/model>/' \
  /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world \
  | grep -n 'plugin\|<left_joint>\|<right_joint>\|odometrySource\|legacyMode' 

echo '== 4) 本仓库有没有自己的差速插件源码 =='
grep -rln 'cmd_vel' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_gazebo_plugins 2>/dev/null | head
echo DONE
