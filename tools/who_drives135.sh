#!/bin/bash
# car_6 在持续零速流下仍以 ~0.22 rad/s 匀速转圈 → 不是 cmd_vel 在驱它。
# 找真正的驱动源：残留进程？joint 级控制话题？world 里 car_6 挂了几层插件？
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID

echo '== 1) WSL 里还在跑的 python 进程（除本脚本外都该解释得了）=='
ps -eo pid,etime,cmd | grep -a 'python' | grep -av grep | cut -c1-160

echo '== 2) 与 car_6 有关的话题及其发布者数 =='
timeout -k 3 25 ros2 topic list 2>/dev/null | grep -a 'car_6' > /tmp/_t135.txt
while read -r T; do
    [ -z "$T" ] && continue
    N=$(timeout -k 2 12 ros2 topic info "$T" 2>/dev/null | tr -d '\r' | grep -a 'Publisher count' | awk '{print $3}')
    printf '  %-34s pub=%s\n' "$T" "${N:-?}"
done < /tmp/_t135.txt

echo '== 3) 有没有节点在发 joint 力/速度（/apply_joint_* 之类）=='
timeout -k 3 25 ros2 service list 2>/dev/null | grep -aE 'car_6|apply_model|joint' | head -12

echo '== 4) odom 报的是什么速度（轮积分 → 轮子是否真在转）=='
timeout -k 3 20 ros2 topic echo /car_6/odom --no-daemon --once 2>&1 | tr -d '\r' \
  | grep -aA4 'twist:' | grep -aE 'linear|angular|x:|y:|z:' | head -8

echo '== 5) 仿真时钟与实时率 =='
timeout -k 3 15 ros2 topic echo /clock --no-daemon --once 2>&1 | tr -d '\r' | head -2
timeout -k 3 15 ros2 param get /car_6/diff_drive legacyMode 2>&1 | head -2
timeout -k 3 15 ros2 param get /car_6/diff_drive cmdTopicName 2>&1 | head -2
timeout -k 3 15 ros2 param get /car_6/diff_drive maxWheelTorque 2>&1 | head -2

echo '== 6) world 里 car_6 的插件层数 =='
grep -ac 'car_6' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world
grep -an 'libgazebo_ros_diff_drive\|car_6' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world | head -20
echo DONE
