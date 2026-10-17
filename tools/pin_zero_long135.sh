#!/bin/bash
# 上一轮只发了 6 s 零速就没停住 —— 怀疑那 6 s 全花在 DDS 发现上（发布者刚建好就退出，
# 订阅端还没看到它）。这轮改成"边发边采样"：零速流持续 45 s，期间每 4 s 读一次位姿，
# 看 yaw 什么时候不再推进。判据用效果（yaw 停止累积），不看 ros2 的回显。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID

echo '== 开始持续零速流（后台）=='
timeout -k 2 45 ros2 topic pub -r 10 /car_6/cmd_vel \
  geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  >/tmp/_zero135.txt 2>&1 &
PUB=$!
sleep 1
echo '== 采样 12 次（每次间隔 ~3 s，gz 子进程本身约 1 s）=='
for i in $(seq 1 12); do
    printf '%s ' "$(date +%H:%M:%S)"
    timeout 8 gz model -m car_6 -p 2>&1 | head -1
    sleep 2
done
wait $PUB 2>/dev/null
echo '== 流结束后再采 4 次（看零速是不是真的被"保持"住了）=='
for i in 1 2 3 4; do
    printf '%s ' "$(date +%H:%M:%S)"
    timeout 8 gz model -m car_6 -p 2>&1 | head -1
    sleep 3
done
echo '== 发布者计数（应为 0，且这只能说明"没有新指令"，不能说明车停）=='
timeout -k 2 15 ros2 topic info /car_6/cmd_vel 2>&1 | grep -a count
echo DONE
