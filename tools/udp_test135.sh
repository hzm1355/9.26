#!/bin/bash
# 假设：新起的 ROS 进程发不到 gzserver，是 Fast DDS 的共享内存/Data-Sharing 通道坏了
# （WSL2 上 /dev/shm 满或段残留会这样；现象正是"老节点之间一切正常、新发布者发消息
# 对方一个都收不到，可 DDS 发现却显示配对成功"）。
# 这里只做两件事：1) 看 /dev/shm 与 fastrtps 段；2) 用 UDPv4-only 的发一个反向 ω，
# 同时用默认传输的观察者量实测角速度（观察者和发布者分进程，避免观察者也被换成 UDP
# 之后收不到 gzserver 的 /model_states，那样就分不清是通路坏还是观测坏）。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1

echo '== /dev/shm 用量与 fastrtps 段 =='
df -h /dev/shm | tail -1
ls /dev/shm 2>/dev/null | head -20
echo "  段文件数=$(ls /dev/shm 2>/dev/null | wc -l)"

echo '== 基线 10 s（观察者，默认传输）=='
timeout -k 5 20 python3 -u tools/spin_probe_window135.py 10 2>/dev/null || \
    echo '  （没有 spin_probe_window135.py，改用 gz 采样）'

echo '== UDPv4-only 反向 ω=+3.0 发 20 s，同时观察 =='
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 timeout -k 2 25 ros2 topic pub -r 10 /car_6/cmd_vel \
  geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}' \
  >/tmp/_udp135.txt 2>&1 &
UP=$!
sleep 2
timeout -k 5 20 python3 -u tools/spin_probe_window135.py 16 2>/dev/null || \
    bash -c 'for i in 1 2 3 4 5 6; do printf "%s " "$(date +%H:%M:%S)"; timeout 6 gz model -m car_6 -p | head -1; sleep 2; done'
wait $UP 2>/dev/null
echo '== 停发后再观察 10 s =='
timeout -k 5 20 python3 -u tools/spin_probe_window135.py 10 2>/dev/null || \
    bash -c 'for i in 1 2 3; do printf "%s " "$(date +%H:%M:%S)"; timeout 6 gz model -m car_6 -p | head -1; sleep 3; done'
echo DONE
