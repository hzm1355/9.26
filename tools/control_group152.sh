#!/bin/bash
# 对照组实验：同一段时间里看 car_2 / car_4 / car_6 各自动没动。
#
# 为什么要有对照组：car_6 "cmd_vel 有效而位姿不变"这个读数，既可以解释成"这一辆车
# 卡了"，也可以解释成"gzserver 的物理/位姿流整体僵了"。两者处置完全不同（前者要
# 脱困/重 spawn，后者要重启仿真），所以先拿两辆 Ring B 在跑的车当对照。
#
# 写成文件而不是 wsl bash -c 里带 $c：Git Bash 会把 `\$c` 吃掉，实测发过去的参数
# 变成空串，脚本拿着 '/'+' '+''+'/cmd_vel' 去建订阅，报的还是一句看不出根因的
# InvalidTopicNameException。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export ROS_DOMAIN_ID=0
tr -d '\r' < /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/pose_roll152.py > /tmp/pr.py
for c in car_2 car_4 car_6; do
  echo "--- $c ---"
  timeout -k 4 25 python3 /tmp/pr.py "$c" 12 2>&1 | tr -d '\r'
done
echo DONE
