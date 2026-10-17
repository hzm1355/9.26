#!/bin/bash
# 跑 spin_decay135.py（7 分钟纯观察，判 car_6 的自转是在收敛还是死锁）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1
timeout -k 5 500 python3 -u tools/spin_decay135.py 2>&1 | tee /tmp/_sd135.txt
echo "EXIT=$?"
