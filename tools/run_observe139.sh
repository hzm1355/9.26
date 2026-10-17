#!/bin/bash
# 跑只读观察器 observe_car6_139.py，并顺带把 cmd_vel 的发布者数量打出来。
# 纯只读：不发指令、不调服务。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID
cd /tmp || exit 1

echo '[obs] 1) /car_6/cmd_vel 的发布者/订阅者计数（有两个发布者就是有人在跟 shuttle 抢方向盘）'
timeout -k 2 20 ros2 topic info /car_6/cmd_vel --no-daemon 2>&1 | tail -4

echo '[obs] 2) 活体观察（不发任何指令）'
SECS=${1:-12}
python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/observe_car6_139.py "$SECS"

echo '[obs] 3) 同名实例数（shuttle/manager/recorder 各应为 1）'
for N in parking_shuttle parking_manager parking_recorder plate_recognizer; do
  echo "  $N=$(pgrep -cf "v2x[_]demo $N" 2>/dev/null)"
done
exit 0
