#!/bin/bash
# 把五车摆回各自起点并重启驱动节点（当前配置速度、默认跟车距离）。
#
# 为什么长跑前要这一步：车队跑久了会在停车线前排队挤成一坨，而卡口是"一辆车一张抓拍"
# 的触发模型——后车进触发圈时画面里最完整的牌是前车的。
# 受光对照第一轮 10 张里 9 张读的是前车号，识别率数字整个被串牌主导，量不到光照。
# 亮度列不受影响，命中列必须先摆车。
#
# 顺序不能反：驱动节点一 tick 就按当前位姿 snap_to_route，先摆车才不会指错航段。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

for pid in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x_demo signal_drive_node' || true); do
  kill "$pid" 2>/dev/null || true
done
sleep 2
for pid in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true); do
  kill -9 "$pid" 2>/dev/null || true   # rclpy 把 TERM 只当 shutdown 标志
done
sleep 1
timeout -k 3 60 python3 $SIM/tools/place_cars.py 2>&1 | tr -d '\r' | sed 's/^/  place /'
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p config_file:=$CFG \
             -p path_csv:=/tmp/signal_drive_path.csv \
  > /tmp/signal_drive_sweep.log 2>&1 < /dev/null &
sleep 8
echo "[reset_fleet] drive: $(pgrep -f 'lib/v2x_demo/signal_drive_node' | tr '\n' ' ')  cruise=$(grep -m1 '^cruise_speed' $CFG)"
