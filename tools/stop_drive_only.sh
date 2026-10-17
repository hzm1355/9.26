#!/bin/bash
# 只停"复位风暴的发生器"signal_drive_node，保留测速链路/管理器/看板不动。
#
# 为什么不用 tools/stop_signal.sh：它的 pkill 式 '/install/v2x_demo/lib/v2x_demo/' 会把
# speed_detect_node、speed_recorder_node、traffic_light_manager、plate_* 全杀掉，
# 而现在的目的只是掐断 /set_entity_state 复位（每次复位都带 ~8 m/s 穿透回弹，见
# speed_live_pass.py docstring 与 memory「摆位弹射」），让现场静下来。
#
# 杀完必须给五辆车灌零速：驱动是按 10 Hz 持续发巡航指令的，diff_drive 插件会缓存最后
# 一条指令继续推车，不覆盖就等于让它带着 0.7 m/s 一直直行到出界。
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '[stop-drive] 1) 杀掉 signal_drive 的全部实例（rclpy 吞 TERM，TERM 后补 KILL）'
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal[_]drive_node' || true); do
  kill "$PID" 2>/dev/null || true
done
sleep 2
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true); do
  kill -9 "$PID" 2>/dev/null || true
done
sleep 1
N=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
echo "  signal_drive 残留实例：${N:-0}"
[ "${N:-0}" = "0" ] || { echo '!! 没杀干净，停跑'; exit 1; }

echo '[stop-drive] 2) 五辆车灌零速（覆盖掉缓存的巡航指令）'
for c in car_1 car_2 car_3 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
done

echo '[stop-drive] 3) 静置 60 s 后复核：位移应 ~0，检测节点不应再冒护栏行'
sleep 60
for c in car_1 car_2 car_3 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
done
timeout -k 5 90 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/fleet_peek.py 20 2>&1 | tail -8

T0=$(date +%s)
echo "  近 60 s 护栏行数（检测节点日志时间戳 $(( T0 - 60 )) 之后）："
awk -v now="$T0" '/WARN/ {t=$2; gsub(/[][a-z:]/,"",t); if (t+0 > now-60) c++} END {print "   " c+0 " 行"}' \
  /tmp/speed_detect_node.log 2>/dev/null || echo '   读不到日志'
echo '[stop-drive] 完成'
