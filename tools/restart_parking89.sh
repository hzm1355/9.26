#!/bin/bash
# #89 把 parking 栈重启到磁盘上的代码。
#
# 为什么要重启：who88 抓到 /tmp/parking_manager_node.log 的写者是 PID 8796（15:18 起），
# 而 src 同步发生在 16:05 —— 进程跑的是**内存里的旧代码**，旧代码里有个每 60 s 把
# car_6 瞬移回入口等待位的 reset_car6 看门狗。磁盘上现在这三份副本（src / D 盘 /
# install 软链）都 grep 不到 reset_car6：摆渡车的车位归 shuttle 管，manager 不该搬车。
# 这个看门狗就是 _ptrace103 / _pivot102 / _selfdrive85 里"车自己跑到 3~33 m 外底朝天"
# 的元凶，也会把闸机动作日志搅脏，所以先重启再谈 #81/#82/#83。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

CFG=$SIM/src/v2x_demo/config/parking_zones.yaml

echo "=== 1) 杀之前先给 car_6 补一帧零速（旧代码可能还在发指令）==="
timeout -k 2 12 ros2 topic pub --once /car_6/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1
echo "  已补零速"

echo "=== 2) 杀光 parking manager / recorder 的所有实例（rclpy 吞 SIGTERM）==="
for PAT in 'parking_manager[_]node' 'parking_recorder[_]node' 'ros2 run v2x[_]demo parking_(manager|recorder)[_]node'; do
  for PID in $(pgrep -f "$PAT"); do
    echo "  kill $PID ($(tr '\0' ' ' < /proc/$PID/cmdline | cut -c1-70))"
    kill "$PID" 2>/dev/null
  done
done
sleep 2
for PAT in 'parking_manager[_]node' 'parking_recorder[_]node'; do
  for PID in $(pgrep -f "$PAT"); do
    echo "  kill -9 $PID"
    kill -9 "$PID" 2>/dev/null
  done
done
sleep 2
echo "  剩余 manager 实例=$(pgrep -cf 'parking_manager[_]node')（应为 0）"
echo "  剩余 recorder 实例=$(pgrep -cf 'parking_recorder[_]node')（应为 0）"

echo "=== 3) 重新启动（日志另起新文件，旧文件留着对账）==="
mv /tmp/parking_manager_node.log /tmp/parking_manager_node.log.old89 2>/dev/null
mv /tmp/parking_recorder_node.log /tmp/parking_recorder_node.log.old89 2>/dev/null
nohup bash -lc "ros2 run v2x_demo parking_manager_node.py --ros-args -p use_sim_time:=true \
  -p config_file:=$CFG >> /tmp/parking_manager_node.log 2>&1" >/dev/null 2>&1 &
sleep 6
nohup bash -lc "ros2 run v2x_demo parking_recorder_node.py --ros-args -p use_sim_time:=true \
  -p config_file:=$CFG >> /tmp/parking_recorder_node.log 2>&1" >/dev/null 2>&1 &
sleep 10

echo "=== 4) 校验实例数与就绪横幅 ==="
echo "  manager 实例=$(pgrep -cf 'install/v2x[_]demo/parking_manager[_]node')（应为 1）"
echo "  recorder 实例=$(pgrep -cf 'install/v2x[_]demo/parking_recorder[_]node')（应为 1）"
echo "  manager 日志前 12 行:"
head -12 /tmp/parking_manager_node.log 2>&1 | sed 's/^/    /'
echo "  recorder 日志前 6 行:"
head -6 /tmp/parking_recorder_node.log 2>&1 | sed 's/^/    /'

echo "=== 5) 新代码不该再有复位看门狗：等 75 s 看日志里有没有'复位'字样 ==="
sleep 75
echo "  含'复位'的行数=$(grep -ac '复位' /tmp/parking_manager_node.log 2>/dev/null)"
grep -a '复位' /tmp/parking_manager_node.log 2>/dev/null | head -5 | sed 's/^/    /'
gz model -m car_6 -p 2>&1 | head -1 | sed 's/^/  car_6 现在在: /'
timeout -k 2 12 ros2 topic info /car_6/cmd_vel 2>&1 | grep -aE 'Publisher|Subscription' | sed 's/^/  /'
