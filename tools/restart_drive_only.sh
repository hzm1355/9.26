#!/bin/bash
# 只重建 + 只重启 signal_drive_node，把"穿障三护栏"上线，不动 gzserver/测速/灯控链路。
#
# 为什么不走 tools/restart_clean_stack.sh：那一发要整栈重启，会把还没验收的停车场 world
# 改动一起带进去，而且需要现场重摆五车。本次改动只有 1 个 Python 节点 + 1 份自动生成的
# 配置（config/obstacles.yaml），install/ 里节点是符号链接，同步+build+重启单节点即生效。
#
# 三条硬规矩沿用现场既有脚本：
#   1) 先删后拷 + 去 CR（D 盘写出来的是 CRLF）；
#   2) 杀同名实例要杀光（留一个就俩发布者抢 /car_*/cmd_vel）；
#   3) 杀完必须灌零速（diff_drive 插件缓存最后一条指令会继续推车到出界）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash

echo '[1/6] 同步 v2x_demo 源码'
rm -rf ~/v2x_ws/src/v2x_demo
cp -r "$SIM/src/v2x_demo" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo \( -name '*.py' -o -name '*.yaml' -o -name 'CMakeLists.txt' \) \
  -exec sed -i 's/\r//g' {} +

echo '[2/6] 归档当前 20 Hz 轨迹（重启会把 CSV 以 w 模式截断）'
if [ -f /tmp/signal_drive_path.csv ]; then
  DST=/tmp/signal_drive_path.pre_fix.csv
  if [ ! -f "$DST" ]; then
    cp /tmp/signal_drive_path.csv "$DST"
    echo "  修复前轨迹已存 $DST：$(wc -l < "$DST") 行"
  else
    echo "  $DST 已存在（$(wc -l < "$DST") 行），不覆盖"
  fi
fi

echo '[3/6] 停掉 signal_drive 的全部实例'
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
echo "  残留实例：${N:-0}"
[ "${N:-0}" = "0" ] || { echo '!! 没杀干净，停在这里（不要双实例抢发 cmd_vel）'; exit 1; }

echo '[4/6] 五车灌零速，覆盖插件里缓存的巡航指令'
for c in car_1 car_2 car_3 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
done

echo '[5/6] colcon build v2x_demo'
cd ~/v2x_ws
if colcon build --symlink-install --packages-select v2x_demo > /tmp/v2x_drive_fix_build.log 2>&1; then
  echo '  BUILD_OK'
else
  echo '!! 编译失败，原始日志尾部：'
  tail -20 /tmp/v2x_drive_fix_build.log
  exit 1
fi
source ~/v2x_ws/install/setup.bash

echo '[6/6] 重启驱动（use_sim_time 必须带上，否则 CSV 时线退回墙钟）'
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true \
             -p path_csv:=/tmp/signal_drive_path.csv \
  > /tmp/signal_drive_fix.log 2>&1 < /dev/null &
sleep 12
echo "  实例：$(pgrep -f 'lib/v2x[_]demo/signal_drive_node' | tr '\n' ' ')"
echo "  起实数：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)（必须是 1）"
echo '  节点日志尾部（原始，不过滤）：'
tail -8 /tmp/signal_drive_fix.log
echo '  新轨迹行数：'"$(wc -l < /tmp/signal_drive_path.csv)"
echo '[done] 复核请跑 tools/probe_fleet.py 与 tools/analyze_deviation.py'
