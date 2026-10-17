#!/usr/bin/env bash
# 只同步并重建 v2x_demo（纯 Python，秒级），然后按需重启 recorder。
# 用在"改了 plate_*.py 但没碰 C++"的场合，避免走 rebuild_all_restart.sh 的 4 分钟。
set -e -o pipefail
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

rm -rf ~/v2x_ws/src/v2x_demo
cp -r $SIM/src/v2x_demo ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r//g' {} +
cd ~/v2x_ws
colcon build --symlink-install --packages-select v2x_demo 2>&1 | tail -3

# 需要重启的节点：rclpy 会吞掉 TERM，所以先 TERM 再补 KILL。
# 匹配串刻意带上 "run v2x_demo " 和 "lib/v2x_demo/" 前缀：本脚本自己的命令行是
# "sync_demo.sh plate_recorder_node.py"，带上前缀就匹配不到自己，不会把脚本杀掉
# （这是踩过的坑，见 memory/feedback-wsl-pkill-selfkill）。
for NODE in "$@"; do
  for PAT in "run v2x_demo $NODE" "lib/v2x_demo/$NODE"; do
    for PID in $(pgrep -f "$PAT" || true); do
      echo "杀掉 $PAT (pid $PID)"
      kill "$PID" 2>/dev/null || true
      sleep 1
      kill -9 "$PID" 2>/dev/null || true
    done
  done
  setsid nohup ros2 run v2x_demo $NODE > /tmp/$(basename $NODE .py).log 2>&1 < /dev/null &
  sleep 5
  echo "  -> $(pgrep -f "lib/v2x_demo/$NODE" | head -1) $(pgrep -f "lib/v2x_demo/$NODE" >/dev/null && echo "$NODE 已起" || echo "$NODE 没起来")"
done
