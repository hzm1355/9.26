#!/bin/bash
# #77 定性取证：五车同录「指令 vs 实测」，抓下一个越出走廊的车（不赌是哪一辆）。
# 不能用 set -u（ROS setup.bash 里有未定义变量）。
set -e
D=/mnt/d/desktop1/9.26.1
cd "$D/v2x_traffic_sim"
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
mkdir -p "$D/_tr77"
python3 tools/cmd_pose_trace.py --seconds=520 --outdir="$D/_tr77" \
  > "$D/_cmdtrace_77b.log" 2>&1
echo "[trace] rc=$?" >> "$D/_cmdtrace_77b.log"
wc -l "$D"/_tr77/cmd_pose_car_*.csv >> "$D/_cmdtrace_77b.log" 2>&1
