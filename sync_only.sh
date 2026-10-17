#!/usr/bin/env bash
# 仅同步 launch 源码到 WSL 工作区并清 CRLF，校验关键行
set -e
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
DST=~/v2x_ws/src/v2x_demo
cp "$SRC/launch/v2x_bringup.launch.py" "$DST/launch/v2x_bringup.launch.py"
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r$//' {} +
INST=~/v2x_ws/install/v2x_demo/share/v2x_demo/launch/v2x_bringup.launch.py
echo "TimerAction 出现次数: $(grep -c TimerAction "$INST")"
echo "base_footprint_broadcaster 出现次数: $(grep -c base_footprint_broadcaster "$INST")"
echo "delayed_nodes 出现次数: $(grep -c delayed_nodes "$INST")"
echo "语法检查:"
python3 -c "import ast; ast.parse(open('$INST').read()); print('OK: launch 文件语法正确')"
