#!/bin/bash
# 「先让我看到 car_6」：一张 GUI 全屏截图 + 服务端位姿，两头同时对。
#
# 为什么要把截图和位姿放在一起出：客户端丢模型这个毛病（delete+spawn 会让 gzclient 把
# 该模型从场景里摘掉，服务端却一直在）单看任何一头都会判错 —— 只看 /model_states 会说
# "在"，只看画面会说"没有"。所以先截图，再用 gz CLI 独立复读一次位姿，两者对不上才是
# 客户端掉了。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
OUT=/mnt/d/desktop1/9.26.1/_live142/xroot145.png
export DISPLAY=:1
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "--- 1) 服务端：car_6 在不在、在哪 ---"
echo "  $(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)"

echo "--- 2) 客户端：截 VcXsrv 根窗口 ---"
python3 $TOOLS/xshot142.py "$OUT" 2>&1 | sed 's/^/  /'

echo "--- 3) gzclient 进程与 X 连接 ---"
pgrep -af "gz[_]client" | cut -c1-120 | sed 's/^/  /'
ss -tn 2>/dev/null | grep -c ':6001' | sed 's/^/  连到 :6001 的 tcp 连接数：/'
