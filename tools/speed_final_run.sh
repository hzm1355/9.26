#!/usr/bin/env bash
# 跑任务四的最终取证窗口：先让**新代码**上岗（检测节点是 python，装的是符号链接，
# 文件改了但进程里还是旧的 import，必须重启），再整轮 verify_speed.sh。
# 用法：bash speed_final_run.sh [窗口秒]      日志落 /tmp/speed_verify_final2.log
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
WIN=${1:-600}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

bash $D/tools/start_speed_nodes.sh speed_detect_node.py 2>&1 | tail -4
sleep 3
bash $D/tools/ensure_speed_recorder.sh 2>&1 | tail -2
echo "[final_run] 检测节点已换新，窗口 $WIN s 开跑"
WIN=$WIN bash $D/tools/verify_speed.sh
