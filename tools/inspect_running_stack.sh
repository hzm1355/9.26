#!/bin/bash
# 记录当前 Gazebo / ROS 栈的启动方式，供重启时原样复现
echo "===== gzserver / gzclient 进程 ====="
ps -eo pid,etimes,args | grep -E '[g]zserver|[g]zclient' || echo "(无)"

echo
echo "===== ROS2 节点相关 python 进程 ====="
ps -eo pid,etimes,args | grep -E '[p]ython3' | grep -E 'v2x|signal|dashboard|traffic|plate' || echo "(无)"

echo
echo "===== 启动脚本候选 ====="
ls -la /tmp/*.sh 2>/dev/null || echo "(无)"

echo
echo "===== GAZEBO 环境变量（来自 gzserver 进程） ====="
PID=$(pgrep -f gzserver | head -1)
if [ -n "$PID" ]; then
  tr '\0' '\n' < /proc/$PID/environ | grep -E '^(GAZEBO_|DISPLAY|ROS_DISTRO|AMENT_PREFIX_PATH|LP' | sed 's/AMENT_PREFIX_PATH=.*/AMENT_PREFIX_PATH=<...>/'
else
  echo "(gzserver 未运行)"
fi
