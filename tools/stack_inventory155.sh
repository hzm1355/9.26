#!/bin/bash
# 只读：把"现在到底跑了哪些东西"点名列出来，作为整栈重启的清单依据。
# 之所以现在要这份清单：/reset_world 和 `gz model` 瞬移都不能清掉 diff_drive 插件实例里
# 缓存的轮速目标（155 轮实测：还原后正立静止、瞬移后仍自走 1.9 m 并自转 0.98 rad，
# 而 cmd_vel 发布者 0），按 memory「gz-diffdrive-instance-wedge」只剩重 spawn 一条路，
# 而重 spawn 要连带车牌贴附，实际可行的就是整栈重启。整栈重启前必须先弄清有几套
# 常驻节点、谁是谁的父进程，否则重启后会积累同名双实例（memory「重启节点要杀光所有实例」）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/v2x_ws/install/setup.bash 2>/dev/null

echo "--- 1) gz 进程 ---"
ps -eo pid,etimes,args 2>/dev/null | grep -a -E 'gzserve[r]|gzclien[t]' | sed 's/^/  /'

echo "--- 2) 本项目 v2x_demo / smart_car_sim 的 ros2 节点进程（按名字点名）---"
ps -eo pid,etimes,args 2>/dev/null | grep -a -E 'lib/v2x[_]demo/|lib/smart_car[_]sim/|ros2 run' \
  | grep -a -v grep | sed 's/^/  /'

echo "--- 3) 看板 / 录制 / 其它常驻 ---"
ps -eo pid,etimes,args 2>/dev/null | grep -a -E 'dashboard|vid_rec|http.server|record' \
  | grep -a -v grep | sed 's/^/  /'

echo "--- 4) 各节点的启动参数里用的哪份配置（决定重启后要重跑哪些脚本）---"
timeout -k 5 40 ros2 node list 2>&1 | tr -d '\r' | head -40 | sed 's/^/  /'

echo "--- 5) 有没有现成的整栈编排脚本 ---"
for F in $SIM/scripts/*.sh $SIM/*.sh; do
  if grep -a -q 'gzserver' "$F" 2>/dev/null; then
    echo "  $(basename $F)  <- 含 gzserver；它起的节点数：$(grep -a -c 'ros2 run\|setsid' "$F" 2>/dev/null)"
  fi
done
echo DONE
