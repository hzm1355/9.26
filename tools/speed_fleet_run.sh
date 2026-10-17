#!/bin/bash
# 任务四测速取证（被动跟队版）总控：预检三件事（检测节点、记录器、车队驱动），
# 然后跑 speed_fleet_evidence.py 的三个阈值阶段，最后回读跳变护栏增量。
#
# 三个预检都不能省（memory「长跑脚本先预检依赖」）：
#  * 检测节点没活着 → 一个事件都不会有，而脚本会安静地跑完 12 分钟；
#  * 记录器要 FORCE 重启 → /speed/export 导的是内存累积，不重启就把上一轮的旧行混进来；
#  * 车队驱动（signal_drive）必须活着，本方案的真值就是它的 cruise_speed=0.7 m/s。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
OUT=${1:-/tmp/v2x_records/speed_fleet_evidence.json}
LOG=/tmp/speed_fleet_evidence.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[fleet] 1) 预检：检测节点 / 记录器 / 车队驱动"
N=$(pgrep -cf 'lib/v2x[_]demo/speed_detect_node' || true)
echo "  speed_detect_node 实例 $N"
if [ "$N" != "1" ]; then
  bash $SIM/tools/start_speed_nodes.sh speed_detect_node.py
fi
FORCE=1 bash $SIM/tools/ensure_speed_recorder.sh | tail -3
D=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
echo "  signal_drive_node 实例 $D"
if [ "$D" != "1" ]; then
  echo "  车队驱动不在，先恢复车队"
  bash $SIM/tools/reset_fleet.sh
fi

BEFORE=$(grep -c '超物理上限' /tmp/speed_detect_node.log 2>/dev/null || true)
echo "[fleet] 2) 开跑（当前护栏告警累计 ${BEFORE:-0} 行；仿真时间约 3×240 s + 预检）"
cd $SIM/tools
timeout -k 5 1500 python3 -u speed_fleet_evidence.py > $LOG 2>&1 || RC=$?
RC=${RC:-0}

AFTER=$(grep -c '超物理上限' /tmp/speed_detect_node.log 2>/dev/null || true)
echo "[fleet] 3) 跳变护栏新增 $(( ${AFTER:-0} - ${BEFORE:-0} )) 行（车队正常巡航时应该接近 0）"
echo "[fleet] 4) 退出码 $RC，日志 $LOG，JSON $OUT"
tail -6 $LOG | cut -c1-160
