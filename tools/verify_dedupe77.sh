#!/bin/bash
# 验 cmd_pose_trace 的"同刻帧被吞"修复：先确认驱动只有一份实例，再边录边连瞬移两次。
# 用法：wsl -e bash /tmp/verify_dedupe77.sh
set -e
D=/mnt/d/desktop1/9.26.1
cd "$D/v2x_traffic_sim"
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

N=$(pgrep -cf 'install/v2x_demo/lib/v2x_demo/signal_drive_node' || true)
echo "驱动实例数=$N（必须是 1，瞬移后要由它自己发指令）"
if [ "$N" != "1" ]; then
  echo "ABORT：实例数不是 1，先跑 tools/restart_drive_only.sh"
  exit 1
fi

rm -rf "$D/_tr77e"
mkdir -p "$D/_tr77e"

python3 tools/cmd_pose_trace.py --cars=car_5 --seconds=40 --outdir="$D/_tr77e" \
  > "$D/_tr77e/trace.log" 2>&1 &
TP=$!
sleep 6
python3 tools/teleport_twice.py --car=car_5 --step=1.2 --rounds=3 \
  | tee "$D/_twice77e.txt"
wait $TP || true

echo "=== 采集脚本自报："
cat "$D/_tr77e/trace.log"
echo "=== 新表判读："
python3 tools/dedupe_check.py --jump=0.3 "$D/_tr77e/cmd_pose_car_5.csv" \
  | tee -a "$D/_twice77e.txt"
echo "=== 对照（改前同一条车、同样两次瞬移的旧表）："
python3 tools/dedupe_check.py --jump=0.3 "$D/_tr77d/cmd_pose_car_5.csv" \
  | tee -a "$D/_twice77e.txt"
