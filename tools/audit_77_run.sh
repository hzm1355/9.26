#!/bin/bash
# #77 改路线后的实跑验收：先记基线计数，跑一个 240 s 窗口，再用同一份世界碰撞清单
# 做建筑刮擦审计 + 信号灯合规审计 + 卡口/测速产物增量。
#
# 为什么基线计数要先记：/tmp/v2x_records 下的 JSONL 是**追加**的，重启只备份不清空
# （restart_clean_stack.sh 第 0 步是 cp 不是 rm），所以"有告警"不能当本轮证据，
# 必须用增量。
set -e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
D=/mnt/d/desktop1/9.26.1
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

n() { [ -f "$1" ] && wc -l < "$1" || echo 0; }

echo '[audit] 0) 基线'
A0=$(n /tmp/v2x_records/speed_alerts.jsonl)
C0=$(n /tmp/v2x_records/speed_captures.jsonl)
P0=$(n /tmp/v2x_records/plate_records.jsonl)
G0=$(ls /tmp/v2x_gates 2>/dev/null | wc -l)
echo "  alerts=$A0 captures=$C0 plates=$P0 gate_frames=$G0"

echo '[audit] 1) 刷新世界碰撞清单（新几何）'
python3 -u $WS/tools/world_collision_inventory.py > $D/_coll_gap3.txt 2>&1
tail -n 14 $D/_coll_gap3.txt

echo '[audit] 2) 实跑窗口 240 s'
sleep 240

echo '[audit] 3) 取轨迹切片'
cp /tmp/signal_drive_path.csv $D/_path_77.csv
tail -n 1 $D/_path_77.csv

echo '[audit] 4) 建筑刮擦/横偏审计（阈值 1.0 m）'
python3 -u $WS/tools/analyze_deviation.py 1.0 $D/_path_77.csv /tmp/world_collisions.json \
  > $D/_dev_77.txt 2>&1 || true
tail -n 40 $D/_dev_77.txt

echo '[audit] 5) 信号灯合规审计'
python3 -u $WS/tools/analyze_path.py $D/_path_77.csv > $D/_sig_77.txt 2>&1 || true
tail -n 45 $D/_sig_77.txt

echo '[audit] 6) 产物增量'
A1=$(n /tmp/v2x_records/speed_alerts.jsonl)
C1=$(n /tmp/v2x_records/speed_captures.jsonl)
P1=$(n /tmp/v2x_records/plate_records.jsonl)
G1=$(ls /tmp/v2x_gates 2>/dev/null | wc -l)
echo "  超速告警 +$((A1-A0))  测速抓拍 +$((C1-C0))  车牌记录 +$((P1-P0))  卡口帧 +$((G1-G0))"
ls -lt /tmp/v2x_gates | head -6
echo '[audit] DONE'
