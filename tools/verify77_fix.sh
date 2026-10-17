#!/bin/bash
# #77 复位两段式（先刹轮→落位→静默）上线后的 520 s 五车同录验收。
# 不能用 set -u（ROS setup.bash 里有未定义变量）。
#
# 判据口径（改前基线，全部来自 _tr77_before/ + _resetscan77.txt）：
#   瞬移 35 次、真复位 7 次、复位后峰值实测 8.53 m/s（指令上限 0.70）、
#   car_2 被冲到 x=37.2 出界 —— 这就是"车穿过建筑物"的成因。
set -e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
D=/mnt/d/desktop1/9.26.1
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$WS"

n() { [ -f "$1" ] && wc -l < "$1" || echo 0; }

echo '[v77] 0) 前置检查：驱动必须只有 1 个实例'
echo "  实例：$(pgrep -f 'lib/v2x[_]demo/signal_drive_node' | tr '\n' ' ')"
C=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
[ "${C:-0}" = "1" ] || { echo "!! 实例数=${C}，不是 1，本轮作废"; exit 1; }

echo '[v77] 1) 基线产物计数（JSONL 是追加的，只能用增量当本轮证据）'
A0=$(n /tmp/v2x_records/speed_alerts.jsonl)
C0=$(n /tmp/v2x_records/speed_captures.jsonl)
P0=$(n /tmp/v2x_records/plate_records.jsonl)
G0=$(ls /tmp/v2x_gates 2>/dev/null | wc -l)
echo "  alerts=$A0 captures=$C0 plates=$P0 gate_frames=$G0"

echo '[v77] 2) 五车同录 520 s（指令 vs 实测）'
mkdir -p "$D/_tr77c"
python3 -u tools/cmd_pose_trace.py --seconds=520 --outdir="$D/_tr77c" > "$D/_cmdtrace_77c.log" 2>&1
wc -l "$D"/_tr77c/cmd_pose_car_*.csv

echo '[v77] 3) 驱动自带 20 Hz 轨迹归档'
cp /tmp/signal_drive_path.csv "$D/_path_77c.csv"
wc -l "$D/_path_77c.csv"
tail -n 1 "$D/_path_77c.csv"

echo '[v77] 4) 复位弹射体检（改前：35 次瞬移、峰值 8.53 m/s）'
python3 -u tools/trace_reset_scan.py --dir="$D/_tr77c" > "$D/_resetscan77c.txt" 2>&1 || true
tail -n 60 "$D/_resetscan77c.txt"

echo '[v77] 5) 速度上限越界（改前：car_2 冲到 8.53）'
for f in "$D"/_tr77c/cmd_pose_car_*.csv; do
  echo "--- $(basename "$f")"
  python3 -u tools/cmd_pose_report.py "$f" --vceil=0.70 --wceil=0.45 \
    --vlimit=0.90 --wlimit=0.55 --min_run=4 2>&1 | tail -n 12 || true
done

echo '[v77] 6) 建筑/设施侵入审计（阈值 1.0 m）'
python3 -u tools/analyze_deviation.py 1.0 "$D/_path_77c.csv" /tmp/world_collisions.json \
  > "$D/_dev_77c.txt" 2>&1 || true
tail -n 40 "$D/_dev_77c.txt"

echo '[v77] 7) 信号灯合规审计'
python3 -u tools/analyze_path.py "$D/_path_77c.csv" > "$D/_sig_77c.txt" 2>&1 || true
tail -n 40 "$D/_sig_77c.txt"

echo '[v77] 8) 产物增量'
A1=$(n /tmp/v2x_records/speed_alerts.jsonl)
C1=$(n /tmp/v2x_records/speed_captures.jsonl)
P1=$(n /tmp/v2x_records/plate_records.jsonl)
G1=$(ls /tmp/v2x_gates 2>/dev/null | wc -l)
echo "  超速告警 +$((A1-A0))  测速抓拍 +$((C1-C0))  车牌记录 +$((P1-P0))  卡口帧 +$((G1-G0))"
echo '[v77] DONE'
