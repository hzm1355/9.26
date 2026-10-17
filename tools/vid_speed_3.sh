#!/bin/bash
# 任务四 条目3 视频取证：可自定义限速阈值；超速时界面触发告警标记并记全五要素。
#
# 这一段为什么这么排：
#   · 用**同一批过车**做前后对照：OP_SET_GLOBAL_LIMIT 99.0 → 同样 2.52 km/h 的车必不告警，
#     压到 1.0 → 必告警。这样"告警是阈值驱动的"在同一份车流里自证，不靠两次不同的交通量；
#   · 阈值热改走 OP_SET_GLOBAL_LIMIT/OP_SET_LIMIT，两者都**不重置统计**（ADD/REMOVE/CLEAR 才重置），
#     所以面板'累计'列在两个阶段之间是连续的，能看出"告警数突增发生在阈值切换之后"；
#   · 五要素（车辆编号/车牌/瞬时速度/超速时刻/位置）分别取自话题原样行与 JSONL 落盘行，
#     界面侧再用 :8090 的 /state.json（面板渲染的同一数据源）复核一遍。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
OUT="$VID_ROOT/任务四_测速路段/条目3_自定义限速阈值_告警标记与五要素.mp4"
CLI="python3 $TOOLS/speed_zones_cli.py"
PROBE=/tmp/vid_s3_probe.txt

vid_start "$OUT" 520 "$VIEW_MON,$VIEW_INTER" speed \
  "任务四 条目3  自定义限速阈值 + 界面告警标记 + 超速记录五要素"
cap "同一批车流做对照：先把全部点位阈值抬到 99.0（必不告警），再压到 1.0（必告警）"
cap "== 3a OP_SET_GLOBAL_LIMIT 99.0：一次改全部点位，不重启节点、不重置统计 =="
capq "$CLI global --limit 99.0 | head -2"
timeout -k 3 480 python3 -u "$TOOLS/speed_probe.py" 470 $PROBE > /tmp/vid_s3_probe.log 2>&1 &
PP=$!
sleep 6
cap "  面板'阈值'列应已全部变 99.0；接下来过车的判定列都是'正常'"
sleep 105
cap "== 阶段 A（阈值 99.0）过车明细：注意判定列 =="
capq "grep -aE '^\[瞬时\]|^\[区间\]' $PROBE | tail -4 | cut -c1-118"
sleep 110
cap "== 3b OP_SET_GLOBAL_LIMIT 1.0：同一车速立刻变超速，界面打告警标记 =="
capq "$CLI global --limit 1.0 | head -2"
sleep 120
cap "  面板判定列转成'超速'（红），'告警 N' 计数与 /speed/alert 同步增长"
sleep 60
cap "== 3c 告警五要素：车辆编号 / 车牌 / 瞬时速度 / 超速时刻 / 位置 =="
capq "grep -a '告警' $PROBE | tail -4 | cut -c1-118"
cap "  同一件事的落盘形态（/tmp/v2x_records/speed_alerts.jsonl 末两行，字段与 msg 定义一致）："
capq "tail -2 /tmp/v2x_records/speed_alerts.jsonl | cut -c1-118"
cap "== 3d 界面侧复核：:8090 看板'测速路段'面板的告警区 =="
capq "python3 $TOOLS/dashboard_speed_dump.py | sed -n '/告警/,\$p' | head -7"
sleep 6
cap "== 3e 恢复各点位基线阈值（供后续条目使用）=="
capq "$CLI limit --zone SZ-A-west-instant --limit 2.0 | head -1; $CLI limit --zone SZ-B-north-instant --limit 4.0 | head -1; $CLI limit --zone SZ-B-south-instant --limit 2.0 | head -1; $CLI limit --zone SZ-A-south-segment --limit 2.0 | head -1; $CLI limit --zone SZ-B-east-segment --limit 3.0 | head -1"
capq "wait $PP; tail -1 /tmp/vid_s3_probe.log"
vid_wait
echo DONE
