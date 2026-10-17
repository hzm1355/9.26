#!/bin/bash
# 任务四 条目4 视频取证：流量统计——测速路段车流量、平均车速、超速次数等。
#
# 这一段为什么这么排：
#   · 统计有三个口径，混着说一定被质疑，所以同框点名：窗口 flow（120 s）/ 累计 total_flow
#     （自上次点位变更起，只在 10 s tick 上快照）/ 落盘全量（JSONL 与导出 CSV）；
#   · 阈值保持基线混布（2.0 的三个点位必告、3.0/4.0 的两个点位必干净），
#     这样"超速次数"这一列既非全 0 也非全满，能看出它是按各点位阈值分别算的；
#   · 原始回包用 ros2 topic echo 打整条消息（不猜过滤式），面板与看板只是同一数据的两种呈现。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
OUT="$VID_ROOT/任务四_测速路段/条目4_流量统计_车流量均速超速次数.mp4"
CLI="python3 $TOOLS/speed_zones_cli.py"

vid_start "$OUT" 430 "$VIEW_MON,$VIEW_INTER" speed \
  "任务四 条目4  流量统计：车流量 / 平均车速 / 超速次数（窗口 + 累计）"
cap "统计口径：窗口 120 s 内的流量/车数/均速/峰/谷/超速次数/占比 + 自上次点位变更起的累计"
capq "$CLI list | head -2"
sleep 12
cap "== 4a 原始回包：/speed/stats 一条消息的全部字段（每个在册点位一条，同轮 stamp 相同）=="
capq "timeout -k 3 25 ros2 topic echo /speed/stats --once --no-daemon | tr -d '\\r' | sed -n '1,14p'"
sleep 70
cap "== 4b 面板实拍：右栏'条目4 流量统计'逐点位表，每 10 s 随一轮 stats 刷新 =="
sleep 80
cap "  流量 = 窗内过车次数（同一辆车重复通过按次计）；均速/峰/谷是这些车次的统计量"
sleep 80
cap "== 4c 界面侧：:8090 看板逐点位表（数据源 /state.json → speed.zones，与面板同源）=="
capq "python3 $TOOLS/dashboard_speed_dump.py | sed -n '1,9p'"
cap "== 4d 三口径点名：窗口 flow / 累计 total_flow / 落盘全量（JSONL 跨轮追加，只核不少于）=="
capq "wc -l /tmp/v2x_records/speed_captures.jsonl /tmp/v2x_records/speed_segments.jsonl /tmp/v2x_records/speed_alerts.jsonl | tail -4"
sleep 70
cap "== 4e 窗口已滚动：再看一轮，窗口列会随过车进出窗口变化，累计列只增不减 =="
capq "python3 $TOOLS/dashboard_speed_dump.py | sed -n '1,9p'"
vid_wait
echo DONE
