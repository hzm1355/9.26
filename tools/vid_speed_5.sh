#!/bin/bash
# 任务四 条目5 视频取证：测速数据记录存储，可导出仿真测试原始数据。
#
# 这一段为什么这么排：
#   · 先演"存储"再演"导出"：JSONL 是记录器跨轮追加的原始存储（每行一条完整消息），
#     中间隔一次过车再数一遍行数，证明它是活的、不是一次性写死的文件；
#   · /speed/export 的回包原文要贴出来（条数 + 超速占比 + 逐点位），这是"导出成套"的第一手证据；
#     回包只说明服务端答应了，所以导出前后各留一次 CSV mtime（+%H:%M:%S）——两刻不同才是"这次真落盘"；
#   · 最后跑 check_speed_export.py 做体检：CSV==同名 JSON、JSONL>=CSV、超速行 alert 列非空、
#     逐点位重算一致、关键列齐全 —— 别只报"文件生成了"，收尾产物要自己体检。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
OUT="$VID_ROOT/任务四_测速路段/条目5_记录存储与原始数据导出.mp4"
R=/tmp/v2x_records

vid_start "$OUT" 330 "$VIEW_MON,$VIEW_INTER" speed \
  "任务四 条目5  测速数据记录存储 + 导出仿真测试原始数据"
cap "存储：speed_recorder_node 以追加模式把三路话题逐行写成 JSONL（每行一条完整消息）"
cap "== 5a 存储侧现状 =="
capq "ls -l --time-style=+%m-%d_%H:%M $R/speed_captures.jsonl $R/speed_segments.jsonl $R/speed_alerts.jsonl"
capq "wc -l $R/speed_captures.jsonl $R/speed_segments.jsonl $R/speed_alerts.jsonl | tail -4"
cap "  末列字节数在涨 = 记录器仍在写；JSONL 行数 = 本会话累计落盘条数（与 5c 回包条数对账）"
sleep 150
cap "== 5b 期间过车后重数行数：与 5a 对比即'持续写入' =="
capq "wc -l $R/speed_captures.jsonl $R/speed_segments.jsonl $R/speed_alerts.jsonl | tail -4"
capq "tail -1 $R/speed_captures.jsonl | cut -c1-118"
cap "== 5c 调 /speed/export（std_srvs/srv/Trigger）：回包给条数、超速占比与逐点位 =="
cap "  导出前先留 CSV 的 mtime，与 5d 对比 —— 回包 + mtime 都动才算这次真落了盘"
capq "ls -l --time-style=+%H:%M:%S $R/speed_captures.csv $R/speed_alerts.csv | cut -c1-118"
# Humble 的 ros2 service call 没有 --no-daemon，加上会被 argparse 拒掉、调用根本不执行（片内留旧 CSV 就是这个坑）。
capq "timeout -k 3 25 ros2 service call /speed/export std_srvs/srv/Trigger | tr -d '\\r' | tail -3 | fold -s -w 118"
cap "== 5d 导出产物：3 张明细 CSV + 同名 JSON + 统计 CSV（utf-8-sig 中文表头）=="
capq "ls -l --time-style=+%H:%M:%S $R/speed_captures.csv $R/speed_segments.csv $R/speed_alerts.csv $R/speed_stats.csv"
capq "head -2 $R/speed_captures.csv | cut -c1-118"
capq "tail -1 $R/speed_alerts.csv | cut -c1-118"
cap "== 5e 导出体检（check_speed_export.py）：成套自洽 + alert 列 + 逐点位重算 + 关键列齐全 =="
capq "python3 $TOOLS/check_speed_export.py 2>&1 | tail -6"
vid_wait
echo DONE
