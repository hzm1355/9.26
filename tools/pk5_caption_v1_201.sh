#!/bin/bash
# 抢救条目 5 v1（作废片）的字幕流文本层：/tmp/vid_caption.txt 是单一文件，
# 重录开录时 vid_start 会先轮转再清零，但轮转名只带时刻、不带条目号，
# 所以趁现在还在 /tmp 里，先按证据目录的命名固化一份。
set +e
SRC=/tmp/vid_caption.txt
E=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/evidence/任务五_视频取证字幕
DST="$E/条目5_字幕流_v1作废片_对账器4条FAIL原样.txt"
[ -s "$SRC" ] || { echo '!! 源字幕文件为空，停止'; exit 1; }
grep -aq '5h 在环全量对账器' "$SRC" || { echo '!! 这份字幕不含 5h 段，不是条目 5 的，停止'; exit 1; }
cp "$SRC" "$DST"
echo -n '已归档：'; wc -lc "$DST" | tr -d '\r'
echo '--- 片内 4 条 ✗ 原文（这就是作废的原因）---'
grep -a '✗' "$DST" | cut -c1-220
echo '--- 片内汇总行 ---'
grep -a '通过判据' "$DST" | cut -c1-220
echo CAP5V1_SAVED
