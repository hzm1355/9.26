#!/bin/bash
# 等条目 4 的驱动脚本收尾（打印 DONE），然后把驱动日志与成片体检一次性吐回来。
# 为什么要等而不是隔几分钟去瞄一眼：条目 4 的内容最坏 ~2100 s，中途读日志只会拿到半截。
LOG=/tmp/pk4_driver.log
OUT="/mnt/d/desktop1/9.26.1/视频取证/任务五_停车场闸机/条目4_管控逻辑_三类放行拒绝与手动故障.mp4"
for i in $(seq 1 300); do
  grep -q DONE "$LOG" 2>/dev/null && break
  sleep 12
done
echo "--- 驱动日志尾 25 行 ---"
tail -25 "$LOG" | cut -c1-190
echo "--- 是否还卡在等通行 ---"
grep -ac '超时\|timeout\|Traceback' "$LOG"
echo "--- 成片 ---"
ls -l --time-style=+%H:%M:%S "$OUT" 2>&1 | tail -1
echo "--- 字幕总行数与段头 ---"
grep -ac '^\[T' /tmp/vid_caption.txt
grep -a '== ' /tmp/vid_caption.txt | tail -8 | cut -c1-165
