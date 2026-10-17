#!/bin/bash
# 等整栈重启落 DONE，并把进度与尾部一起吐出来（不要在管道上猜过滤式，先落盘再取）。
set +e
LOG=/mnt/d/desktop1/9.26.1/_diag155/restart182.log
for K in $(seq 1 26); do
  sleep 20
  N=$(wc -l < $LOG 2>/dev/null)
  printf '[%03ds] 日志行数 %s  DONE=%s\n' "$((K * 20))" "${N:-0}" "$(grep -ac DONE $LOG 2>/dev/null)"
  grep -aq 'DONE' $LOG && break
done
echo '--- 尾部 40 行 ---'
tail -40 $LOG 2>/dev/null | tr -d '\r' | cut -c1-190
echo WAITDONE
