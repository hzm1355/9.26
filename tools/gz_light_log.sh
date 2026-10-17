#!/usr/bin/env bash
# 找出 gzserver 的日志并把与灯/渲染相关的行打出来。
# 目的：确认 world 里的两盏 point 补光灯在 gzserver 侧到底有没有被渲染出来。
set -e
cd /tmp
LOG=$(ls -t /tmp/*gz*.log /tmp/*server*.log /tmp/restart*.log 2>/dev/null | head -1 || true)
if [ -z "$LOG" ]; then
  echo "没找到 /tmp 下的 gzserver 日志，改用 journal/无日志"
  exit 0
fi
echo "== 日志: $LOG"
echo "-- 灯相关 --"
grep -in 'light' "$LOG" | head -40 || true
echo "-- 渲染/OGRE --"
grep -in 'ogre\|rendering\|sensor.*scene\|Failed' "$LOG" | head -40 || true
