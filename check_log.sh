#!/usr/bin/env bash
# 翻最近几次 ros2 launch 的 gzserver 日志，找崩溃前的真实报错
source /opt/ros/humble/setup.bash
LOGROOT=~/.ros/log
echo "===== 最近的 launch 日志目录 ====="
ls -dt $LOGROOT/*/ 2>/dev/null | head -5
echo ""
LATEST=$(ls -dt $LOGROOT/*/ 2>/dev/null | head -1)
echo "===== 最新目录: $LATEST ====="
ls -la "$LATEST"
echo ""
echo "===== 该目录内 gzserver 相关日志末尾 ====="
for f in "$LATEST"*gzserver*; do
  [ -f "$f" ] || continue
  echo "---- $f ----"
  tail -40 "$f"
done
echo ""
echo "===== launch.log 中 gzserver/died/segfault 关键行 ====="
grep -riE 'gzserver|died|segfault|signal|abort|terminate|what\(\)|OGRE|Rendering|camera' "$LATEST"launch.log 2>/dev/null | tail -30
