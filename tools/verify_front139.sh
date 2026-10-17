#!/bin/bash
# 复核"前牌贴上了没有"。回读不能只看 ros2 topic echo：/plate/set 与 /plate/number
# 都是 transient_local 之外的普通话题，echo --once 在发现竞态下必然空回包（memory
# 「DDS 计数不能当即时真相」），空回包不等于没贴。真正的凭据是两处落盘：
#  ① plate_gen 节点日志里那一行"已生成 pXXXXX"；
#  ② gzserver 日志里 v2x_plate 插件的 SetMaterial 目标名（要出现 plate_front）。
set +e
echo "=== /tmp/plate_gen.log 尾 12 ==="
tail -12 /tmp/plate_gen.log
echo "=== gzserver 日志里 v2x_plate 相关（尾 15）==="
grep -a 'v2x_plate\|SetMaterial\|plate_front' /tmp/gzserver.log | tail -15
echo "=== 纹理目录里最新 3 个 pXXXXX ==="
ls -1t /tmp/v2x_plates/ 2>/dev/null | head -4
echo "=== /plate/set 最近一条（用 topic echo --once 重试 4 次）==="
for i in 1 2 3 4; do
  S=$(timeout -k 3 15 ros2 topic echo /plate/set --once --no-daemon 2>&1 | tr -d '\r' | grep -a data)
  [ -n "$S" ] && { echo "  $S"; break; }
  echo "  第 $i 次空回包"; sleep 8
done
exit 0
