#!/bin/bash
# 把闸机抓拍文件的时间戳、manager 的判定行、以及 car_6 贴牌状态对齐到一条时间轴上。
# 为什么要这一步：gate_in-13 的实拍画面里车体上是一块空白灰框（没有字符），
# 而 gate_out-13 同一轮能读出 沪A12345。两种解释的修法完全相反：
#  ① gate_in-13 早于重 spawn（那时无牌）→ 只是取证帧旧了，放宽门槛就够；
#  ② gate_in 看到的是车尾/车头另一面，牌只贴了一面 → 要么补贴另一面，要么挪相机。
# 靠 mtime 排序 + manager 日志里的 sim 时间戳就能分开这两种。
set +e
echo "=== 抓拍文件 mtime（按时间升序，全部）==="
for D in /tmp/v2x_parking_in /tmp/v2x_parking_out; do
  echo "## $D"
  ls -1 --time-style=+%m-%d_%H:%M:%S -l $D/*.png 2>/dev/null | awk '{print "  ",$6,$7,$9,$5}'
done

echo "=== manager 日志里的判定/抓拍行（尾 60）==="
grep -aE 'gate_in|gate_out' /tmp/parking_manager_node.log | tail -60 | cut -c1-190

echo "=== 识别器日志（plate_rec.log 尾 30）==="
tail -30 /tmp/plate_rec.log 2>/dev/null | cut -c1-190

echo "=== 贴牌插件日志里最后一次给 car_6 贴牌的时间 ==="
grep -aE 'car_6|plate' /tmp/*.log 2>/dev/null | grep -aiE 'SetMaterial|贴牌|texture|plate_gen' | tail -12 | cut -c1-190

echo "=== car_6 当前位姿 ==="
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/v2x_ws/install/setup.bash 2>/dev/null
gz model -m car_6 -p 2>&1 | head -2
exit 0
