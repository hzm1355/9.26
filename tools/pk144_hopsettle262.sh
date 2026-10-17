#!/bin/bash
# 262：把一辆车**分小跳**挪到目标点位，每跳之后刹轮放能量，最后一跳再做起步速度闸。
#
# 为什么不能一步摆过去（本轮实测）：car_3 顶在 car_2 角上被挂起后，pk142 一步摆到
# (12.00,-12.80) —— 静止体检 4 s 位移 0.00 m 判"通过"，可起步试探 v=0.10 四秒走出
# 1.834 m、峰值 1.77 m/s、还自己转了 176°，最后顶死在出口接收柱上（距柱 0.62 m，
# 车身半长 0.8）。这就是 memory「摆位弹射」记的形态：瞬移只搬根链接，轮子关节留在
# 原地，楔住期间攒下的轮位误差在"开始给油那一下"往外放。一步 3.4 m 的跳距太大，
# 所以这里按 ≤1.2 m 切三跳，每跳之后连发零速把放出来的能量当拍吃掉，末了再走闸门。
#
# 用法：bash pk144_hopsettle262.sh <model> <x> <y> <yaw>
# 退出码 0 = 到位且起步闸通过；2 = 闸没过（车停在原地、shuttle 不启动，交人工判）。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
M=${1:-car_3}
TX=${2:-14.30}
TY=${3:--14.60}
TW=${4:-1.5708}
HOP=1.2
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

CUR=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "--- 起点：$CUR"
read -r X Y Z R P W <<<"$CUR"
if [ -z "$X" ]; then
  echo "FAIL 读不到 $M 的位姿（gzserver 没起？模型名不对？）"
  exit 2
fi

D=$(python3 -c 'import math,sys; print("%.3f"%math.hypot(float(sys.argv[1])-float(sys.argv[2]), float(sys.argv[3])-float(sys.argv[4])))' "$TX" "$X" "$TY" "$Y")
N=$(python3 -c 'import math,sys; d=float(sys.argv[1]); h=float(sys.argv[2]); print("%d"%max(1,math.ceil(d/h)))' "$D" "$HOP")
echo "--- 跳距 $D m → 切 $N 跳（每跳上限 $HOP m），每跳后刹轮 6 s"

for K in $(seq 1 "$N"); do
  # 线性插值到第 K 跳的落点：第 N 跳正好落在目标点上。朝向分两半插值，避免末跳才转
  # 完 90° 时把"瞬移 + 大转角"叠在同一拍里（转角本身就是轮位误差的来源）。
  set -- $(python3 -c '
import math, sys
x, y, w = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
tx, ty, tw = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
n, k = int(sys.argv[7]), int(sys.argv[8])
f = k / float(n)
dw = (tw - w + math.pi) % (2 * math.pi) - math.pi
print("%.4f %.4f %.4f" % (x + (tx - x) * f, y + (ty - y) * f, w + dw * f))
' "$X" "$Y" "$W" "$TX" "$TY" "$TW" "$N" "$K")
  HX=$1
  HY=$2
  HW=$3
  echo "--- 第 $K/$N 跳 → ($HX,$HY) yaw=$HW"
  gz model -m $M -x "$HX" -y "$HY" -z 6.70 -R 0 -P 0 -Y "$HW" 2>&1 | tr -d '\r' | sed 's/^/  /'
  timeout -k 5 25 python3 $TOOLS/hold_zero141.py $M 6 2>&1 | tail -1 | sed 's/^/  /'
  A=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
  sleep 3
  B=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
  echo "  跳后：$A"
  echo "  3 s 后：$B"
  python3 $TOOLS/pose_delta144.py "$A" "$B"
  RC=$?
  if [ $RC -ne 0 ]; then
    echo "  !! 这一跳之后还在自己动（静止体检没过）—— 再刹 12 s 后继续下一跳"
    timeout -k 5 30 python3 $TOOLS/hold_zero141.py $M 12 2>&1 | tail -1 | sed 's/^/  /'
  fi
done

echo "--- 到位复核 + 起步实测速度闸（刹轮 8 s → v=0.10 试探 4 s → 峰值上限 0.40 m/s）"
FIN=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "  $FIN"
timeout -k 10 90 python3 -u $TOOLS/launch_guard155.py $M 8 0.10 4 0.40 2>&1 | tr -d '\r' | sed 's/^/  /'
RC=${PIPESTATUS[0]}
if [ $RC -ne 0 ]; then
  echo "HOPSETTLE_FAILED $M 闸门退出码 $RC"
  exit 2
fi
echo "--- 收尾净空：与周围实体的间距"
timeout -k 5 60 python3 -u $TOOLS/nearby_live.py $M 3.0 2>&1 | tail -6 | sed 's/^/  /'
echo HOPSETTLE_OK $M
exit 0
