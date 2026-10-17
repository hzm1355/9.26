#!/bin/bash
# 260：place_and_settle155.sh 的**按模型**版本 —— 把指定车辆摆回点位并过"起步实测速度闸"。
#
# 为什么这一轮需要它：car_3 在 (11.95,-13.06) 被 car_2（排队位 12.00,-14.615、车头朝北）
# 顶住 —— 体检实测"给反向 v=-0.30 六秒，轮子转到 0.20 rad/s 而车身只走 0.001 m"，
# 按 memory「不动的三态定性」这是**被顶住**而不是"驱动实例没出力"，所以不需要重 spawn，
# 需要的是把两台车分开。分开必须走摆位，而摆位会瞬移根链接 → 攒弹射，所以沿用 155 轮
# 定下来的一体化流程：杀 shuttle → 刹轮 → 六分量摆位 → 静止体检 → 试探油门看峰值速度
# → 过闸后再精摆一次。
#
# 与 155 版的差别只有"模型名从参数来"，判据阈值一字不动（0.40 m/s 上限、静止体检必须先过）。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
M=${1:-car_3}
PX=${2:-12.00}
PY=${3:--12.80}
PYAW=${4:-1.5708}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "--- 1) 杀 shuttle（只杀它；manager/recorder 不动，保住内存环形记录）---"
for PID in $(pgrep -f "shuttle_$M"); do
  echo "  杀 pid $PID"
  kill -9 "$PID" 2>/dev/null
done
sleep 2
echo "  剩余 shuttle 实例：$(pgrep -cf "shuttle_$M")"

echo "--- 2) 摆位前刹轮 1.5 s ---"
timeout -k 5 25 python3 $TOOLS/hold_zero141.py $M 1.5 2>&1 | tail -2 | sed 's/^/  /'

echo "--- 3) 摆位前现场 ---"
gz model -m $M -p 2>&1 | tr -d '\r' | head -1 | sed 's/^/  /'

echo "--- 4) 六分量摆位到 ($PX,$PY,6.70) yaw=$PYAW（roll/pitch 归零）---"
gz model -m $M -x "$PX" -y "$PY" -z 6.70 -R 0 -P 0 -Y "$PYAW" 2>&1 | tr -d '\r' | sed 's/^/  /'
B=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "  摆后立即：$B"
sleep 4
C=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "  4 s 后：  $C"
python3 $TOOLS/pose_delta144.py "$B" "$C"
RC=$?
echo "  静止体检退出码 $RC"
if [ $RC -ne 0 ]; then
  echo "FAIL 静止体检就没过，别往下做激励测试"
  exit $RC
fi

echo "--- 5) 起步实测速度闸：刹轮 8 s → v=0.10 试探 4 s → 峰值上限 0.40 m/s ---"
timeout -k 8 90 python3 -u $TOOLS/launch_guard155.py $M 8 0.10 4 0.40 2>&1 | tr -d '\r' | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "  速度闸退出码 $RC（0 通过 / 2 残余误差还在放 / 3 给油不动）"
if [ $RC -ne 0 ]; then
  echo "FAIL 没通过速度闸，车保持在当前位姿、shuttle 保持不启动，请人工决定下一步"
  exit $RC
fi

echo "--- 6) 过了闸：把试探推走的距离补回来，重新精摆到 ($PX,$PY) ---"
timeout -k 5 25 python3 $TOOLS/hold_zero141.py $M 1.5 2>&1 | tail -1 | sed 's/^/  /'
gz model -m $M -x "$PX" -y "$PY" -z 6.70 -R 0 -P 0 -Y "$PYAW" 2>&1 | tr -d '\r' | sed 's/^/  /'
D=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "  复摆后：$D"
sleep 5
E=$(gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
echo "  5 s 后：$E"
python3 $TOOLS/pose_delta144.py "$D" "$E"
RC=$?
echo "  收尾体检退出码 $RC"
if [ $RC -ne 0 ]; then
  echo "FAIL 复摆后又开始动 —— 说明有别的驱动者在发指令（不是 shuttle，它实例数=0）"
  exit $RC
fi

echo "--- 7) 净空复核：这台车与周围实体的间距（摆位后必须真的分开）---"
timeout -k 5 60 python3 -u $TOOLS/nearby_live.py $M 3.0 2>&1 | tail -6 | sed 's/^/  /'
echo "SETTLE_OK $M ($PX,$PY,$PYAW)"
exit 0
