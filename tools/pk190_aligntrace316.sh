#!/bin/bash
# pk190：把 car_3 在 exit#3 (6.80,-8.90,π) 那次"翻转 5 次不收敛"的**逐拍轨迹**采下来，
# 判它是控制自激还是被接触顶回 —— 这两种病状的处置完全相反。
#
# 已知事实（pk189 实测，不是推测）：
#   20:04:03 重 spawn 回等待位 → 速度闸 PASS（给 0.10 m/s 走出 0.378 m，峰值 0.102）
#   → 20:05:14 入口闸拍到 京B88990 置信 0.9946、8.60 s 落杆、P0026 进场闭合
#   → 跑到出场段第 4 个点时 20:06:53 报"同一航点方向误差已翻转 5 次（0.32 rad）"停驶挂起。
#   同一轮里 `没转过 / 顶住次数到上限 / 偏离本段车道 / 原地画圈` 全 0 行 —— 所以它**不是**
#   已知的"楔在设施上"形态，而是误差在 ±0.15 门限上来回跨符号。
#
# 为什么必须重起节点才能采到：挂起后节点只连发零速、不再发状态行，echo 读空是"它不说话"
# 而不是 DDS 竞态。重启会走 seat_at() 按当前位姿重选最近航点（就在 exit#3 旁边 0.14 m），
# 所以它接手的第一件事就是重试那一次转向 —— 不用重跑一整轮。
#
# 本脚本只动摆渡车节点：不碰 gzserver / gzclient / manager / recorder（它们的内存记录
# 要保持连续），不删模型、不摆位。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
WIN=${1:-200}
TRACE=/tmp/pk190_trace.txt
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 0) 前置：谁在跑 / car_3 现在在哪 ==='
echo "  gzserver=$(pgrep -x gzserver | head -1)  gzclient=$(pgrep -x gzclient | head -1)"
echo "  manager/recorder 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')/$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')（要求各 1，本脚本不动）"
echo "  car_3 现位姿：$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-62)"

echo
echo '=== 1) 留档上一轮日志，停摆渡车（rclpy 吞 TERM，一律 kill -9）==='
cp -f /tmp/shuttle_car_3.log /tmp/shuttle_car_3.log.pk189 2>/dev/null
for PID in $(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node'); do kill -9 "$PID" 2>/dev/null; done
sleep 2
N=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')
echo "  shuttle 实例 $N（要求 0）"
[ "$N" = "0" ] || { echo 'FAIL 还有实例没死，会有两个发布者抢发状态行，采样就混了'; exit 1; }
timeout -k 5 25 python3 "$SIM/tools/hold_zero141.py" car_3 1.5 2>&1 | tr -d '\r' | tail -1 | sed 's/^/  /'

echo
echo "=== 2) 起状态行采样（$WIN s，后台），再起摆渡车 ==="
rm -f /tmp/pk190_dump.log
setsid nohup python3 -u "$SIM/tools/pk190_statusdump316.py" "$WIN" "$TRACE" \
  > /tmp/pk190_dump.log 2>&1 < /dev/null &
echo $! > /tmp/pk190_dump.pid
sleep 3
echo "  采样 pid=$(cat /tmp/pk190_dump.pid) 存活=$(kill -0 $(cat /tmp/pk190_dump.pid) 2>/dev/null && echo yes || echo NO)"
SHUTTLE_MODELS=car_3 bash "$SIM/tools/start_parking_nodes.sh" parking_shuttle_node.py 2>&1 | tail -6 | sed 's/^/  /'
echo "  就绪行：$(grep -a '就绪' /tmp/shuttle_car_3.log | tail -1 | cut -c1-150)"
echo "  重新选路：$(grep -a '重新选路' /tmp/shuttle_car_3.log | tail -1 | cut -c1-160)"

echo
echo "=== 3) 等采样窗口收口（每 10 s 报一次已采行数）==="
END=$(( $(date +%s) + WIN + 60 ))
while kill -0 $(cat /tmp/pk190_dump.pid) 2>/dev/null; do
  sleep 10
  echo "  t+ 已采 $(wc -l < "$TRACE" 2>/dev/null; true) 行｜挂起=$(grep -ac '停驶挂起' /tmp/shuttle_car_3.log 2>/dev/null; true) 行｜$(date +%H:%M:%S)"
  [ "$(date +%s)" -lt "$END" ] || { echo '  超上限，强杀采样'; kill -9 $(cat /tmp/pk190_dump.pid) 2>/dev/null; break; }
done
cat /tmp/pk190_dump.log | tr -d '\r' | tail -2 | sed 's/^/  /'

echo
echo '=== 4) 逐拍判读 ==='
python3 "$SIM/tools/pk190_flipanalyze316.py" "$TRACE" 2>&1 | sed 's/^/  /'

echo
echo '=== 5) 节点侧关键词与终态 ==='
for K in '轮跑完' '本轮作废' '停驶挂起' '没转过' '顶住次数到上限' '原地画圈' '偏离本段车道' '重新选路'; do
  printf '  %-16s %s 行\n' "$K" "$(grep -ac "$K" /tmp/shuttle_car_3.log 2>/dev/null; true)"
done
echo '  ## shuttle 尾 6 行'
tail -6 /tmp/shuttle_car_3.log 2>/dev/null | cut -c1-175 | sed 's/^/    /'
echo "  ## car_3 终态：$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-62)"
echo 'PK190DONE'
exit 0
