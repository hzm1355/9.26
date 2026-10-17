#!/usr/bin/env bash
# car_7 上路的收尾两段（= tools/onboard_car7_272.sh 的 G/H 段，单独拿出来跑）。
#
# 为什么这次不重跑整个 onboard：B 段重建、C 段停链路、D 段摆位都已经做过且回读通过，
# 而 D 段这次会走进一条与之前不同的分支 —— car_7 此刻**已经存在**，place_cars 会真的去
# 摆它（之前它不存在，只打印一行"失败"把槽位占住）。从 (-21.8,-16.8) 一步瞬移到
# (-21.8,-6.5) 是"摆位弹射"的老配方（记忆「摆位弹射与复位风暴」），拿它去换一次已经
# 验证过的重建不值得。所以这里只做剩下的两步：交棒 + 实测位移。
#
# 两道判据都是"效果复核"口径，不接受配置回读：
#   G：四个节点实例数必须正好 1/1/1/1（多实例会抢方向盘，这是重启纪律里最容易漏的）；
#   H：9 帧 /model_states 实测 y 的首末净位移 >= 4 m。允许被红灯合法停住，所以取整窗
#      净位移而不是"每秒都在动"；原始读数逐帧打出来，不替它编。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
NODES="traffic_light_manager adaptive_timing_node traffic_light_recorder signal_drive_node"
LOG=/tmp/drive_car7_292.log
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash

say '===== car_7 交棒与位移复核 ====='
say '--- 交棒前 car_7 的位姿（应当还在 (-21.80,-16.8) 台面上）---'
timeout -k 5 25 python3 $SIM/tools/pose155.py car_7 2>&1 | grep -a '当前位姿\|FAIL' | sed 's/^/  /' | tee -a "$LOG"

say '--- G 拉起整条信号灯链路（start_signal.sh 自己的口径）---'
bash $SIM/tools/start_signal.sh > /tmp/start_signal_292.txt 2>&1
RC=$?
say "  start_signal.sh 退出码 $RC"
tail -16 /tmp/start_signal_292.txt | cut -c1-170 | sed 's/^/    /' | tee -a "$LOG"
sleep 12
FAIL=0
for N in $NODES; do
  R=$(pgrep -cf "lib/v2x[_]demo/${N}" || true)
  say "  ${N} 实例 ${R:-0}（必须正好 1）"
  [ "${R:-0}" = "1" ] || FAIL=1
done
say '  /tmp/signal.log 里 car_7 的装载与入线行（原样）：'
grep -aE 'car_7|入线|装载|ERROR' /tmp/signal.log | tail -12 | cut -c1-170 | sed 's/^/    /' | tee -a "$LOG"

say '--- H 90 s 里按 /model_states 实测 car_7 有没有沿西腿北行 ---'
: > /tmp/car7_trail292.txt
for I in $(seq 1 15); do
  timeout -k 5 20 python3 $SIM/tools/pose155.py car_7 >> /tmp/car7_trail292.txt 2>&1
  sleep 4
done
grep -a '当前位姿' /tmp/car7_trail292.txt | sed 's/^/  /' | tee -a "$LOG"
NS=$(grep -ac '当前位姿' /tmp/car7_trail292.txt)
say "  采样到 $NS 帧（少于 3 帧判据不成立）"
[ "$NS" -ge 3 ] || FAIL=1
grep -a '当前位姿' /tmp/car7_trail292.txt \
  | awk '{for(i=1;i<=NF;i++) if ($i~/^y=/) {sub(/^y=/,"",$i); print $i}}' > /tmp/car7_y292.txt
FIRST=$(head -1 /tmp/car7_y292.txt); LAST=$(tail -1 /tmp/car7_y292.txt)
say "  首末 y：${FIRST:-无} -> ${LAST:-无}（西腿北行 = y 增大）"
OK=$(awk -v a="${FIRST:-nan}" -v b="${LAST:-nan}" 'BEGIN{print (b-a)>=4.0 ? "OK" : "NO"}')
say "  判据：整窗净位移 >= 4 m → $OK"
[ "$OK" = "OK" ] || FAIL=1
say "  逐帧有没有倒退（倒退=被顶住或抢方向盘）：单调上升的次数 / 总间隔"
awk '{if (NR>1) {n++; if ($1>p) up++} p=$1} END{printf "    %d/%d 帧在前进\n", up+0, n+0}' \
  /tmp/car7_y292.txt | tee -a "$LOG"

say '--- /etc/query 的 car_7 基线（还没走到识别点时 matched=0 是正常的）---'
timeout -k 3 25 ros2 service call /etc/query v2x_msgs/srv/EtcQuery \
  "{plate_number: '', vehicle_id: 'car_7', obu_id: '', gate_id: '', pass_id: '', result: 0, \
    t0: 0.0, t1: 0.0, limit: 5, newest_first: true, include_identify: true}" \
  > /tmp/car7_query292.txt 2>&1
say "  $(grep -aoE 'success=[A-Za-z]+|matched=[0-9-]+|returned=[0-9-]+' /tmp/car7_query292.txt | tr '\n' ' ')"

GS=$(pgrep -x gzserver | head -1)
say "  现场 gzserver pid=${GS:-无}（还在）"
if [ "$FAIL" = "0" ]; then
  say 'DRIVE_CAR7_OK：链路已交棒，car_7 实测沿西腿北行且有净位移'
else
  say 'DRIVE_CAR7_PARTIAL：见上面逐帧读数，先定位是哪一段没接上'
fi
exit $FAIL
