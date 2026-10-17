#!/bin/bash
# 活体验收两段式复位：驱动节点在跑，把一辆**正在给油巡航**的车摆到 oob_x(22) 之外，
# 逼它走"出界 → 零速刹轮 1.5 s → SetEntityState 落位 → 零速静默 1.0 s"这条链。
#
# 为什么这是可比较的对照：改前 _resetscan77.txt 里最"轻"的那几次复位，起始条件跟这次
# 一样（摆位瞬间 cmd 是 ±0.1 级、轮子在受驱），落位后 2.4 s 内实测仍冲到 1.1~1.76 m/s
# （car_3 t=2603.9、car_4 t=2584.4、car_5 t=2754.7）；楔住十几秒的则冲到 3.7~8.5。
# 所以判据取"落位后 2.4 s 内峰值 <0.90"是能分辨的，不是 0→0 假过。
# 不能用 set -u（ROS setup.bash 里有未定义变量）。
set -e
D=/mnt/d/desktop1/9.26.1
WS=$D/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$WS"

C=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
echo "驱动实例数=${C}（必须是 1，本轮靠它自己下指令）"
[ "${C:-0}" = "1" ] || { echo '!! 前置不满足，本轮作废'; exit 1; }

cp /tmp/signal_drive_fix.log "$D/_siglog77d.prefix" || true
L0=$(wc -l < /tmp/signal_drive_fix.log)
echo "驱动日志基线行数=${L0}"

mkdir -p "$D/_tr77d"
python3 -u tools/cmd_pose_trace.py --seconds=100 --outdir="$D/_tr77d" \
  > "$D/_cmdtrace_77d.log" 2>&1 &
TRACE=$!
sleep 6
python3 -u tools/wedge_trigger.py --car=car_5 --xy=23.4,1.5 --yaw=0 --observe=40 \
  > "$D/_wedge77d.txt" 2>&1
echo "[trigger] 观察结束，等同录收尾"
wait $TRACE
wc -l "$D"/_tr77d/cmd_pose_car_*.csv

echo "[trigger] 驱动日志新增行（原始，尾部 40 行）："
tail -n +$((L0 + 1)) /tmp/signal_drive_fix.log | tail -40
echo '[trigger] DONE'
