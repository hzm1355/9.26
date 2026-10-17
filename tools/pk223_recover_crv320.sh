#!/bin/bash
# 曲率闸 + 顶住死区两处改动的第一轮活物理验收。
#
# 前提事实（都是查出来的，不是假设的）：restart_clean_stack.sh 第 6 步 start_signal.sh 起的
# traffic_signal_full.launch.py 里 enable_signal_drive 默认 true，所以**驱动节点已经跟着整栈
# 起来了**（116 s 时查是 1 个实例、/tmp/signal.log 365 行、复位/出界/离道/重取/脱困全 0）。
# 因此这一版不再自己起驱动 —— 双实例会抢同一个 cmd_vel，历史踩过。计数按"日志增量"算，
# 基线取开跑时的行数，只统计新增那部分，免得把重启前那段也计进来。
#
# 读数一律走 ROS 侧（/model_states、cmd_vel 订阅），不再用 `timeout gz model` CLI：
# pk220 的 gzserver 就是在连着六发 gz CLI 全空回的那一步被 SIGABRT 打死的
# （boost shared_ptr<gazebo::transport::Connection> 空指针）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
LOG=/tmp/signal.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM" || exit 1

cnt() { grep -ac "$1" "$LOG" 2>/dev/null; true; }

echo "############ H0) 世代与被测对象 ############"
INST=~/v2x_ws/install/v2x_demo/lib/v2x_demo/signal_drive_node.py
echo "  gzserver=$(pgrep -cx gzserver; true) 驱动实例=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1，本脚本不起第二个）"
echo "  被测 md5=$(md5sum "$INST" 2>/dev/null | cut -c1-32)"
echo -n "  两处改动在位："
echo -n "曲率闸=$(grep -c 'self.max_ang \* la / (2.0' "$INST" 2>/dev/null; true) "
echo "顶住第二条=$(grep -c 'and not ag.stopped and ag.vmag < 0.05' "$INST" 2>/dev/null; true)"
for P in oob_x oob_y lane_guard turn_lead max_angular; do
  echo "    $P = $(timeout -k 2 12 ros2 param get /signal_drive_node $P 2>&1 | tr -d '\r' | tail -1)"
done
BASE=$(wc -l < "$LOG" 2>/dev/null; true)
echo "  日志基线行=$BASE，此刻累计 复位=$(cnt 复位) 出界=$(cnt 出界) 离道=$(cnt 离道) 重取=$(cnt 重取航段) 脱困=$(cnt 倒车脱困)"
timeout -k 5 40 python3 tools/pk222_poses320.py 4 | sed 's/^/  /'

echo
echo "############ H1) 150 s 增量计数（对照：pk214 原值 68/122s、pk217 33/150s、pk220 46/150s）############"
ELAP=0
for W in 30 30 30 30 30; do
  sleep "$W"; ELAP=$((ELAP + W))
  NOW=$(wc -l < "$LOG" 2>/dev/null; true)
  NEW=$((NOW - BASE))
  echo "  +${ELAP}s：新增 $NEW 行，复位 $(tail -n +$((BASE + 1)) "$LOG" 2>/dev/null | grep -ac 复位; true)，出界 $(tail -n +$((BASE + 1)) "$LOG" 2>/dev/null | grep -ac 出界; true)，离道 $(tail -n +$((BASE + 1)) "$LOG" 2>/dev/null | grep -ac 离道; true)，重取 $(tail -n +$((BASE + 1)) "$LOG" 2>/dev/null | grep -ac 重取航段; true)，脱困 $(tail -n +$((BASE + 1)) "$LOG" 2>/dev/null | grep -ac 倒车脱困; true)"
done
echo '  ## 新增日志尾 12 行'
tail -12 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ H2) 进角限速的直接证据（指令 × 离最近角点）############"
timeout -k 5 90 python3 tools/pk221_cmdcorner320.py 45 "$CFG" car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ H3) 第三方在道 / 里程 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
timeout -k 5 75 python3 tools/pk203_track320.py 30 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ H4) 分态：顶住 还是 diff_drive 实例卡死（能列到轮关节就一起量）############"
timeout -k 2 15 ros2 topic list 2>&1 | tr -d '\r' | grep -iE "wheel|joint|link_states" | head -10 | sed 's/^/    /'
timeout -k 5 40 python3 tools/pk222_poses320.py 4 | sed 's/^/  /'

echo
echo "############ H5) 终态 ############"
NOW2=$(wc -l < "$LOG" 2>/dev/null; true)
echo "  日志 $BASE -> $NOW2；累计 复位=$(cnt 复位) 出界=$(cnt 出界) 离道=$(cnt 离道)"
echo "  gzserver=$(pgrep -cx gzserver; true) 驱动=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)"
echo 'PK223DONE'
exit 0
