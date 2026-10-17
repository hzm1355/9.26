#!/bin/bash
# 进角限速改完之后的第一轮：静置→摆位→起驱动（零参数覆盖）→第三方验收。
#
# 与 pk217 的差别只有一处，就是被测对象换了：这轮跑的是改过的 signal_drive_node.py
#   旧：to_corner = leg_len - along（当前航段剩余里程）。换段发生在角点**前** turn_lead
#       =1.2 m，换完段车在新段上 along≈-1.2，于是 to_corner=leg_len+1.2>=2.2，限速在
#       "正要开始转"的那一帧自己失效，车带 cruise 0.7（r=1.56 m，90° 要 2.45 m 弧长）
#       冲过角点 → 楔进南侧 c80 路缘/桩列 → 离道 → 复位 → 弹射。pk217 实测形态：car_2
#       沿 x≈2 冲到 y=17、car_4 从东腿 20.8 被甩到 x=25/31.9，150 s 内复位 33 次。
#   新：to_corner = 车到"最近角点顶点"的欧氏距离（正在绕的角=本段起点，下一个角=本段
#       终点，取小），限速横跨角的两侧，换段清不掉。
#
# 上一轮为什么把摆位提到静置之前（pk216 在静置闸 exit 4）：驱动一死，车上残余弹射的
# 动能还在物理里走，而台外的车没有摩擦会一直滚；先摆回台面再静置，静置闸量的才是
# "落点自己站不站得住"，而不是"还在不在被上一轮的弹射拖着"。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
LOG=/tmp/signal_drive_cor320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM" || exit 1

po() { timeout 6 gz model -m "$1" -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64; }
zero6() {
  for c in car_1 car_2 car_3 car_4 car_5 car_6; do
    timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
  done
}

echo "############ F0) 现场留证 + 被测世代 ############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在，这轮全部读数作废'; exit 1; }
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)"
INST=~/v2x_ws/install/v2x_demo/lib/v2x_demo/signal_drive_node.py
echo "  被测文件 md5：$(md5sum "$INST" 2>/dev/null | cut -c1-32)（install 是符号链接，指向 home src）"
echo "  限速那一行的形态："
grep -n 'to_corner = ' "$INST" 2>/dev/null | cut -c1-120 | sed 's/^/    /'
grep -c 'to_corner = leg_len - along' "$INST" 2>/dev/null | sed 's/^/    旧公式残留行数：/'

echo
echo "############ F1) 杀光驱动实例 + 六台灌零速 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
zero6
echo "  驱动残留实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 0）"

echo
echo "############ F2) 摆位：车流四台回 start_idx 车道，car_3 回等待位，car_6 回出生位 ############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '  摆完 %-6s %s\n' "$M" "$(po $M)"; done

echo
echo "############ F3) 静置闸：残余弹射必须停，读数不干净就不起驱动 ############"
SETTLE=1
for TRY in 1 2 3; do
  timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk220_settle.log 2>&1
  SETTLE=$?
  sed 's/^/  /' /tmp/pk220_settle.log
  [ "$SETTLE" = "0" ] && { echo "  第 $TRY 轮静下来了"; break; }
  echo "  第 $TRY 轮还没静：再灌一次零速并静置 20 s"
  zero6
  sleep 20
done
[ "$SETTLE" = "0" ] || { echo '!! 三轮静置后仍有车在自己动，起驱动只会把弹射叠上去；停。'; exit 4; }

echo
echo "############ F4) 起驱动（零参数覆盖，跑工程树原值）+ 世代/参数效果复核 ############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV > "$LOG" 2>&1 < /dev/null &
sleep 15
NP=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)
echo "  驱动实例：${NP:-0}（必须 1，且只启动这一次）"
for P in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  运行期加载副本：cwd=$(readlink -f /proc/$P/cwd 2>/dev/null) exe=$(tr '\0' ' ' < /proc/$P/cmdline 2>/dev/null | cut -c1-90)"
done
for P in oob_x oob_y lane_guard turn_lead max_angular cruise_speed; do
  echo "    $P = $(timeout -k 2 12 ros2 param get /signal_drive_node $P 2>&1 | tr -d '\r' | tail -1)"
done

echo
echo "############ F5) 计数：复位是否还在续发（30/60/90/120/150 s）############"
ELAP=0
for W in 30 30 30 30 30; do
  sleep "$W"; ELAP=$((ELAP + W))
  echo "  起量 ${ELAP}s（$(date +%H:%M:%S)）：日志 $(wc -l < "$LOG" 2>/dev/null; true) 行，复位 $(grep -ac 复位 "$LOG" 2>/dev/null; true)，出界 $(grep -ac 出界 "$LOG" 2>/dev/null; true)，离道 $(grep -ac 离道 "$LOG" 2>/dev/null; true)，重取航段 $(grep -ac 重取航段 "$LOG" 2>/dev/null; true)"
done
echo '  ## 尾 14 行'
tail -14 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ F6) 进角限速的直接证据：指令速度与'离最近角点多近'配对 ############"
timeout -k 5 90 python3 tools/pk221_cmdcorner320.py 45 "$CFG" car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ F7) 第三方在道/里程/指令-实测差 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
timeout -k 5 75 python3 tools/pk203_track320.py 30 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'
timeout -k 5 45 python3 tools/pk206_odomgap320.py 8 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ F8) 终态 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo "  本轮零参数覆盖；日志 $LOG；驱动保持运行，不重启第二次"
echo 'PK220DONE'
exit 0
