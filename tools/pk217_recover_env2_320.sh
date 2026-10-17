#!/bin/bash
# pk216 在静置闸上停住了，原因是新形态：car_5 掉出台面（z≈0，落在沙盘外的无限地面上）。
# 台面上有摩擦，残余动能几十秒就耗尽；台面外没有，所以它带着 pk214 风暴攒下的速度一直滚，
# 第 3 个窗口才静止 —— 闸的判读"动了/不在台面"两条都对，只是**顺序**不对：
# 台面上的车要等它静，台面外的车必须先捞回来再谈静。
# 所以这里把摆位提到静置闸之前（place_cars 会把车流四台连同 car_5 一起放回各自
# start_idx 的车道中心线，z 用静止实测值 6.70），然后再钉、再起驱动。
#
# 生效代码这一轮换了什么（都在工程树 signal_drive_node.py，已同步 home 源）：
#   1) 出界判据不再用全体共用的写死方框（oob_x 22.0 对西环中心线 -21.8 只剩 0.2 m，
#      正常压线就判出界），改成"每台车自己路线的包络 + lane_half + 1.2"，
#      并夹在台面 25 m 内侧；参数留作下限（取两者更宽）。
#      离线自测 pk215 双向校准过：6 个真实"正常压线"读数全部不判出界，
#      4 个真实"被弹出台面"读数（含 x=27.78、32.2）仍然全部判出界。
#   2) lane_guard 1.5 -> 2.6：换段那一瞬车对新航段的合法横偏本来就是 turn_lead(1.2)
#      加转弯半径余量，1.5 会把每次正常过 90° 角都数进离道。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
LOG=/tmp/signal_drive_env2_320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM" || exit 1

po() { timeout 6 gz model -m "$1" -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64; }
zero() {
  for c in car_1 car_2 car_3 car_4 car_5 car_6; do
    timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
  done
}

echo "############ E0) 现场留证（含台面外的车点名）############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在'; exit 1; }
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
ND=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)
echo "  signal_drive 实例：${ND:-0}（必须 0，非 0 就先杀）"
[ "${ND:-0}" = "0" ] || { echo '!! 还有驱动在发指令，停手'; exit 1; }
zero

echo
echo "############ E1) 先把台面上的车摆回车道（含把掉出台面的捞回来）############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '  摆完 %-6s %s\n' "$M" "$(po $M)"; done
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 6 2>&1 | sed 's/^/  /'

echo
echo "############ E2) 静置闸（摆位之后再钉）############"
SETTLE=1
for TRY in 1 2 3; do
  timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk217_settle$TRY.log 2>&1
  SETTLE=$?
  sed 's/^/  /' /tmp/pk217_settle$TRY.log
  [ "$SETTLE" = "0" ] && { echo "  第 $TRY 轮静下来了"; break; }
  echo "  第 $TRY 轮没过：再灌零速并静置 20 s，然后再摆一次（台面外的车要重新捞）"
  zero
  sleep 20
  timeout -k 2 90 python3 tools/place_cars.py "$CFG" >/dev/null 2>&1; true
done
if [ "$SETTLE" != "0" ]; then
  echo '!! 摆位后仍有车在自己动或被挤出车道，停在这里 —— 这时候起驱动只会把读数搅浑。'
  exit 4
fi

echo
echo "############ E3) 起驱动：零参数覆盖，全部走代码/配置原值 ############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV > "$LOG" 2>&1 < /dev/null &
sleep 15
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1）"
for P in oob_x oob_y lane_guard turn_lead max_angular; do
  echo "    $P = $(timeout -k 2 12 ros2 param get /signal_drive_node $P 2>&1 | tr -d '\r' | tail -1)"
done

echo
echo "############ E4) 验收一：复位是否续发（30/60/90/120/150 s）############"
for W in 30 30 30 30 30; do
  sleep "$W"
  echo "  $(date +%H:%M:%S)：日志 $(wc -l < "$LOG" 2>/dev/null; true) 行，复位 $(grep -ac 复位 "$LOG" 2>/dev/null; true)，出界 $(grep -ac 出界 "$LOG" 2>/dev/null; true)，离道 $(grep -ac 离道 "$LOG" 2>/dev/null; true)，脱困 $(grep -ac 脱困 "$LOG" 2>/dev/null; true)"
done
echo '  ## 尾 12 行'
tail -12 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ E5) 验收二：第三方在道 / 里程 / 有没有还在被弹 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
timeout -k 5 75 python3 tools/pk203_track320.py 30 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'
timeout -k 5 45 python3 tools/pk206_odomgap320.py 8 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ E6) 终态 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo 'PK217DONE'
exit 0
