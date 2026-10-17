#!/bin/bash
# 把车队真正恢复到"在各自车道上正常跑"：先让残余弹射停下来，再摆位，再用与车道几何
# 自洽的判据参数起驱动，最后按第三方读数验收。
#
# 为什么不能像上一版那样杀完节点立刻摆位（pk207 的 T4 实测就是这一条）：
#   杀节点只停止**新**的复位指令，车上已经被复位弹射攒下的动能还在物理里走。T4 里
#   car_5 在驱动节点已经死掉、六台零速已灌的情况下 10 s 内又跑了 36.8 m 并掉到 z=0
#   （x=63.9 早越过沙盘 25 m 边界），car_2 也在摆位之后被顶到 (-0.45,7.76)。
#   摆在这种还在 extrude 的状态上，落点读数全是废的。所以这里先加一道"静置闸"，
#   读数不干净就不摆。
#
# 为什么起驱动要带三个 -p（都是运行期覆盖，不改源码、不编译、不同步 home 侧）：
#   oob_x 22.0 / oob_y 19.0 是**世界坐标方框**，而车道中心在西侧 x=-21.8、南侧 y=-16.8，
#   方框只比中心线多出 0.2 m / 2.2 m；车过直角弯的外切本来就要压到 x=-23.9（pk208 实测
#   2.07 m 外偏），于是"正常过弯"被判成出界 → 复位 → 4~8 m/s 弹射 → 弹出去真的出界 →
#   风暴自我维持（/tmp/signal_drive_oob320.log 60 s 内 24 行复位）。放到 24.5 / 22.0：
#   仍在沙盘 ±25 之内，还能拦住"真被弹出台面"的车，只是不再把外切当出界。
#   lane_guard 1.5 → 3.0：这条车道带车队实际贴在 y≈-18.95（离配置中心线 2.15 m），
#   而 obstacle_file 缺省是空串（源码 244 行）意味着 lat_target 恒 0 —— 控制器永远把
#   2.15 m 记成"离道"，于是 line 643 把速度限到 off_lane_speed=0.28、并且连数
#   off_lane_reacquire_seconds 就进"按位姿重取航段"，三次回不来就复位。3.0 让稳态横偏
#   不再算离道，而真正的索引跑偏（实测 3.8~15.8 m）照样拦得住。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
LOG=/tmp/signal_drive_rec320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM" || exit 1

po() { timeout 6 gz model -m "$1" -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64; }

echo "############ R0) 现场留证 ############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在'; exit 1; }
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
if [ -f "$CSV" ] && [ ! -f "${CSV%.csv}.pre_rec.csv" ]; then
  cp "$CSV" "${CSV%.csv}.pre_rec.csv"; echo "  轨迹已归档 ${CSV%.csv}.pre_rec.csv"
fi

echo
echo "############ R1) 杀光驱动实例 + 六台灌零速（风暴的发动者先停手）############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
ND=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)
echo "  signal_drive 残留实例：${ND:-0}（必须 0）"
[ "${ND:-0}" = "0" ] || { echo '!! 没杀干净，停在这里'; exit 1; }
for c in car_1 car_2 car_3 car_4 car_5 car_6; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
done
echo "  六台零速已灌　shuttle=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node'; true) manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node'; true) recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node'; true)"

echo
echo "############ R2) 静置闸：先让残余弹射停下来，读数不干净就不摆 ############"
SETTLE=1
for TRY in 1 2 3; do
  timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk212_settle.log 2>&1
  SETTLE=$?
  sed 's/^/  /' /tmp/pk212_settle.log
  [ "$SETTLE" = "0" ] && { echo "  第 $TRY 轮就静下来了，可以摆位"; break; }
  echo "  第 $TRY 轮还没静：再灌一次零速并静置 20 s"
  for c in car_1 car_2 car_3 car_4 car_5 car_6; do
    timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
  done
  sleep 20
done
if [ "$SETTLE" != "0" ]; then
  echo '!! 三轮静置后仍有车在自己动。这时候摆位等于往还在 extrude 的接触堆里塞车，'
  echo '   落点读数全是废的 —— 停在这里，先把"谁还在动"看完再决定。'
  exit 4
fi

echo
echo "############ R3) 摆位：车流四台回各自 start_idx 车道，car_3 回等待位，car_6 回出生位 ############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '  摆完 %-6s %s\n' "$M" "$(po $M)"; done

echo
echo "############ R4) 钉得住闸：摆完这一版必须六台全静止 ############"
timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk212_static2.log 2>&1
ST2=$?
sed 's/^/  /' /tmp/pk212_static2.log
echo "  静态闸退出码 $ST2（0=六台全钉住）"
[ "$ST2" = "0" ] || echo '!! 摆位钉不住（落点与运行期世界互穿），后面的读数会混进弹射，先记下这条'

echo
echo "############ R5) 起驱动：带与车道几何自洽的三个运行期参数 ############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV \
  -p oob_x:=24.5 -p oob_y:=22.0 -p lane_guard:=3.0 > "$LOG" 2>&1 < /dev/null &
sleep 15
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1）"
for P in oob_x oob_y lane_guard; do
  echo "    $P = $(timeout -k 2 12 ros2 param get /signal_drive_node $P 2>&1 | tr -d '\r' | tail -1)"
done

echo
echo "############ R6) 验收一：复位不再续发（起量后 30/60/90 s 三个采样点）############"
for W in 30 30 30; do
  sleep "$W"
  echo "  起量后 $(date +%H:%M:%S)（日志尾行时刻为准）：日志 $(wc -l < "$LOG" 2>/dev/null; true) 行，复位 $(grep -ac 复位 "$LOG" 2>/dev/null; true)，出界 $(grep -ac 出界 "$LOG" 2>/dev/null; true)，离道 $(grep -ac 离道 "$LOG" 2>/dev/null; true)"
done
echo '  ## 尾 12 行'
tail -12 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ R7) 验收二：第三方按 yaml 折线量在道情况 + 实际走的里程 + 有没有还在被弹 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
timeout -k 5 75 python3 tools/pk203_track320.py 30 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'
timeout -k 5 45 python3 tools/pk206_odomgap320.py 8 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ R8) 终态 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo "  运行期参数覆盖清单（要撤的话就用不带这三个 -p 的同一行重启即可）：oob_x:=24.5 oob_y:=22.0 lane_guard:=3.0"
echo 'PK212DONE'
exit 0
