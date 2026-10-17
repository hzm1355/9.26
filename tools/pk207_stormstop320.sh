#!/bin/bash
# 止住"出界→复位→弹射→再出界"的自我维持风暴，并把六台车重新钉回各自的格子。
#
# 这轮的实测起点（不是猜的）：signal_drive_node 里两条判据口径不一致 ——
#   入道判据 lane_half=2.4（横偏 2.4 m 内算在道），
#   出界判据却是世界坐标方框 |x|>oob_x(22.0) 或 |y|>oob_y(19.0)；
#   而西车道中心是 x=-21.8、南车道是 y=-16.8，等于西车道只留 0.2 m、南车道留 2.2 m。
#   /tmp/signal_drive_home320.log 里 66 行复位、46 行出界，最早的一条就是 car_1 在
#   (-22.0, 9.5)（正跑在西车道上）被判出界；每次复位落点又带 4~8 m/s 弹射
#   （tools/place_eject_probe.py 的老读数），弹到 x=33.75 已经越过沙盘 25 m 边界。
# 处置：把出界方框放到与 lane_half 自洽的尺寸（西 -21.8-2.4=-24.2、南 -16.8-2.4=-19.2，
# 沙盘边界 ±25，所以取 oob_x=24.0 / oob_y=21.5 —— 仍然拦得住"真被弹出台面"的车，
# 只是不再把正常的避障横偏当出界）。
# 这是**运行期 -p 覆盖**，没写进源码、没同步 home 侧、没编译；等那一次整栈重启时再决定
# 是否把默认值一起改掉（restore_video_stage303.sh 会核对"出界阈值原样"，改默认值要一起处理它）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
OLDLOG=/tmp/signal_drive_home320.log
LOG=/tmp/signal_drive_oob320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd "$SIM" || exit 1

echo "############ T0) 前置与留证 ############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在'; exit 1; }
echo "  风暴中的位姿（这就是要被止住的那个状态）："
for M in car_1 car_2 car_3 car_4 car_5 car_6; do
  printf '    %-6s %s\n' "$M" "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
done
if [ -f "$OLDLOG" ]; then
  echo "  旧驱动日志：$(wc -l < "$OLDLOG") 行，含「复位」$(grep -ac 复位 "$OLDLOG"; true) 行 —— 原样留着当成因证据"
fi
if [ -f "$CSV" ] && [ ! -f "${CSV%.csv}.pre_oobfix.csv" ]; then
  cp "$CSV" "${CSV%.csv}.pre_oobfix.csv"
  echo "  已归档轨迹 ${CSV%.csv}.pre_oobfix.csv：$(wc -l < "${CSV%.csv}.pre_oobfix.csv") 行"
fi

echo
echo "############ T1) 杀光 signal_drive 实例（风暴的发动者），并给六台全灌零速 ############"
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
echo "  六台零速已灌"
echo "  shuttle 仍是 0 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node'; true)　manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node'; true) recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node'; true)"

echo
echo "############ T2) place_cars 把车流四台摆回 start_idx 车道 ############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'

echo
echo "############ T3) car_3 回等待位、car_6 回出生位（风暴里可能被顶走）############"
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'

echo
echo "############ T4) 钉得住闸（摆位本身不能回弹，否则后面的读数全是风暴）############"
# 不接管道：`python ... | sed` 之后 $? 是 sed 的退出码，判据会被抹成 0（假绿）。
timeout -k 5 60 python3 tools/pk204_static320.py 10 > /tmp/pk207_static.log 2>&1
ST=$?
sed 's/^/  /' /tmp/pk207_static.log
echo "  静态闸退出码 $ST（0=六台全钉住）"

echo
echo "############ T5) 重起驱动，带与 lane_half 自洽的出界方框（只这一处覆盖）############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV -p oob_x:=24.0 -p oob_y:=21.5 \
  > "$LOG" 2>&1 < /dev/null &
sleep 15
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1）"
echo "  运行期参数回读（节点自己的 DDS 参数，不是我写的命令行）："
timeout -k 2 12 ros2 param get /signal_drive_node oob_x 2>&1 | tr -d '\r' | tail -1 | sed 's/^/    /'
timeout -k 2 12 ros2 param get /signal_drive_node oob_y 2>&1 | tr -d '\r' | tail -1 | sed 's/^/    /'
echo "  前 15 s 内「复位」行数：$(grep -ac 复位 "$LOG" 2>/dev/null; true)（期望 0）"

echo
echo "############ T6) 让风暴判定自己浮出来：静置 60 s 再数复位 ############"
sleep 60
echo "  60 s 后日志共 $(wc -l < "$LOG" 2>/dev/null; true) 行；其中「复位」$(grep -ac 复位 "$LOG" 2>/dev/null; true) 行、「出界」$(grep -ac 出界 "$LOG" 2>/dev/null; true) 行、「离道」$(grep -ac 离道 "$LOG" 2>/dev/null; true) 行"
echo '  ## 尾 12 行'
tail -12 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ T7) 第三方复核：按 yaml 折线量真实横向距离，不看节点自己的判据 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
echo '=== 30 s 累计路程（顺走≈0.70 m/s×30 s×仿真倍率）==='
timeout -k 5 75 python3 tools/pk203_track320.py 30 2>&1 | sed 's/^/  /'
echo '=== odom vs 模型位姿 + 实测速（期望实测速 <=1 m/s，出现 3~4 m/s 就是还在被弹）==='
timeout -k 5 45 python3 tools/pk206_odomgap320.py 8 2>&1 | sed 's/^/  /'

echo
echo "############ T8) 终态 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do
  printf '    %-6s %s\n' "$M" "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
done
echo 'PK207DONE'
exit 0
