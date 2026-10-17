#!/bin/bash
# 把车队恢复到各自航线上，并且**不带任何运行期参数覆盖** —— 与 pk212 的唯一区别就是
# R5 那一行去掉了 -p oob_x/-p oob_y/-p lane_guard，让节点用工程树代码里的原值
# （oob_x 22.0 / oob_y 19.0 / lane_guard 1.5）跑，跑完用 ros2 param get 把生效值钉回来。
#
# 三条覆盖是怎么来的、为什么撤：
#   那三条是我在现场拍的数（24.5/22.0/3.0），既不在 D 盘配置里、也不在 home 侧，
#   属于"只有这一次启动生效"的偏离。用户点名沙盘乱套并要求按正常航线恢复，
#   我先把偏离撤干净，再拿实测判断到底是不是参数的问题 —— 而不是拿覆盖把病灶糊住。
#
# 已经量到的两条事实（决定这一轮验收要看什么）：
#   1) 配置南环中心线 y=-16.8 是**能开的**：pk213 按完整位姿叠加口径统计，50 m 宽里
#      只被 x=-25..-24.75 那 0.25 m 挡到；所以车队贴在 y≈-18.9 不是"配置线被占"。
#   2) 沙盘南侧 y=-20.00..-19.75 有一条横贯全宽的实体路缘（fixtures/c80，顶面 z=7.00
#      高于路面 6.70），y=-19.50..-19.25 还有一排桩（c88/c89/c90/c91/c92）。车队机头朝正南
#      怼在这两样之间 —— 这是"过角冲出去"的落点，不是随机漂移。
#   按这个形状，如果撤掉覆盖后复现"过南角冲过 -16.8 再楔进路缘"，那就是转向前置量
#   turn_lead=1.2 m 与指令上限 max_angular=0.45 rad/s 不自洽（0.70/0.45≈1.56 m 的最小
#   转弯半径比换段点还大），要在代码/配置里改，而不是靠现场加宽护栏。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
LOG=/tmp/signal_drive_env320.log
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

echo "############ C0) 现场留证 + 生效世代指纹 ############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在'; exit 1; }
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
if [ -f "$CSV" ] && [ ! -f "${CSV%.csv}.pre_can.csv" ]; then
  cp "$CSV" "${CSV%.csv}.pre_can.csv"; echo "  轨迹已归档 ${CSV%.csv}.pre_can.csv"
fi
echo "  --- 生效节点文件与 D 盘/归档件的关系（md5，点名用的是哪一代）---"
md5sum ~/v2x_ws/install/v2x_demo/lib/v2x_demo/signal_drive_node.py \
       ~/v2x_ws/src/v2x_demo/v2x_demo/signal_drive_node.py \
       $SIM/src/v2x_demo/v2x_demo/signal_drive_node.py \
       ~/v2x_ws/src/v2x_demo/v2x_demo/signal_drive_node.py.reverted1014_1016_094753.keep \
       2>/dev/null | cut -c1-80

echo
echo "############ C1) 杀光驱动实例 + 六台灌零速 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
ND=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)
echo "  signal_drive 残留实例：${ND:-0}（必须 0）"
[ "${ND:-0}" = "0" ] || { echo '!! 没杀干净，停在这里'; exit 1; }
zero
echo "  六台零速已灌　shuttle=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node'; true) manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node'; true) recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node'; true)"

echo
echo "############ C2) 静置闸：残余弹射没停就不摆 ############"
SETTLE=1
for TRY in 1 2 3; do
  timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk216_settle.log 2>&1
  SETTLE=$?
  sed 's/^/  /' /tmp/pk216_settle.log
  [ "$SETTLE" = "0" ] && { echo "  第 $TRY 轮静下来了，可以摆位"; break; }
  echo "  第 $TRY 轮还没静：再灌零速并静置 20 s"
  zero
  sleep 20
done
if [ "$SETTLE" != "0" ]; then
  echo '!! 三轮静置后仍有车在自己动 —— 这时候摆位落点读数全是废的，停在这里。'
  exit 4
fi

echo
echo "############ C3) 摆位：车流回各自 start_idx 车道中心线 ############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '  摆完 %-6s %s\n' "$M" "$(po $M)"; done
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 6 2>&1 | sed 's/^/  /'

echo
echo "############ C4) 钉得住闸 ############"
timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk216_static2.log 2>&1
ST2=$?
sed 's/^/  /' /tmp/pk216_static2.log
echo "  静态闸退出码 $ST2（0=六台全钉住）"

echo
echo "############ C5) 起驱动：只有文档化的两个参数，护栏/转向全部用代码原值 ############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV > "$LOG" 2>&1 < /dev/null &
sleep 15
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1）"
for P in oob_x oob_y lane_guard turn_lead max_angular; do
  echo "    $P = $(timeout -k 2 12 ros2 param get /signal_drive_node $P 2>&1 | tr -d '\r' | tail -1)"
done

echo
echo "############ C6) 验收一：复位是否续发（30/60/90/120 s 四个采样点）############"
for W in 30 30 30 30; do
  sleep "$W"
  echo "  $(date +%H:%M:%S)：日志 $(wc -l < "$LOG" 2>/dev/null; true) 行，复位 $(grep -ac 复位 "$LOG" 2>/dev/null; true)，出界 $(grep -ac 出界 "$LOG" 2>/dev/null; true)，离道 $(grep -ac 离道 "$LOG" 2>/dev/null; true)，脱困 $(grep -ac 脱困 "$LOG" 2>/dev/null; true)"
done
echo '  ## 尾 14 行'
tail -14 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ C7) 验收二：第三方在道 / 里程 / 是否还被弹 ############"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 12 2>&1 | sed 's/^/  /'
timeout -k 5 75 python3 tools/pk203_track320.py 30 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'
timeout -k 5 45 python3 tools/pk206_odomgap320.py 8 car_1 car_2 car_4 car_5 2>&1 | sed 's/^/  /'

echo
echo "############ C8) 终态位姿 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo 'PK216DONE'
exit 0
