#!/bin/bash
# 收尾用：不再验证"能不能跑住"，只把沙盘摆成干净状态 —— 杀掉驱动（风暴的发动者），
# 六台按各自路线中心线/等待位/出生格钉住，然后用第三方读数验收"在道且静止"。
# 为什么停在这一步：pk217 已经证明出界包络+lane_guard 两处改完，复位仍 33 次/150 s，
# 剩余病灶是"过角不拐 + 复位弹射"，要继续动控制律，不该在乱态上试。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
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

echo "############ P0) 杀光驱动 + 灌零速 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
echo "  signal_drive 残留：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 0）"
zero

echo
echo "############ P1) 静置：让残余弹射耗尽（台面外的车先不动它，摆位会捞）############"
timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk218_pre.log 2>&1
sed 's/^/  /' /tmp/pk218_pre.log
zero

echo
echo "############ P2) 摆位：车流回中心线，car_3 回等待位，car_6 回出生格 ############"
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -10 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | tail -2 | sed 's/^/  /'
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | tail -2 | sed 's/^/  /'
sleep 6
zero

echo
echo "############ P3) 验收：钉得住 + 在道 ############"
timeout -k 5 60 python3 tools/pk204_static320.py 12 > /tmp/pk218_static.log 2>&1
ST=$?
sed 's/^/  /' /tmp/pk218_static.log
echo "  静态闸退出码 $ST（0=六台全钉住且在台面、姿态正）"
timeout -k 5 60 python3 tools/pk208_onlane320.py "$CFG" 8 2>&1 | sed 's/^/  /'

echo
echo "############ P4) 终态位姿 ############"
for M in car_1 car_2 car_3 car_4 car_5 car_6; do printf '    %-6s %s\n' "$M" "$(po $M)"; done
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（保持 0，等下一步定控制律再起）"
echo 'PK218DONE'
exit 0
