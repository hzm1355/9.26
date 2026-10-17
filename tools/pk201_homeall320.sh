#!/bin/bash
# 一次把"所有小车回归正常航线"做完：车流四台摆回各自 start_idx 车道，停车场那台
# 翻倒掉出台面的 car_3 扶正放回等待位，car_6 摆回标定出生位，然后重起驱动让每个
# agent 重新 snap，并用 40 s 累计路程做效果复核。
#
# 顺序里的三个理由（都是这轮或前几轮实测出来的，不是猜的）：
#   1) 先杀 shuttle 再摆 car_3。shuttle 是 /car_3/cmd_vel 的发布者，现在它处于
#      "停驶挂起"（轮轴偏离本段车道 2.97 m > 上限 1.20 m）但实例还活着、还在发；
#      探针更要求它是 0 实例，否则两边的指令互相盖掉，读数说明不了任何事。
#   2) 驱动节点必须在摆位之前停（place_cars.py 的 docstring 就写了这条），
#      而且只杀 signal_drive_node —— 绝不叫 stop_signal.sh / restart_clean_stack.sh，
#      那两片按 lib/ 目录整片 pkill，会把 manager/recorder 一起带走（宽 pkill 连带杀伤）。
#   3) 三态探针排在"重起驱动之前"：探针要读"被顶住"就得让车队静止，
#      车队一跑起来，它的邻近车距离每 0.1 s 都在变，读数没法解释。
#
# 不做的事：不 colcon build、不同步 home 侧副本（#128 的 0.85/1.19 与 #104 的刹车闸
# 都按用户指令等一次整栈重启一起生效），不碰 /delete_entity（重 spawn 是高危动作，
# 要用户明文同意；这里只把车扶正摆回去，能不能出力交给探针实测）。
# 三条硬规矩沿用现场脚本：杀同名实例要杀光；杀完 cmd_vel 发布者必须灌零速；只用已验证过的调用形状。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
DLOG=/tmp/signal_drive_home320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "############ S0) 拆之前的硬前置（没过就不动任何进程）############"
GZ=$(pgrep -cx gzserver 2>/dev/null; true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在，摆位与探针全是空转'; exit 1; }
P3=$(timeout 6 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r')
echo "  car_3 位姿（拆前）：$(echo "$P3" | cut -c1-70)"
echo "$P3" | grep -aq 'Unable' && { echo '!! 运行期 world 里没有 car_3'; exit 1; }
echo "  各车摆前位姿："
for M in car_1 car_2 car_3 car_4 car_5 car_6; do
  printf '    %-6s %s\n' "$M" "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
done
NP0=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null; true)
NA0=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null; true)
echo "  账本基线：通行 ${NP0:-0} 行 / 闸机动作 ${NA0:-0} 行"
echo "  归档当前驱动轨迹（重起驱动会把 CSV 以 w 截断）"
if [ -f "$CSV" ] && [ ! -f "${CSV%.csv}.pre_homeall.csv" ]; then
  cp "$CSV" "${CSV%.csv}.pre_homeall.csv"
  echo "    已存 ${CSV%.csv}.pre_homeall.csv：$(wc -l < "${CSV%.csv}.pre_homeall.csv") 行"
else
  echo "    不动（已存在或无 CSV）"
fi

echo
echo "############ S1) 只停 parking_shuttle_node 的全部实例，并给 car_3/car_6 灌零速 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
NS=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node'; true)
echo "  shuttle 残留实例：${NS:-0}（必须 0，探针要当唯一发布者）"
[ "${NS:-0}" = "0" ] || { echo '!! 没杀干净，停在这里'; exit 1; }
for c in car_3 car_6; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
  echo "  $c 零速已灌"
done
echo "  manager/recorder 没被碰：manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node'; true) recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node'; true)"

echo
echo "############ S2) 只停 signal_drive_node 的全部实例，并给车流四台灌零速 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node'; true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
ND=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)
echo "  signal_drive 残留实例：${ND:-0}（必须 0，摆位不能和驱动抢方向盘）"
[ "${ND:-0}" = "0" ] || { echo '!! 没杀干净，停在这里'; exit 1; }
for c in car_1 car_2 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
  echo "  $c 零速已灌"
done

echo
echo "############ S3) place_cars：车流四台按 start_idx 摆回各自车道（跳过 drive:false 的 car_3）############"
cd "$SIM" || exit 1
timeout -k 2 90 python3 tools/place_cars.py "$CFG" 2>&1 | tail -8 | sed 's/^/  /'

echo
echo "############ S4) 把翻倒掉台的 car_3 扶正回等待位，car_6 摆回标定出生位 ############"
echo "  car_3 等待位 = parking_zones.yaml 里 fleets.car_3 的 entry#0，也是它 world 出生位"
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'
echo "  car_6 出生位 = world 里 <pose>23.50 -23.50 7.00</pose>（台面静置高度按 6.70 给）"
timeout -k 2 40 python3 tools/pk202_putpose.py car_6 23.50 -23.50 6.70 0 0 0 2>&1 | sed 's/^/  /'
echo "  摆完：car_3 $(timeout 6 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
echo "  摆完：car_6 $(timeout 6 gz model -m car_6 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"

echo
echo "############ S5) car_3 三态探针（车队此刻静止，才读得出「顶住」这一支）############"
bash tools/pk_run308.sh 2>&1 | tail -20 | sed 's/^/  /'
timeout -k 2 8 ros2 topic pub --once /car_3/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1; true
echo "  探针后补发 car_3 零速"
python3 tools/pk181_decide308.py /tmp/pk_probe308.log
RC=$?
echo "  判定退出码 $RC（0=出力了/不必重 spawn，2=零出力/只有重 spawn 能复位，3=探针自己没跑完）"

echo
echo "############ S6) 探针给过油，car_3 可能已经离开等待位 —— 再钉一次 ############"
timeout -k 2 40 python3 tools/pk202_putpose.py car_3 12.00 -13.95 6.70 0 0 1.5708 2>&1 | sed 's/^/  /'

echo
echo "############ S7) 钉得住闸：一台都不许动、都要在台面上、姿态都要正 ############"
timeout -k 5 60 python3 tools/pk204_static320.py 10 2>&1 | sed 's/^/  /'
ST=$?
echo "  静态闸退出码 $ST（0=六台全钉住）"

echo
echo "############ S8) 重起 signal_drive_node（六个 agent 里五个 drive:true 全部重新 snap）############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV > "$DLOG" 2>&1 < /dev/null &
sleep 12
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true)（必须 1）"
echo "  驱动日志尾 6 行："
tail -6 "$DLOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo
echo "############ S9) 效果复核：40 s 每车累计路程（等红灯的车照样有路程，真卡死的才是 0）############"
timeout -k 5 90 python3 tools/pk203_track320.py 40 2>&1 | sed 's/^/  /'
TR=$?
echo "  效果复核退出码 $TR（1 只表示有台读数低，等红灯也会这样，末行点名列）"

echo
echo "############ S10) 收尾对账 ############"
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null; true)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null; true)
echo "  通行账本：$NP0 -> ${NP1:-0} 行；闸机动作：$NA0 -> ${NA1:-0} 行"
echo "  节点实例：drive=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node'; true) manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node'; true) recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node'; true) shuttle=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node'; true)（shuttle 保持 0，等整栈重启那一次再拉）"
echo "  终态位姿："
for M in car_1 car_2 car_3 car_4 car_5 car_6; do
  printf '    %-6s %s\n' "$M" "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-64)"
done
echo '  ## 两道闸当前状态（gate_in 的 fault 是否还锁存着）'
timeout -k 3 15 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 \
  | tr -d '\r' | grep -aE 'gate_id|state:|angle_rad|fault' | head -8 | sed 's/^/    /'
echo "  ## car_3 探针判定：$RC（0=不重 spawn；2=只有 /delete_entity 重 spawn 能复位，那一步要用户明文批准；3=探针没跑完，读数不作数）"
echo "  ## 静态闸：$([ "$ST" = 0 ] && echo PASS || echo FAIL)　效果复核：$([ "$TR" = 0 ] && echo PASS || echo 有台读数低)"
echo 'PK201DONE'
exit 0
