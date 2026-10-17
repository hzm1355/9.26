#!/bin/bash
# 条目 3 的速度扫参：逐档改写 cruise_speed -> 只重启 signal_drive 节点 -> 统计该档抓拍
#
# 只重启 signal_drive，不动 traffic_light_manager / 车牌三件套 / gzserver：
#   · signal_drive_node 在首个 tick 会 snap_to_route() 按当前位姿入线，重启不需要重新摆车；
#   · 重启整栈会把 /tmp/v2x_gates 的历史抓拍和黑名单一起清掉，前后档位就不可比了。
#
# 用法：bash tools/speed_sweep.sh                 跑默认四档（0.7/2/4/6 m/s）
#       bash tools/speed_sweep.sh "3.0 120"       只跑一档（速度 秒）
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=~/v2x_ws/src/v2x_demo/config/signal_drive_params.yaml
# 录像取证时要写一份独立明细：和历史 sweep 混在同一个 label 里，字幕上的
# "抓拍 N 命中 M" 就说不清是这一轮拍的还是上一轮攒的。
CSV=${SWEEP_CSV:-/tmp/v2x_records/speed_sweep.csv}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

RUNS="0.7 300 2.0 170 4.0 140 6.0 140"
PREFLIGHT=0
# 想只跑一档就传成对参数，例如 `bash tools/speed_sweep.sh 3.0 120`
set -- ${*:-$RUNS}

restart_drive() {
  for pid in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true) \
             $(pgrep -f 'run v2x_demo signal_drive_node' || true); do
    kill "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true); do
    kill -9 "$pid" 2>/dev/null || true   # rclpy 把 TERM 只当 shutdown 标志，主线程卡住就得 KILL
  done
  sleep 1
  # 先把五车摆回各自起点再拉起驱动：上一档高速会把车队挤成一坨（跟车距离是定值 2.6 m，
  # 不随速度放大），带着这种"串牌现场"进下一档，量到的就分不清是速度还是车距。
  # 顺序不能反：驱动节点一 tick 就按当前位姿 snap_to_route，先摆车才不会指错航段。
  timeout -k 3 60 python3 $SIM/tools/place_cars.py 2>&1 | tr -d '\r' | sed 's/^/  place /'
  # 跟车距离随速度放大（0.7 m/s 时仍是默认 2.6 m，不影响基线档）
  local gap
  gap=$(awk -v s="$SP" 'BEGIN{printf "%.2f", 2.6 + 0.7*s}')
  setsid nohup ros2 run v2x_demo signal_drive_node.py \
    --ros-args -p config_file:=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml \
               -p path_csv:=/tmp/signal_drive_path.csv \
               -p follow_gap:=$gap \
    > /tmp/signal_drive_sweep.log 2>&1 < /dev/null &
  sleep 8
  echo "  drive: $(pgrep -f 'lib/v2x_demo/signal_drive_node' | tr '\n' ' ')  follow_gap=$gap"
}

while [ $# -ge 2 ]; do
  SP=$1; SEC=$2; shift 2
  if [ "$PREFLIGHT" != 1 ]; then
    # 预检：五副牌必须真的在车上。/plate/request 是一次性指令，重启后车上是占位牌，
    # 扫十五分钟拿到的会是一张"全是空号"的废表（这一轮就踩过）。
    echo '### 预检：下发五副默认牌并确认 plate_gen 应用'
    bash $SIM/tools/apply_default_plates.sh || \
      { echo '[speed_sweep] 中止：先跑 tools/restore_default.sh 或 restart6.sh'; exit 1; }
    PREFLIGHT=1
  fi
  echo "=================== $SP m/s 档（采样 ${SEC}s）==================="
  sed -i "s/cruise_speed: [0-9.]*/cruise_speed: $SP/" $CFG
  echo "  配置里 cruise_speed 现为: $(grep -c "cruise_speed: $SP\$" $CFG) 辆"
  restart_drive
  python3 -u $SIM/tools/speed_capture_test.py "${SP}" "$SEC" "$CSV" 2>&1 | tr -d '\r'
done

# 跑完把配置恢复到默认巡航速度，别让下一次联调拿到一个 6 m/s 的栈
sed -i "s/cruise_speed: [0-9.]*/cruise_speed: 0.7/" $CFG
SP=0.7
restart_drive
echo "[speed_sweep] DONE，配置已恢复 cruise_speed: 0.7，明细见 $CSV"
