#!/bin/bash
# 任务二 条目5 在线复验：**当场**录一段 -> 停实时配时栈 -> 回放这一段 -> 逐条对照。
#
# 和 verify_replay5.sh 的区别：那份直接拿 10-06 归档的 586 s 录像起播，只能证明
# "回放器读得懂文件"；这一份录的是当前时线的灯态与车位，录完立刻回放对照，
# 才是"全流程录制回放"的闭环（时序变化 + 车辆通行过程两样都要对得上）。
#
# 已知副作用：stop_signal.sh 的 pkill 匹配整个 v2x_demo lib 目录，会把任务三的
# 车牌三件套一起杀掉。所以结尾必须把信号栈和车牌节点都拉回来并重新下发默认牌，
# 否则下一轮卡口测试拿到的是一个没有牌的栈（跑十分钟得到一张废表）。
#
# 用法：bash tools/verify_replay_fresh.sh [录制秒数=150] [回放对照秒数=90]
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
REC_SEC=${1:-150}
PROBE_SEC=${2:-90}
REC_FILE=/tmp/tl_log.json
OBS_FILE=/tmp/replay_observed.json
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 1) 当场录制 ${REC_SEC} s（clear -> start -> save）==="
timeout -k 3 15 ros2 service call /traffic_light_recorder/clear std_srvs/srv/Trigger 2>&1 | tail -1
timeout -k 3 15 ros2 service call /traffic_light_recorder/start std_srvs/srv/Trigger 2>&1 | tail -1
sleep "$REC_SEC"
timeout -k 3 20 ros2 service call /traffic_light_recorder/save std_srvs/srv/Trigger 2>&1 | tail -1
python3 $SIM/tools/tl_rec_meta.py "$REC_FILE"

echo "=== 2) 停实时配时栈（回放期间灯态与车位由 player 接管）==="
bash $SIM/tools/stop_signal.sh

echo "=== 3) 起回放器并 load（先不起播，等探针就位）==="
pkill -f 'traffic_light_recorder.py player' 2>/dev/null || true
sleep 1
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=$REC_FILE -p loop:=true -p playback_speed:=1.0 \
  > /tmp/player.log 2>&1 < /dev/null &
sleep 8
timeout -k 3 20 ros2 service call /traffic_light_player/load std_srvs/srv/Trigger 2>&1 | tail -1

echo "=== 4) 探针先就位，再起播（两者的墙钟差就是对照时的时间轴零点）==="
timeout -k 3 $((PROBE_SEC + 40)) python3 -u $SIM/tools/replay_probe.py "$PROBE_SEC" "$OBS_FILE" \
  > /tmp/replay_probe.log 2>&1 &
PROBE_PID=$!
TP=$(date +%s.%N)
sleep 2
TL=$(date +%s.%N)
timeout -k 3 20 ros2 service call /traffic_light_player/play std_srvs/srv/Trigger 2>&1 | tail -1
wait $PROBE_PID || true
OFF=$(awk -v a="$TP" -v b="$TL" 'BEGIN{printf "%.2f", b-a}')
echo "  起播相对探针零点 $OFF s，探针实收：$(tail -1 /tmp/replay_probe.log)"

echo "=== 5) 逐条对照（灯态相变时长偏差 + 车辆位姿误差）==="
# 对照不通过时 replay_compare.py 会以非 0 退出；这里不能让它把第 6 步（恢复栈 + 车牌节点）截掉，
# 否则跑一次失败的复验会把整个仿真留在"实时栈已停、车上没有牌"的状态。
python3 $SIM/tools/replay_compare.py "$REC_FILE" "$OBS_FILE" "$OFF" || \
  echo "  [注意] replay_compare 退出码非 0，对照结论见上面的逐条表"

echo "=== 6) 恢复实时栈与车牌节点 ==="
pkill -f 'traffic_light_recorder.py player' 2>/dev/null || true
bash $SIM/tools/start_signal.sh 2>&1 | tail -6
bash $SIM/tools/start_plate_nodes.sh 2>&1 | tail -4
bash $SIM/tools/apply_default_plates.sh
echo "[verify_replay_fresh] DONE：录制 $REC_SEC s / 对照 $PROBE_SEC s，原始输出见 /tmp/replay_probe.log"
