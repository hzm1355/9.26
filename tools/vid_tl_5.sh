#!/bin/bash
# 任务二 条目5 视频取证：仿真全流程录制 -> 回放，一镜到底。
# 前段是实时仿真（同时开录），中段停掉实时配时栈、装载录制文件，后段由回放器接管，
# 灯态与车位重新按录制时的时序演一遍；片尾把"逐条对照"的量化结论写进字幕。
# 收尾（恢复实时栈 + 车牌三件套 + 默认牌）放在录制结束之后执行，不占画面。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务二_信号灯配时/条目5_全流程录制与回放复现灯时序与车辆通行.mp4"
# save 落盘路径由录制器节点的 output_file 参数决定（launch 传的是 record_file:=/tmp/tl_log.json），
# 写成别的名字就会去读一个不存在的文件，回放对照就成了无源之水。
REC_FILE=/tmp/tl_log.json
OBS_FILE=/tmp/replay_observed_vid.json
INTER=$VIEW_INTER
CAR=$VIEW_CAR

vid_start "$OUT" 330 "$INTER,$CAR" tl \
  "任务二 条目5  全流程录制与回放（上：路口相机 下：car_1 前视相机）"
cap "录制器 = traffic_light_recorder（订阅 6 组 state/state_detail/pedestrian + 5 车位姿）"
cap "回放器 = traffic_light_recorder.py player，1.0 倍速、循环，接管灯态与车辆位姿"
sleep 8
capq "timeout -k 3 15 ros2 service call /traffic_light_recorder/clear std_srvs/srv/Trigger | tail -1"
capq "timeout -k 3 15 ros2 service call /traffic_light_recorder/start std_srvs/srv/Trigger | tail -1"
cap "== 开始录制，持续 80 s（画面为实时仿真，非回放）=="
sleep 40
cap "  录制中……注意右栏相位与画面里的灯头颜色是同步变化的"
sleep 36
capq "timeout -k 3 20 ros2 service call /traffic_light_recorder/save std_srvs/srv/Trigger | tail -1"
capq "ls -la $REC_FILE; python3 $TOOLS/tl_rec_meta.py $REC_FILE"
cap "↑ 录制文件元信息：时长 / 事件数 / 覆盖的信号组与车辆"
sleep 6
cap "== 停掉实时配时栈：回放期间灯态与车位必须由 player 接管，否则两路数据源会互相覆盖 =="
capq "bash $SIM_ROOT/tools/stop_signal.sh | tail -3"
sleep 10
cap "== 起回放器并 load（先不起播）=="
pkill -f 'traffic_light[_]recorder.py player' 2>/dev/null || true
sleep 1
setsid nohup ros2 run v2x_demo traffic_light_recorder.py player \
  --ros-args -p input_file:=$REC_FILE -p loop:=true -p playback_speed:=1.0 \
  > /tmp/player_vid.log 2>&1 < /dev/null &
sleep 9
capq "timeout -k 3 20 ros2 service call /traffic_light_player/load std_srvs/srv/Trigger | tail -1"
capq "grep -aE 'loaded|events' /tmp/player_vid.log | tail -2"
sleep 4
cap "== 探针就位，2 s 后起播；下面 90 s 画面即为回放内容 =="
timeout -k 3 140 python3 -u "$TOOLS/replay_probe.py" 90 "$OBS_FILE" > /tmp/replay_probe_vid.log 2>&1 &
PROBE_PID=$!
sleep 2
capq "timeout -k 3 20 ros2 service call /traffic_light_player/play std_srvs/srv/Trigger | tail -1"
sleep 40
cap "  回放中……灯色跳变与车辆通行按录制时的绝对时序重现（1.0 倍速）"
sleep 52
capq "wait $PROBE_PID; tail -1 /tmp/replay_probe_vid.log"
cap "== 逐条对照：录制流相变序列 vs 回放实测序列 =="
# 对照窗口必须短于录制时长：player 是 loop:=true，窗口跨过回绕点就会多出一次
# "末相位 -> 首相位"的拼接跳变，看起来像非法相变，其实是循环边界。
cap "对照窗口取 0~68 s，短于录制的 75.9 s，避开 player 循环回绕处的拼接跳变"
capq "timeout -k 3 60 python3 -u $TOOLS/replay_compare.py $REC_FILE $OBS_FILE 2.0 68 | sed -n '1,2p;6,12p'"
vid_wait
echo "--- 录制结束，开始恢复栈 ---"
pkill -f 'traffic_light[_]recorder.py player' 2>/dev/null || true
bash "$SIM_ROOT/tools/stop_signal.sh" >/dev/null 2>&1 || true
bash "$SIM_ROOT/tools/start_signal.sh" 2>&1 | tail -4
bash "$SIM_ROOT/tools/start_plate_nodes.sh" 2>&1 | tail -3
bash "$SIM_ROOT/tools/apply_default_plates.sh" 2>&1 | tail -3
echo DONE
