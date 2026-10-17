#!/bin/bash
# 整栈重启到一个干净物理态（gzserver + gzclient + 车牌链路 + 测速链路 + 信号子系统
# + 停车场链路）。
#
# 为什么必须整栈重启而不是只重启 gzserver：车牌是 gzserver **进程内** SetMaterial 贴上去的
# （见 memory「运行时换车牌纹理机制」），服务端一换，识别器/记录器内存里的累积也全作废；
# 而驱动节点的复位风暴会把车甩到界外，界外的车落在地面层 z≈0.45，越界复位再瞬移回来，
# 这个循环里没有任何一次测量是可信的。
#
# 与 restart6.sh 的差别：
#  1) 先备份 /tmp/v2x_gates 的实拍帧（restart6 第 2 步会 rm 掉，那些帧是 blur_sweep 的回归输入）；
#  2) 不依赖 /tmp/list_models.py，改用本仓库的 tools/world_inventory.py 校验实体；
#  3) 这一步**不摆位**：摆位（place_cars）会瞬移车辆，而本轮实测每次瞬移都带一个
#     ~8 m/s 的穿透回弹（tools/traction_test.py 三段实测）。摆位留给调用方按需要单独做。
#
# 用法：bash restart_clean_stack.sh [yes|skip]
#        第二个参数=skip 时不拉信号子系统（受控测速取证需要驱动节点全程不在）
set -e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
FLEET=${2:-yes}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '[restart] 0) 备份已经复现过问题的抓拍帧与测速记录'
# /tmp/v2x_parking 一起备份：停车场两份 JSONL 是追加式账本，重起后记录器从 0 开始
# 数内存，先轮转出时间戳副本，事后对账才分得清"重启前的旧账"和"重启后的新账"。
for SRC in /tmp/v2x_gates /tmp/v2x_records /tmp/v2x_parking; do
  if [ -d $SRC ] && [ -n "$(ls -A $SRC 2>/dev/null)" ]; then
    BK=${SRC}_$(date +%m%d_%H%M%S)
    cp -r $SRC $BK
    echo "  $SRC -> $BK（$(ls $BK | wc -l) 项）"
  fi
done

echo '[restart] 1) 杀掉所有 ROS 侧节点（rclpy 吞 TERM，一律补 KILL）'
bash $WS/tools/stop_signal.sh >/dev/null 2>&1 || true
for PAT in 'lib/v2x_demo/signal_drive_node' 'run v2x_demo signal_drive_node' \
           'lib/v2x_demo/speed_detect_node' 'run v2x_demo speed_detect_node' \
           'lib/v2x_demo/speed_recorder_node' 'run v2x_demo speed_recorder_node' \
           'lib/v2x_demo/plate_gen_node' 'run v2x_demo plate_gen_node' \
           'lib/v2x_demo/plate_recognizer_node' 'run v2x_demo plate_recognizer_node' \
           'lib/v2x_demo/plate_recorder_node' 'run v2x_demo plate_recorder_node' \
           'lib/v2x_demo/traffic_light' 'run v2x_demo adaptive_timing_node' \
           'recover[_]fleet' 'speed[_]ctrl_evidence'; do
  for PID in $(pgrep -f "$PAT" 2>/dev/null || true); do
    kill -9 "$PID" 2>/dev/null || true
  done
done
sleep 2

echo '[restart] 2) 停 gzclient / gzserver'
pkill -x gzclient 2>/dev/null || true
sleep 2
for i in 1 2 3; do
  P="$(pgrep -x gzserver || true)"
  [ "${P// /}" = "" ] && break
  kill -9 $P 2>/dev/null || true
  sleep 3
done
echo "  残留 server='$(pgrep -x gzserver || true)' client='$(pgrep -x gzclient || true)'"

echo '[restart] 3) 起 gzserver（沙盘世界加载要 30~90 s）'
bash $WS/scripts/start_gazebo_city.sh 2>&1 | tail -3

echo '[restart] 4) 起 gzclient（必须带 GAZEBO_MODEL_PATH，否则会静默退出）'
bash $WS/scripts/start_gzclient.sh 2>&1 | tail -2

echo '[restart] 5) 等世界加载完，再校验实体'
sleep 75
# 用 tail 不用 head：head 提前关管道会让上游 python 吃 SIGPIPE 非零退出，set -e 直接把重启掐断
timeout -k 5 60 python3 -u $WS/tools/world_inventory.py 2>&1 | tail -40 || true

# 顺序必须在车牌/测速链路之前：start_signal.sh 第 7 行会先跑 stop_signal.sh，而它的
# pkill 式 '/install/v2x_demo/lib/v2x_demo/' 匹配整个包目录，会把已经起好的
# plate_* / speed_detect / speed_recorder 一并带走。实测踩过：整栈重启打印 DONE，
# 事后查 speed_detect_node 实例数 0、/speed/zones 服务不可用、零抓拍。
if [ "$FLEET" = "yes" ]; then
  echo '[restart] 6) 信号子系统 + 车队驱动（含摆位，会瞬移车辆）'
  bash $WS/tools/start_signal.sh 2>&1 | tail -5
else
  echo '[restart] 6) 按参数 skip：不拉信号子系统（驱动节点全程不在）'
fi

echo '[restart] 7) 车牌链路（生成器 + 参考识别 + 记录导出）'
setsid nohup ros2 run v2x_demo plate_gen_node.py > /tmp/plate_gen.log 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 run v2x_demo plate_recognizer_node.py > /tmp/plate_rec.log 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 run v2x_demo plate_recorder_node.py > /tmp/plate_recorder.log 2>&1 < /dev/null &
sleep 3
echo "  plate_gen=$(pgrep -cf 'lib/v2x[_]demo/plate[_]gen[_]node' || true) recognizer=$(pgrep -cf 'lib/v2x[_]demo/plate[_]recognizer[_]node' || true) recorder=$(pgrep -cf 'lib/v2x[_]demo/plate[_]recorder[_]node' || true)"

echo '[restart] 8) 下发五副默认车牌（进程内 SetMaterial，不贴则卡口一律拍到无牌）'
bash $WS/tools/apply_default_plates.sh 2>&1 | tail -4 || echo '  !! 贴牌不完整'

echo '[restart] 9) 测速链路（检测 + 记录导出）'
bash $WS/tools/start_speed_nodes.sh 2>&1 | tail -4

echo '[restart] 9b) 兜底闸：宽 pkill 若还带走任何一路，当场拉回（记录器不强制重启，保住内存累积）'
bash $WS/tools/ensure_recorder.sh 2>&1 | tail -2 || true
bash $WS/tools/ensure_speed_recorder.sh 2>&1 | tail -2 || true
ND=$(pgrep -cf 'lib/v2x[_]demo/speed[_]detect[_]node' || true)
if [ "${ND:-0}" != "1" ]; then
  echo "  检测节点实例=${ND:-0}，补拉"
  bash $WS/tools/start_speed_nodes.sh speed_detect_node.py 2>&1 | tail -3
fi

echo '[restart] 9c) 停车场三节点（管控 + 记录 + 摆渡车）'
# 必须排在 6) start_signal.sh 之后：那一步第 7 行会跑 stop_signal.sh，它的 pkill 式
# '/install/v2x_demo/lib/v2x_demo/' 匹配整个包目录，先起的 parking_* 会被一并带走。
# 自检脚本自己会打印"实例数/话题发布者/car_6 在不在世界里的"三重确认，
# 这里先整份落到 /tmp，再从落盘文件里取要点 —— 直接在管道上 grep 一旦失配就静默出空证据。
bash $WS/tools/start_parking_nodes.sh > /tmp/start_parking.log 2>&1 || true
grep -aE '!!|python 实例|Publisher count|shuttle_|位姿|闸机动力学|OK：|服务' /tmp/start_parking.log | tail -20
echo "  （完整自检在 /tmp/start_parking.log）"

echo '[restart] 10) 现场快照'
timeout -k 5 60 python3 -u $WS/tools/fleet_peek.py 8 2>&1 | tail -8
echo "[restart] DONE  gzserver=$(pgrep -x gzserver | head -1) gzclient=$(pgrep -x gzclient | head -1)"
