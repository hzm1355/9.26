#!/bin/bash
# 任务三卡口联调重启：world 里已写入两处正式卡口，需要 gzserver 重新加载。
# 当前 gzserver 已在 delete_entity 后崩溃，本脚本同时承担恢复职责。
D=/mnt/d/desktop1/9.26.1
unset ROS_DOMAIN_ID

echo '### 1/5 停止 ROS 子系统与残留 gz 进程'
bash $D/stop_signal.sh 2>&1 | tail -1
pkill -f plate_gen_node 2>/dev/null || true
for i in 1 2 3; do
  P="$(pgrep -x gzserver || true) $(pgrep -x gzclient || true)"
  [ "${P// /}" = "" ] && break
  kill -9 $P 2>/dev/null || true
  sleep 3
done
echo "残留: server='$(pgrep -x gzserver || true)' client='$(pgrep -x gzclient || true)'"

echo '### 2/5 启动 gzserver（加载含卡口的新 world）'
rm -f /tmp/v2x_gates/*.png 2>/dev/null || true
bash $D/v2x_traffic_sim/scripts/start_gazebo_city.sh 2>&1 | tail -2
sleep 8
echo '### 3/5 启动 gzclient'
bash $D/v2x_traffic_sim/scripts/start_gzclient.sh 2>&1 | tail -2

echo '### 4/5 等待世界加载，校验卡口模型'
sleep 55
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
timeout -k 3 40 python3 -u /tmp/list_models.py > /tmp/models6.txt 2>&1 || true
grep -E 'gate_|plate_manager|car_' /tmp/models6.txt || echo '(模型列表读取失败)'

echo '### 5/5 拉起信号子系统 + 车牌生成/识别'
bash $D/start_signal.sh 2>&1 | grep -E 'SignalDrive|recorder|groups|ERROR' | head -6
setsid nohup ros2 run v2x_demo plate_gen_node.py > /tmp/plate_gen.log 2>&1 < /dev/null &
setsid nohup ros2 run v2x_demo plate_recognizer_node.py > /tmp/plate_rec.log 2>&1 < /dev/null &
sleep 6
echo "plate_gen: $(pgrep -af plate_gen_node | head -1 | cut -c1-60)"
echo "recognizer: $(pgrep -af plate_recognizer_node | head -1 | cut -c1-60)"

# 重启后必须把五副默认牌重新下发：/plate/request 是一次性指令，plate_gen 重启后
# 车上只剩占位牌纹理，卡口拍到的一律是"无牌"。这个坑静默得很——识别全返回空号、
# 真值列空，跑十五分钟速度扫参拿到的全是废数据。
echo '### 6/6 下发五副默认车牌'
bash $D/v2x_traffic_sim/tools/apply_default_plates.sh || \
  echo "  !! 贴牌不完整，卡口测试会拍到无牌车 —— 别开始扫参"
echo "gate 话题: $(ros2 topic list | grep -c gate)"

# 记录/导出/黑名单节点（条目 4/5）也要跟着栈起来，否则重启后抓拍只出结果不落盘，
# gate_lighting_test 之类的事后统计无从分组。它不依赖 gzserver，放最后拉起。
setsid nohup ros2 run v2x_demo plate_recorder_node.py > /tmp/plate_recorder.log 2>&1 < /dev/null &
sleep 4
echo "recorder: $(pgrep -af plate_recorder_node | head -1 | cut -c1-60)"
echo '[restart6] DONE'
