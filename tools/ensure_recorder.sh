#!/bin/bash
# 记录器还活着吗？/plate/result 没有订阅者（记录器）就把 plate_recorder_node 拉起来。
#
# 为什么要这道闸：stop_signal.sh 是按 lib/v2x_demo/ 整片 pkill 的，车牌三件套里的
# plate_recorder_node 也在其中，而 start_plate_nodes.sh 只补生成器和识别器。
# 于是记录会停在停栈那一刻，卡口 PNG 照常生成但 JSONL 一行不再增长——
# 条目4"话题存储/数据导出"和条目1/5 的落盘核对就全成了无源之水。
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

SUB=$(ros2 topic info /plate/result 2>/dev/null | tr -d '\r' | awk '/Subscription count/{print $3}')
if [ "$(pgrep -cf 'lib/v2x[_]demo/plate[_]recorder[_]node' || true)" -ge 1 ] && [ "${FORCE:-0}" != "1" ]; then
  echo "[ensure_recorder] 在线（订阅者 ${SUB:-?}，JSONL $(wc -l < /tmp/v2x_records/plate_records.jsonl 2>/dev/null || echo 0) 行）"
  exit 0
fi

echo "[ensure_recorder] 拉起记录器（当前 /plate/result 订阅者 ${SUB:-?}）"
mkdir -p /tmp/v2x_records
setsid nohup ros2 run v2x_demo plate_recorder_node.py > /tmp/plate_recorder.log 2>&1 < /dev/null &
sleep 8
N=$(pgrep -cf 'lib/v2x[_]demo/plate[_]recorder[_]node' || true)
echo "[ensure_recorder] 实例数 $N（>1 会重复落盘，必须先杀干净）"
tail -3 /tmp/plate_recorder.log 2>/dev/null | cut -c1-120 || true
