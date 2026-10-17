#!/bin/bash
# 测速记录器还活着吗？没活着就拉起来。
# 与 ensure_recorder.sh 同一道闸的理由：stop/restart 类脚本按 lib/v2x_demo/ 整片 pkill，
# speed_recorder_node 也在其中，而 start_speed_nodes.sh 只在"两个都缺"时才补——
# 记录一停，/speed/export 与条目 5 的落盘核对就成了无源之水。
#
# 判活只看进程数：/speed/capture 的订阅者除了记录器还有 :8090 看板，拿订阅者数判活
# 会把"看板在、记录器没了"读成在线。
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

N=$(pgrep -cf 'lib/v2x[_]demo/speed[_]recorder[_]node' || true)
# 文件还不存在时 `wc -l < 不存在的文件` 会先把 bash 的重定向报错打到 stderr
# （窗口清零那一步刚把 JSONL 归档走），所以先判存在。
lines() { [ -f "$1" ] && wc -l < "$1" || echo 0; }
if [ "${N:-0}" -ge 1 ] && [ "${FORCE:-0}" != "1" ]; then
  echo "[ensure_speed_rec] 在线（实例 $N，JSONL 瞬时 $(lines /tmp/v2x_records/speed_captures.jsonl) / 区间 $(lines /tmp/v2x_records/speed_segments.jsonl) 行）"
  exit 0
fi

if [ "${N:-0}" -gt 0 ]; then
  # FORCE=1 时哪怕只有 1 个实例也要杀：记录器一重启内存累积才清零，而窗口清零正是
  # verify_speed.sh 开片前要的。早先这里写成 `-gt 1`，单实例直接被跳过，
  # 结果新拉起的和旧的并存，同一批抓拍落了两行 JSONL。
  echo "[ensure_speed_rec] 先杀干净旧实例（当前 $N 个，重复实例会把同一批抓拍落两行）"
  for PID in $(pgrep -f 'lib/v2x[_]demo/speed[_]recorder[_]node' || true); do
    kill "$PID" 2>/dev/null || true; sleep 1; kill -9 "$PID" 2>/dev/null || true
  done
fi

echo "[ensure_speed_rec] 拉起测速记录器"
mkdir -p /tmp/v2x_records
setsid nohup ros2 run v2x_demo speed_recorder_node.py --ros-args -p use_sim_time:=true \
  > /tmp/speed_recorder_node.log 2>&1 < /dev/null &
sleep 8
N=$(pgrep -cf 'lib/v2x[_]demo/speed[_]recorder[_]node' || true)
SUB=$(ros2 topic info /speed/capture 2>/dev/null | tr -d '\r' | awk '/Subscription count/{print $3}')
echo "[ensure_speed_rec] 实例数 $N（重复落盘看这个，必须=1），/speed/capture 订阅者 ${SUB:-?}"
echo "                 （订阅者含 :8090 看板，记录器+看板=2 是正常值，不是重复实例）"
tail -2 /tmp/speed_recorder_node.log 2>/dev/null | cut -c1-120 || true
