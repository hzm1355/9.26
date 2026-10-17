#!/bin/bash
# 拉起测速两节点（检测 + 记录导出），并做"实例数=1、话题有发布者"的双重确认。
#
# 两道确认都不能省：
#  * 同名实例 >1 会让同一批抓拍落两行 JSONL，导出统计直接翻倍（任务三踩过，见
#    memory「重启节点要杀光所有实例」）；
#  * 刚 launch 的节点 DDS 发现还没完成，`ros2 topic info` 会瞬时读到 0 个发布者，
#    所以判活要 spin 着等（tools/speed_probe.py 里等，这里只看进程）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

# 默认重启两个节点；只想恢复阈值/点位基线时传 `speed_detect_node.py` 单独重启检测节点
# ——记录器一重启，内存里的累积就清零了，而 /speed/export 导的是内存累积（JSONL 只是追加日志）。
NODES="${*:-speed_detect_node.py speed_recorder_node.py}"

for NODE in $NODES; do
  # 先杀干净：rclpy 吞 TERM，所以 TERM 之后补 KILL。带 lib/ 前缀的匹配式不会命中本脚本自己。
  for PID in $(pgrep -f "lib/v2x_demo/$NODE" || true); do
    echo "杀掉旧实例 $NODE (pid $PID)"
    kill "$PID" 2>/dev/null || true
    sleep 1
    kill -9 "$PID" 2>/dev/null || true
  done
done

mkdir -p /tmp/v2x_records
# 顺序：$NODES 里谁在前谁先起（检测节点排在前面，记录器晚几秒不影响正确性）
for NODE in $NODES; do
  setsid nohup ros2 run v2x_demo $NODE --ros-args -p use_sim_time:=true \
    > /tmp/${NODE%.py}.log 2>&1 < /dev/null &
  sleep 3
done
sleep 5

echo "--- 实例数（每个都是 python 本体计数）---"
for NODE in $NODES; do
  N=$(pgrep -cf "lib/v2x[_]demo/${NODE%.py}" || true)
  echo "  ${NODE%.py} python 实例 $N"
done
echo "--- 检测节点日志 ---"
tail -8 /tmp/speed_detect_node.log 2>/dev/null | cut -c1-140
echo "--- 记录节点日志 ---"
tail -3 /tmp/speed_recorder_node.log 2>/dev/null | cut -c1-140
echo "--- 话题 ---"
ros2 topic list 2>/dev/null | grep -E '^/speed/' || echo "(没有 /speed/* 话题，节点没起来)"
