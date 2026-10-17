#!/bin/bash
# 识别器还活着吗？/plate/result 没有发布者就把它拉起来，否则退出码非 0。
#
# 为什么要这道闸：gate_lighting_test.py 和 speed_sweep 都是"切预设 -> 等识别结果"的
# 长跑脚本，而识别器是个会被单张抓拍带崩的进程（bbox_* 是 uint16，牌贴到画面边缘时
# minAreaRect 的角点 x/y 为负，赋值直接 AssertionError 崩掉）。它一死，
# 十几分钟的轮次只会静默收 0 行，表里连"为什么没数"都看不出来。
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

pubs() {
  ros2 topic info /plate/result 2>/dev/null | tr -d '\r' | awk '/Publisher count/{print $3}'
}

PUB=$(pubs || true)
if [ "${PUB:-0}" -ge 1 ] && [ "${FORCE:-0}" != "1" ]; then
  echo "[ensure_recognizer] 在线（/plate/result 发布者 $PUB）"
  exit 0
fi

# FORCE=1：刚重建过 v2x_demo 时必须用——进程活着不代表它跑的是新代码，
# python 节点改完 src 后仍会把旧模块留在内存里。
echo "[ensure_recognizer] ${FORCE:+强制}重启识别器（当前发布者 ${PUB:-0}）"
for pid in $(pgrep -f 'lib/v2x_demo/plate_recognizer_node' || true) \
           $(pgrep -f 'run v2x_demo plate_recognizer_node' || true); do
  kill -9 "$pid" 2>/dev/null || true   # rclpy 把 TERM 只当 shutdown 标志
done
sleep 1
setsid nohup ros2 run v2x_demo plate_recognizer_node.py > /tmp/plate_rec.log 2>&1 < /dev/null &
sleep 6
PUB=$(pubs || true)
if [ "${PUB:-0}" -ge 1 ]; then
  echo "[ensure_recognizer] 已拉起（发布者 $PUB）"
  exit 0
fi
echo '[ensure_recognizer] 仍然没有发布者，看 /tmp/plate_rec.log：'
tail -8 /tmp/plate_rec.log | tr -d '\r'
exit 1
