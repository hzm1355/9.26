#!/bin/bash
# 第 151 轮重启前的现场体检（只读，不改任何东西）。
# 上一轮的后台启动返回了退出码 0 但产物文件一行都没写 —— 说明 nohup ... & 那个后台
# 子 shell 随 wsl.exe 退出一起被收走了。重跑长跑之前先确认现场没被上一轮改脏。
set +e
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

echo "== 1) parking 节点实例数 =="
# 节点名必须照 src/v2x_demo/v2x_demo/ 里的真实文件名写。第一版这里填了
# v2x_plate_recognizer，实际文件叫 plate_recognizer_node.py，grep 失配就输出 0，
# 看着像"识别器没跑"，其实是我猜的名字不存在 —— 过滤失配的静默空证据。
for n in parking_shuttle_node parking_manager_node parking_recorder_node plate_recognizer_node plate_recorder_node; do
  C=$(pgrep -fc "lib/v2x[_]demo/$n" 2>/dev/null)
  echo "  $n: ${C:-0}"
done

echo "== 2) car_6 位姿 / 是否有残留驱动 =="
timeout -k 8 30 python3 -u $TOOLS/probe_state151.py

echo "== 3) 记录与抓拍基线 =="
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null |
  sed 's/^/  /'
echo "  gzserver.log 里 '抓拍 gate_in' 行数: $(grep -ac '抓拍 gate_in' /tmp/gzserver.log 2>/dev/null)"

echo "== 4) gzserver / gzclient 存活 =="
pgrep -x gzserver | sed 's/^/  gzserver pid /'
pgrep -x gzclient | sed 's/^/  gzclient pid /'
echo "DONE"
