#!/bin/bash
# 条目 2 在线验证：把内置识别器停掉，换成"外部算法"桩节点跑一段，再拉回来。
#
# 为什么非要停内置的：两个节点同时发 /plate/result 的话，一条抓拍会有两条结果，
# 记录器按 capture_id 收两条，看不出"外部算法单独接上也能跑通"。
#
# 用法：bash tools/external_algo_check.sh [桩算法运行秒数=150]
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
SEC=${1:-150}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 1) 停掉内置识别器（两类模式都要杀，杀到 /plate/result 没有发布者为止）'
for i in $(seq 1 8); do
  pids=$(pgrep -f 'lib/v2x_demo/plate_recognizer_node' || true)
  pids="$pids $(pgrep -f 'run v2x_demo plate_recognizer_node' || true)"
  pids=$(echo $pids | tr ' ' '\n' | grep -v '^$' | sort -u | tr '\n' ' ')
  [ -z "$pids" ] && break
  kill -9 $pids 2>/dev/null || true
  sleep 1
done
timeout 20 bash -c 'until [ "$(ros2 topic info /plate/result | awk "/Publisher count/{print \$3}")" = "0" ]; do sleep 1; done' \
  && echo '内置识别器已停，/plate/result 发布者 0' \
  || echo '!! 发布者没归零（可能还有别的节点在发），继续但要看输出里的 algorithm 字段分辨'

echo '=== 2) 只跑外部算法桩节点'
cd $SIM
python3 -u tools/external_algo_demo.py $SEC

echo '=== 3) 拉回内置识别器'
bash $SIM/tools/ensure_recognizer.sh

echo '=== 4) 导出记录，看外部算法那几条有没有被下游原样收下'
ros2 service call /plate/export std_srvs/srv/Trigger 2>/dev/null | tr -d '\r' | sed -n 's/.*message: //p'
python3 $SIM/tools/show_external_records.py
