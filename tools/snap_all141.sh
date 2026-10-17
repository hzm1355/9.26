#!/bin/bash
# 相机到底活不活，只认 best_effort 订阅拿到的那一帧；拿到就落到 D 盘好直接看图。
#
# 为什么不用 ros2 topic hz：Humble 的 hz 没有 QoS 选项，默认 reliable 订 best_effort
# 的 sensor 话题恒为"没有帧"（上一轮就是这样把 8 路相机全读成断流，而 :8080 同一时刻
# 还在回 90 KB 的 JPEG）。判活要么用带 --qos-profile 的 echo，要么用专门的订阅脚本。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
OUT=/mnt/d/desktop1/9.26.1/_live141
mkdir -p "$OUT"

for T in /parking/gate_in/image_raw /parking/gate_out/image_raw \
         /gate_j3_north/gate_cam/image_raw; do
  N=$(basename $(dirname $T))
  echo "--- $T ---"
  timeout -k 3 40 python3 $TOOLS/snap_gate140.py "$T" "$OUT/$N.png" 30 2>&1 | sed 's/^/  /'
done

echo "--- 落盘清单 ---"
ls -l "$OUT" | sed 's/^/  /'
