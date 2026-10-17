#!/bin/bash
# 速度扫参前的状态自检：五车驱动 / 三个车牌节点 / 看板 / 当前光照档
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "gzserver: $(pgrep -x gzserver || echo DEAD)"
for n in plate_gen_node plate_recognizer_node plate_recorder_node signal_drive_node traffic_light_manager live_view_server; do
  echo "$n -> $(pgrep -f "$n" | tr '\n' ' ')"
done
echo '--- install 里的 config 是不是软链到源码（决定改档要不要重建）---'
ls -l ~/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml
echo '--- 当前光照档 ---'
timeout -k 2 6 ros2 topic echo /gate/lighting/current --once 2>&1 | head -3
echo '--- 抓拍图目录 ---'
ls /tmp/v2x_plates 2>/dev/null | wc -l
echo '--- plate_gen 日志尾部（看 lit 状态）---'
tail -3 /tmp/plate_gen.log 2>/dev/null
