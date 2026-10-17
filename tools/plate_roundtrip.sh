#!/bin/bash
# 车牌往返测试：拉起 plate_gen_node -> 发一条 /plate/request -> 看插件与日志结果
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

T=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

if pgrep -f '[p]late_gen_node' > /dev/null; then
  echo "plate_gen_node 已在运行"
else
  setsid nohup python3 /home/hzm/v2x_ws/install/v2x_demo/lib/v2x_demo/plate_gen_node.py \
    > /tmp/plate_gen.log 2>&1 < /dev/null &
  sleep 4
  echo "plate_gen_node pid: $(pgrep -f '[p]late_gen_node' | head -1)"
fi

echo "=== plate_gen.log ==="
tail -5 /tmp/plate_gen.log

echo
echo "=== 发送换牌请求 ==="
python3 $T/plate_request_cli.py 2>&1 | tail -6

echo
echo "=== 生成的纹理目录 ==="
ls -t /tmp/v2x_plates/ | head -4

echo
echo "=== gzserver 里的 v2x_plate 日志 ==="
grep 'v2x_plate' /tmp/gzserver.log | tail -8

echo
echo "=== /plate/number 真值话题 ==="
timeout -k 2 8 ros2 topic echo --once /plate/number std_msgs/msg/String 2>&1 | head -3 || true
