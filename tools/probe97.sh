#!/bin/bash
# 只读取证第二轮：car_6 是怎么从出场车道跑到 (32.39,11.42) 底朝天的，
# 以及 gate_out 抓拍为什么失败（#78 的 rearm_on_fail 到底有没有在重试）。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 1) 节点日志文件都在哪 ==="
ls -la /tmp/parking_*.log /tmp/plate_rec*.log 2>/dev/null | sed 's/^/  /'

echo "=== 2) shuttle 日志尾 100 行（阶段推进/退回/停等/告警）==="
for F in /tmp/parking_shuttle_node.log; do
  [ -f "$F" ] || continue
  echo "  -- $F（$(wc -l < "$F") 行）"
  tail -n 100 "$F" | cut -c1-200 | sed 's/^/    /'
done

echo "=== 3) manager 日志：gate_out 抓拍/重试/挂起（grep 出场关键字）==="
if [ -f /tmp/parking_manager_node.log ]; then
  echo "  -- 行数 $(wc -l < /tmp/parking_manager_node.log)"
  grep -anE 'gate_out|核验|抓拍|重拍|rearm|拒绝|挂起|人工' /tmp/parking_manager_node.log |
      tail -n 40 | cut -c1-200 | sed 's/^/    /'
fi

echo "=== 4) 识别器日志尾（plate_recognizer：算法有没有回结果）==="
tail -n 25 /tmp/plate_rec.log 2>/dev/null | cut -c1-200 | sed 's/^/  /'

echo "=== 5) 出场实拍帧目录（抓拍到底出了几张图、多大）==="
ls -la /tmp/v2x_parking_out/ 2>/dev/null | tail -12 | sed 's/^/  /'
ls -la /tmp/v2x_parking_in/ 2>/dev/null | tail -6 | sed 's/^/  /'

echo "=== 6) gate_in 状态再读一次（上一轮报「还没发布」，可能是发现竞态）==="
for i in 1 2 3; do
  timeout -k 3 20 ros2 topic echo /parking/gate/gate_in/state --qos-profile sensor_data --no-daemon --once \
      > /tmp/p97_in$i.txt 2>&1
  echo -n "  第 $i 次（$(wc -l < /tmp/p97_in$i.txt) 行）："
  tr -d '\n' < /tmp/p97_in$i.txt | cut -c1-230
  echo
  sleep 3
done

echo "=== 7) 其它车有没有在动（car_6 会不会是被撞飞的）==="
for M in car_1 car_2 car_3 car_4 car_5 car_6; do
  echo "  $M: $(timeout -k 3 15 gz model -m $M -p 2>/dev/null | head -1)"
done
exit 0
