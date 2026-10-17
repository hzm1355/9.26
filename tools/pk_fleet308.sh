#!/bin/bash
# 车队位姿全景 + gzserver 存活时长：用来分清"车自己动了"和"world 被重启/重 spawn 过"。
# 判据口径：gz model -m X -p 不存在时回 "Unable ..."（那句里也带数字，不能按有没有数字判）。
set +e
for M in car_1 car_2 car_3 car_4 car_5 car_6 car_7; do
  P=$(gz model -m $M -p 2>&1 | head -2 | tr -d '\n' | cut -c1-58)
  printf '%-7s %s\n' "$M" "$P"
done
echo "--- gzserver/gzclient 进程 ---"
ps -o pid,etimes,cmd -C gzserver 2>/dev/null | head -3
pgrep -af gzclient | head -2 | cut -c1-120
echo "--- 停车场节点 ---"
for N in parking_manager_node parking_recorder_node parking_shuttle_node; do
  echo "  $N: $(pgrep -cf "lib/v2x[_]demo/${N}")"
done
echo "--- JSONL 行数 ---"
wc -l /tmp/v2x_parking/*.jsonl 2>/dev/null
echo "--- shuttle 日志尾（最后一次跑的那轮）---"
tail -6 /tmp/shuttle_car_3.log 2>/dev/null | cut -c1-160
