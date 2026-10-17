#!/bin/bash
# 判别"摆位弹射"根因的一轮实验：杀驱动 -> 等全场静下来 -> 同一坐标只换摆放 z，看谁被弹。
#
# 为什么先杀驱动：signal_drive_node 的越界看门狗每 2~5 s 就把车 /set_entity_state 瞬移
# 一次（本轮日志里 20 s 内 20 次复位），它在的话每一格观测都被下一次复位污染。
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[probe] 1) 杀掉车队驱动（TERM 后补 KILL，rclpy 把 TERM 只当 shutdown 标志）"
for PAT in 'lib/v2x_demo/signal_drive_node' 'run v2x_demo signal_drive_node'; do
  for PID in $(pgrep -f "$PAT" || true); do kill "$PID" 2>/dev/null || true; done
done
sleep 2
for PAT in 'lib/v2x_demo/signal_drive_node' 'run v2x_demo signal_drive_node'; do
  for PID in $(pgrep -f "$PAT" || true); do kill -9 "$PID" 2>/dev/null || true; done
done
sleep 1
N=$(pgrep -cf 'lib/v2x[_]demo/signal[_]drive[_]node' || true)
echo "  signal_drive 残留 python 实例：${N:-0}"
[ "${N:-0}" -ne 0 ] && { echo '!! 没杀干净，停跑'; exit 1; }

echo "[probe] 2) 静置 70 s，让上一轮弹射的余速自己磨掉（没有发布者了，谁也不会再推车）"
sleep 70
timeout -k 5 60 python3 -u $D/tools/fleet_peek.py 10 2>&1 | tail -8

echo "[probe] 3) car_4 在 (8.0,-14.6) 车头朝东（该点 6 m 内没有灯杆/卡口），只换 z 各摆一次"
timeout -k 10 300 python3 -u $D/tools/place_z_probe.py \
  --car car_4 --x 8.0 --y -14.6 --yaw 0 \
  --zs 6.70,6.90,7.10,6.50 --gap 6 --obs 4

echo "[probe] 4) 再在灯杆边上摆一次（2.0,-14.6 距 tl_j2_ns 2.6 m）验证是不是重叠弹开"
timeout -k 10 200 python3 -u $D/tools/place_z_probe.py \
  --car car_4 --x 2.0 --y -14.6 --yaw 0 \
  --zs 6.70,6.70 --gap 4 --obs 4
echo "[probe] 完成"
