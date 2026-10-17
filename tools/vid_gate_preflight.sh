#!/bin/bash
# 任务三视频录制前预检：依赖脚本齐不齐、卡口在不在出图、车在不在跑、识别链路活不活。
# 缺任何一项就别开录——录出来的只会是"等待中……"的静默画面。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
echo "=== 1) 依赖脚本 ==="
for f in plate_request_cli.py verify_item1_custom_plate.py external_algo_demo.py \
         show_external_records.py check_export_csv.py blacklist_targeted_test.py; do
  [ -f "$TOOLS/$f" ] && echo "OK   tools/$f" || echo "MISS tools/$f"
done
for f in speed_sweep.sh set_lighting.sh restore_default.sh ensure_recognizer.sh apply_default_plates.sh; do
  [ -f "$SIM_ROOT/tools/$f" ] && echo "OK   sim tools/$f" || echo "MISS sim tools/$f"
done
echo "=== 2) 卡口抓拍图是否在更新（两次采样 mtime） ==="
for g in gate_j3_north gate_j2_south; do
  n=$(ls /tmp/v2x_gates/$g-*.png 2>/dev/null | wc -l)
  m=$(stat -c %Y /tmp/v2x_gates/$g-*.png 2>/dev/null | tail -1)
  echo "$g  张数=$n  最新mtime=$m  (现在 $(date +%s))"
done
echo "=== 3) 常驻节点与话题 ==="
timeout -k 3 25 ros2 node list 2>/dev/null | grep -aE "plate|gate|drive|signal"
echo "--- /plate/result 端点数 ---"
timeout -k 3 20 ros2 topic info /plate/result 2>/dev/null | tr -d '\r'
echo "=== 4) 车辆是否在跑（odom 线速度） ==="
for c in car_1 car_3; do
  v=$(timeout -k 3 15 ros2 topic echo /$c/odom nav_msgs/msg/Odometry 2>/dev/null | tr -d '\r' |
        grep -a -m1 -A1 "linear:" | tail -1 | sed 's/.*x: //')
  echo "$c  vx=$v"
done
echo "=== 5) 记录文件与看板 ==="
wc -l /tmp/v2x_records/plate_records.jsonl 2>/dev/null
timeout -k 3 10 curl -s -o /dev/null -w '看板 HTTP %{http_code}\n' http://localhost:8090/
timeout -k 3 10 curl -s -o /dev/null -w '取帧 HTTP %{http_code}\n' http://localhost:8080/cam/inter
echo DONE
