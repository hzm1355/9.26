#!/bin/bash
# #77 隔离实验：摆位本身会不会弹射。
# 必须先杀掉 signal_drive_node，否则它和探针抢 /car_x/cmd_vel，结论无效。
# 禁 set -u：ROS 的 setup.bash 里有未定义变量。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
OUT=/mnt/d/desktop1/9.26.1/_eject77.txt

source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

# 用字符类打断模式，避免 pgrep/pkill 匹配到自己这条命令行
PIDS=$(pgrep -f 'signal[_]drive[_]node' || true)
if [ -n "$PIDS" ]; then
  echo "$PIDS" | xargs -r kill -9
  sleep 1
fi
LEFT=$(pgrep -cf 'signal[_]drive[_]node' || true)
echo "kill 后 signal_drive_node 实例数=$LEFT" | tee "$OUT"
[ "$LEFT" = "0" ] || { echo "还有实例活着，实验作废" | tee -a "$OUT"; exit 1; }

cd "$SIM"
echo '===== A：摆位前后全程零速 =====' | tee -a "$OUT"
python3 tools/place_eject_probe.py --car=car_2 --xy=11.5,1.5 --yaw=0.0 --drive=zero --verbose >> "$OUT" 2>&1 || echo "A 退出码非 0" | tee -a "$OUT"
echo '===== B：摆位前后持续给油 0.70 =====' | tee -a "$OUT"
python3 tools/place_eject_probe.py --car=car_2 --xy=11.5,1.5 --yaw=0.0 --drive=cruise --verbose >> "$OUT" 2>&1 || echo "B 退出码非 0" | tee -a "$OUT"
echo "完成，详见 $OUT"
