#!/bin/bash
# 复位风暴归因：摆好五辆车 → 起驱动跑 45 s → 回读逐 tick 路径 CSV 看有没有米级跳变。
# 目的：把"驱动只能给 0.7 m/s"和"3.5 s 内跑到 y=-19.3（≈2.2 m/s）"这两件矛盾的事
# 分开——要么速度真的被抬上去了（物理/指令问题），要么中间发生过 /set_entity_state
# 瞬移（复位逻辑自己造出了"出界"）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[storm] 1) 停驱动"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal_drive_node' || true); do
  kill "$PID" 2>/dev/null || true; sleep 1; kill -9 "$PID" 2>/dev/null || true
done
sleep 1
echo "  残留 drive 实例 $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)"

echo "[storm] 2) 摆位（recover_fleet.py 只摆位，不重启驱动）"
cd "$SIM/tools"
timeout -k 10 900 python3 -u recover_fleet.py || echo "  !! 摆位有异常车"

echo "[storm] 3) 起驱动跑 45 s"
rm -f /tmp/signal_drive_path.csv
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p config_file:=$CFG -p path_csv:=/tmp/signal_drive_path.csv \
  > /tmp/driver_probe.log 2>&1 < /dev/null &
sleep 45
echo "  drive 实例 $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)"

echo "[storm] 4) 逐 tick 轨迹体检"
timeout -k 5 60 python3 -u path_csv_report.py /tmp/signal_drive_path.csv 10
echo "[storm] 5) 驱动日志"
cut -c1-170 /tmp/driver_probe.log | tail -25
