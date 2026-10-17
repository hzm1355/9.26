#!/usr/bin/env bash
# 修复后回归复核：对新的 20 Hz 轨迹跑横偏/实体盒相交分析。
# 不要写 set -u —— ROS 的 setup.bash 引用未定义变量。
set -e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
CSV=${1:-/tmp/signal_drive_path.csv}
echo "复核 $CSV（行数 $(wc -l < "$CSV")，时间跨度 $(awk -F, 'NR==2{print $1}' "$CSV")..$(tail -n 1 "$CSV" | cut -d, -f1)）"
python3 tools/analyze_deviation.py 1.0 "$CSV" 2>&1 | tail -45
