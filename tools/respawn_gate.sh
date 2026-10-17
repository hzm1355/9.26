#!/bin/bash
# 重跑北环卡口取景：删掉现有 gate_j3_north，注入 $1 指定的 SDF，等一次抓拍。
set -e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

SDF="${1:-/tmp/gate_v2.sdf}"

ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: ${NAME:-gate_j3_north}}" 2>&1 | grep -E 'success|status' || true
sleep 2
rm -f /tmp/v2x_gates/gate_j3_north-*.png
ros2 run gazebo_ros spawn_entity.py -entity gate_j3_north -file "$SDF" 2>&1 | tail -1
echo "--- 等待抓拍（最多 150s）---"
timeout 150 ros2 topic echo --once /plate/capture v2x_msgs/msg/PlateCapture 2>&1 | grep -E 'vehicle_id|capture_id|image_path' || echo "(超时未触发)"
ls -l /tmp/v2x_gates/ | tail -5
