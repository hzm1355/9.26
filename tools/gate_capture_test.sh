#!/bin/bash
# 卡口抓拍取证：生成临时龙门架相机 -> 给 car_1 挂真车牌 -> 按时间抽样抓帧
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

T=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
OUT=/mnt/d/desktop1/9.26.1/gate_frames
rm -rf $OUT

if ! gz model -l 2>/dev/null | grep -q gate_test; then
  echo "=== 生成 gate_test 龙门架 ==="
  timeout -k 3 40 python3 /opt/ros/humble/lib/gazebo_ros/spawn_entity.py \
    -entity gate_test -file $T/gate_test.sdf 2>&1 | tail -3
  sleep 6
fi

echo "=== 相机话题 ==="
ros2 topic list 2>/dev/null | grep gate || echo "(没有 gate 话题！)"

echo
echo "=== 给 car_1 挂 沪A12345 ==="
python3 $T/plate_request_cli.py 2>&1 | grep -E '已发送|回读|没有' || true
sleep 3
grep -a 'v2x_plate' /tmp/gzserver.log | tail -3 || true

echo
echo "=== 抽样抓帧（15 张，每 8 s 一张，覆盖一整圈）==="
timeout -k 3 220 python3 $T/grab_gate.py /gate_test/camera/image_raw $OUT 15 8 2>&1 | tail -3
echo "产出：$(ls $OUT 2>/dev/null | wc -l) 张"
