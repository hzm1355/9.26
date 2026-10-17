#!/usr/bin/env bash
# 检查点 4 诊断脚本：TF 树 + slam 参数 + 话题状态
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash 2>/dev/null

echo "===== 1. 相关进程 ====="
pgrep -af 'gzserver|gzclient|slam_toolbox|static_transform|component_container|rviz2|spawn_entity' | sed 's#/home/hzm/v2x_ws/install/##'

echo ""
echo "===== 2. ROS 节点列表 ====="
ros2 node list

echo ""
echo "===== 3. slam_toolbox 关键参数 (节点名 /slam_toolbox) ====="
for p in base_frame odom_frame map_frame scan_topic transform_timeout tf_buffer_duration minimum_travel_distance minimum_travel_heading scan_buffer_size; do
  printf "%-26s : " "$p"
  ros2 param get /slam_toolbox "$p" 2>&1 | tail -1
done

echo ""
echo "===== 4. TF 树 (tf2_tools view_frames 文本版) ====="
echo "-- 用 tf2_echo 逐条验证关键链路 --"
timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5
echo "----"
timeout 5 ros2 run tf2_ros tf2_echo base_link laser 2>&1 | head -5

echo ""
echo "===== 5. /scan 消息头 frame_id ====="
timeout 5 ros2 topic echo /scan --field header.frame_id --once 2>&1 | head -3

echo ""
echo "===== 6. /map 是否有数据 ====="
timeout 6 ros2 topic echo /map --field info --once 2>&1 | head -8

echo ""
echo "===== 7. slam 节点最近日志（若在跑） ====="
echo "(需从启动终端查看，此处略)"
