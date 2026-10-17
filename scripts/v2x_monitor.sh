#!/bin/bash
# V2X ROS 监控脚本 - 显示小车和摄像头状态

source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

echo "========================================="
echo "       V2X 智能交通监控系统"
echo "========================================="
echo ""

# 显示小车位置
echo "---- 小车 car_1 状态 ----"
while true; do
    clear
    echo "========================================="
    echo "       V2X 智能交通监控系统"
    echo "========================================="
    echo ""
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    
    echo "---- 小车 car_1 状态 ----"
    ros2 topic echo /car_1/odom --once 2>/dev/null | grep -A8 "position:" | head -8
    echo ""
    
    echo "---- 摄像头状态 ----"
    echo -n "car_1 相机: "
    ros2 topic hz /car_1/camera/image_raw --window 1 2>/dev/null | grep "average rate" || echo "未发布"
    
    echo -n "speed_cam: "
    ros2 topic hz /speed_cam/speed_cam/image_raw --window 1 2>/dev/null | grep "average rate" || echo "未发布"
    
    echo -n "intersection_cam: "
    ros2 topic hz /intersection_cam/intersection_cam/image_raw --window 1 2>/dev/null | grep "average rate" || echo "未发布"
    
    echo -n "monitor_cam: "
    ros2 topic hz /monitor_cam/monitor_cam/image_raw --window 1 2>/dev/null | grep "average rate" || echo "未发布"
    echo ""
    
    echo "---- LiDAR 状态 ----"
    echo -n "scan 频率: "
    ros2 topic hz /car_1/scan --window 1 2>/dev/null | grep "average rate" || echo "未发布"
    echo ""
    
    echo "---- 控制指令 ----"
    echo "小车正在自动巡逻 (速度 0.3m/s, 转向 0.2rad/s)"
    echo "按 Ctrl+C 停止监控"
    echo "========================================="
    
    sleep 2
done
