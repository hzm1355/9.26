#!/bin/bash
# ROS2 智能小车仿真 - 快速启动脚本

echo "═══════════════════════════════════════════════════════════"
echo "    ROS2 智能小车算法仿真应用"
echo "═══════════════════════════════════════════════════════════"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[目录] $SCRIPT_DIR"
echo ""

# 检查 ROS2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "[错误] 未找到 ROS2 Humble"
    echo "请先安装: sudo apt install ros-humble-desktop"
    exit 1
fi

# 加载 ROS2
echo "[1/3] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash

# 加载工作空间
if [ -f "install/setup.bash" ]; then
    echo "[2/3] 加载工作空间..."
    source install/setup.bash
else
    echo "[2/3] 工作空间未编译，先编译..."
    colcon build
    source install/setup.bash
fi

echo "[3/3] 启动应用..."
echo ""

# 运行应用
python3 smart_car_app.py "$@"
