# 快速开始指南

## 立即可用：演示模式

无需任何配置，直接体验 GUI 功能：

```bash
cd D:\desktop1\9.26.1\v2x_traffic_sim\smart_car_gui
python demo_mode.py
```

**你将看到**：
- 深色主题的专业界面
- 5 辆小车在 50x50m 竞技场中实时运动
- 6 个功能标签页均可操作
- 2D 轨迹可视化带历史轨迹

---

## 完整使用：连接真实仿真

### 前提条件

1. **WSL2 已安装**
```bash
wsl --list --verbose
# 应显示 Ubuntu-22.04
```

2. **ROS2 环境就绪**
```bash
# 在 WSL 中
cd ~/v2x_ws
source install/setup.bash
ros2 pkg list | grep smart_car_sim
```

3. **Python 依赖已安装**
```bash
# Windows
pip install PyQt5 websocket-client PyYAML

# WSL
pip3 install websockets
```

### 启动流程

**步骤 1：启动 GUI**
```bash
python main_window.py
```

**步骤 2：启动桥接服务**
- 在"仿真控制"标签页
- 点击"启动桥接"
- 等待状态变为"已连接"（绿色）

**步骤 3：启动仿真**
- 勾选"启用 Gazebo GUI"（可选，需 VcXsrv）
- 点击"启动仿真"
- 等待 10-15 秒

**步骤 4：开始使用**
- 切换到"车辆监控"查看实时数据
- 使用"手动驾驶"控制车辆
- 在"故障模拟"中测试故障注入
- 通过"轨迹回放"回放历史轨迹

---

## 功能速览

### 1. 车辆监控
- 实时位姿（X, Y, Yaw）
- 速度、障碍物距离
- 2D 轨迹可视化

### 2. 手动驾驶
- 选择车辆
- 设置线速度/角速度
- 预设动作：前进/后退/左转/右转

### 3. 故障模拟
- 激光雷达关闭
- 相机关闭
- IMU 漂移
- 里程计打滑

### 4. 轨迹回放
- 加载 JSON 轨迹文件
- 播放/暂停/停止

### 5. 车辆配置
- 编辑 YAML 配置
- 添加/删除车辆
- 修改预设和参数

---

## 常见问题

### Q: 桥接服务无法连接？
**A**: 
1. 检查 WSL 是否运行：`wsl --list`
2. 在 WSL 中安装 websockets：`pip3 install websockets`
3. 检查端口 8765 是否被占用

### Q: 仿真无法启动？
**A**:
1. 确认 v2x_ws 已构建：`cd ~/v2x_ws && colcon build`
2. 检查 ROS2 环境：`source install/setup.bash`
3. 查看日志输出的错误信息

### Q: Gazebo GUI 无法显示？
**A**:
1. 安装 VcXsrv 并配置 `DISPLAY=:0`
2. 或取消勾选"启用 Gazebo GUI"使用无头模式

### Q: 如何使用演示模式？
**A**: 直接运行 `python demo_mode.py`，无需任何配置。

---

## 打包为 EXE

```bash
build.bat
```

生成位置：`dist/SmartCarSimCenter/SmartCarSimCenter.exe`

分发时打包整个 `dist/SmartCarSimCenter/` 目录。

---

## 文件说明

- `main_window.py` - 主程序（需要 WSL）
- `demo_mode.py` - 演示模式（独立运行）
- `wsl_bridge/ros2_bridge.py` - WSL 桥接服务
- `build.bat` - EXE 构建脚本
- `run.bat` - 快速启动脚本
- `README.md` - 完整文档
- `BUILD_SUMMARY.md` - 构建总结

---

## 技术支持

如遇问题，请检查：
1. WSL2 是否正常运行
2. ROS2 环境是否正确配置
3. v2x_ws 是否已构建
4. Python 依赖是否已安装

查看日志输出获取详细错误信息。
