# 智能小车仿真控制中心

Windows GUI 应用程序，用于控制和管理 ROS2 智能小车仿真系统。

## 功能特性

### 1. 仿真控制
- 启动/停止 Gazebo 仿真
- 可选启用 Gazebo GUI 和 Nav2 导航
- 实时显示系统状态（Gazebo Server、车辆数量等）

### 2. 车辆监控
- 实时显示所有车辆位姿（X, Y, Yaw）
- 显示速度、最近障碍物距离、里程计数据
- 2D 轨迹可视化（带轨迹历史）

### 3. 手动驾驶
- 选择车辆进行手动控制
- 设置线速度和角速度
- 预设动作（前进/后退/左转/右转）
- 实时显示车辆位置

### 4. 故障模拟
- 激光雷达关闭
- 相机关闭
- IMU 漂移注入
- 里程计打滑
- 一键重置所有故障

### 5. 轨迹回放
- 加载轨迹文件（JSON）
- 播放/暂停/停止回放
- 可视化轨迹数据

### 6. 车辆配置
- 编辑车辆配置文件（YAML）
- 添加/删除车辆
- 修改预设、位置、颜色
- 保存并同步到仿真

## 系统要求

- Windows 10/11
- WSL2 + Ubuntu 22.04
- ROS 2 Humble
- Python 3.8+（Windows 端）
- 已构建的 v2x_ws 工作空间

## 安装

### 1. 安装 Python 依赖

```bash
cd smart_car_gui
pip install -r requirements.txt
```

### 2. 确保 WSL2 环境就绪

```bash
# 在 WSL2 中
cd ~/v2x_ws
colcon build
source install/setup.bash
```

### 3. 安装 WSL 桥接依赖

```bash
# 在 WSL2 中
pip3 install websockets
```

## 使用方法

### 方式一：直接运行 Python（推荐开发时使用）

```bash
# Windows 命令行
cd smart_car_gui
python main_window.py
```

或双击 `run.bat`

### 方式二：打包为 EXE

```bash
# Windows 命令行
cd smart_car_gui
build.bat
```

生成的 EXE 位于 `dist/SmartCarSimCenter/SmartCarSimCenter.exe`

### 使用流程

1. **启动 GUI**
   - 运行 `python main_window.py` 或 `SmartCarSimCenter.exe`

2. **启动桥接服务**
   - 在"仿真控制"标签页，点击"启动桥接"
   - 等待状态显示"已连接"（绿色）

3. **启动仿真**
   - 勾选"启用 Gazebo GUI"（可选，需要 VcXsrv）
   - 勾选"启用 Nav2 导航"（可选）
   - 点击"启动仿真"
   - 等待 10-15 秒，直到车辆生成完成

4. **监控车辆**
   - 切换到"车辆监控"标签页
   - 查看实时位姿、速度、传感器数据
   - 观察 2D 轨迹可视化

5. **手动驾驶**
   - 切换到"手动驾驶"标签页
   - 选择车辆
   - 设置速度或使用预设动作
   - 点击"发送"

6. **故障模拟**
   - 切换到"故障模拟"标签页
   - 选择车辆
   - 勾选故障类型并点击"应用"
   - 观察车辆行为变化

7. **轨迹回放**
   - 切换到"轨迹回放"标签页
   - 点击"加载轨迹文件"
   - 选择之前记录的 JSON 文件
   - 点击"播放"

8. **配置车辆**
   - 切换到"车辆配置"标签页
   - 浏览并加载配置文件
   - 编辑车辆参数
   - 点击"保存配置"

## 架构说明

```
Windows GUI (PyQt5)
    ↓ WebSocket (ws://localhost:8765)
WSL2 Bridge (ros2_bridge.py)
    ↓ ROS2 Topics/Services
Gazebo Simulation
```

- **Windows GUI**: 用户界面，通过 WebSocket 与桥接服务通信
- **WSL2 Bridge**: 运行在 WSL2 中，订阅 ROS2 话题，转发状态到 GUI
- **Gazebo**: 物理仿真引擎，运行在 WSL2 中

## 故障排除

### 桥接服务无法连接
- 检查 WSL2 是否正常运行
- 确认 `websockets` 已安装：`pip3 install websockets`
- 检查端口 8765 是否被占用

### 仿真无法启动
- 确认 `v2x_ws` 已构建：`cd ~/v2x_ws && colcon build`
- 检查 ROS2 环境：`source install/setup.bash`
- 查看日志输出中的错误信息

### Gazebo GUI 无法显示
- 需要安装 VcXsrv 并配置 `DISPLAY=:0`
- 或在启动时取消勾选"启用 Gazebo GUI"使用无头模式

### 车辆数据不更新
- 检查桥接服务是否已连接
- 确认仿真正在运行（查看 Gazebo Server 状态）
- 切换到其他标签页再切回"车辆监控"

## 文件结构

```
smart_car_gui/
├── main_window.py          # Windows GUI 主程序
├── wsl_bridge/
│   └── ros2_bridge.py      # WSL2 桥接服务
├── requirements.txt        # Python 依赖
├── build.bat              # EXE 构建脚本
├── run.bat                # 快速启动脚本
└── README.md              # 本文档
```

## 开发说明

### 添加新功能

1. **GUI 端**（main_window.py）
   - 在 `MainWindow` 类中添加新的标签页
   - 实现对应的 UI 和槽函数
   - 通过 `ws_client.send()` 发送命令

2. **桥接端**（ros2_bridge.py）
   - 在 `handle_command()` 中添加新的命令处理
   - 订阅新的 ROS2 话题（如需要）
   - 更新 `BridgeState` 数据结构

### 调试

```bash
# 启动桥接服务（WSL2）
cd ~/v2x_ws
source install/setup.bash
python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/smart_car_gui/wsl_bridge/ros2_bridge.py

# 启动 GUI（Windows）
python main_window.py
```

查看控制台输出以调试。

## 许可证

与主项目相同
