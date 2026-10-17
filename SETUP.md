# 路线 A 环境搭建清单（Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11）

> 本清单为 V2X 智能交通虚拟仿真软件项目的**交付主平台**搭建指南。
> 目标栈：Ubuntu 22.04 (Jammy) + ROS 2 Humble LTS + Gazebo Classic 11 + `gazebo_ros_pkgs` + Nav2 + `ros2_control`。
> 全部采用官方 apt 二进制安装（最稳、最快），命令可直接复制执行。

---

## 阶段 0：平台策略（Windows 主机承载 Linux）

由于交付主平台为 Linux，Windows 主机推荐用 **WSL2** 承载 Ubuntu 22.04。

| 方案 | 说明 | 推荐度 |
|------|------|:---:|
| WSL2 + Ubuntu 22.04 | Win11/新版 Win10 自带 WSLg，可直接跑 Gazebo GUI | ★★★ |
| Docker + X Server | 容器化 ROS 2 Humble，需配 X11/VcXsrv 转发 GUI | ★★ |
| 双系统 / 虚拟机 | VMware/VirtualBox 装 Ubuntu，需调优 3D 加速 | ★★ |

```powershell
# Windows PowerShell（管理员）——启用 WSL2 并安装 Ubuntu 22.04
wsl --install -d Ubuntu-22.04
# 重启后进入 Ubuntu，创建非 root 用户
# 验证图形转发（WSLg）：
#   sudo apt install -y x11-apps && xeyes
```

---

## 阶段 1：系统准备（Ubuntu 22.04 内执行）

```bash
# 1.1 设置 UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 1.2 启用 universe 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# 1.3 添加 ROS 2 官方 apt 源
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

---

## 阶段 2：ROS 2 Humble 安装

```bash
# 2.1 桌面版（含 RViz2、rqt、demos、tutorials）—— 推荐
sudo apt install -y ros-humble-desktop

# 2.2 开发工具：colcon / rosdep / vcstool
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-vcstool python3-pip build-essential
sudo rosdep init 2>/dev/null || true
rosdep update

# 2.3 环境变量自动加载
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc

# 2.4 验证
ros2 --help && ros2 doctor --report | head -n 20
```

---

## 阶段 3：Gazebo Classic 11 + gazebo_ros_pkgs（核心桥接）

```bash
# 3.1 安装桥接包（会自动拉取 Gazebo Classic 11 依赖）
sudo apt install -y ros-humble-gazebo-ros-pkgs

# 3.2 显式安装 Gazebo Classic 11
sudo apt install -y gazebo11 libgazebo11 libgazebo-dev

# 3.3 验证 Gazebo 可独立启动
gazebo --version          # 应显示 version 11.x
gazebo                    # 打开空世界 GUI，Ctrl+C 关闭

# 3.4 验证 ROS2↔Gazebo 桥接（关键）
ros2 launch gazebo_ros gzserver.launch.py &
sleep 8
ros2 topic list           # 能看到 /clock → 桥接层正常
pkill -9 gzserver
```

> ✅ 这是项目可行性的第一道关卡：`/clock` 出现即说明 `gazebo_ros_pkgs` 桥接打通。

---

## 阶段 4：Nav2 导航栈（A*/Dijkstra 路径规划）

```bash
# 4.1 安装 Nav2 及启动包
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# 4.2 安装 TurtleBot3 仿真（快速验证 Nav2 的载体）
sudo apt install -y ros-humble-turtlebot3*
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc   # waffle 带激光雷达+相机
source ~/.bashrc

# 4.3 验证 Nav2 演示世界
ros2 launch nav2_bringup tb3_simulation_launch.py
```

> Nav2 默认规划器 `NavFn`/`Smac` 即实现 Dijkstra/A*，满足协议"兼容 A*、Dijkstra"要求。

---

## 阶段 5：ros2_control + gazebo_ros2_control（车辆动力学控制）

```bash
# 5.1 控制框架与常用控制器
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control

# 5.2 差速/阿克曼控制器
sudo apt install -y ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster
sudo apt install -y ros-humble-ackermann-steering-controller 2>/dev/null || \
  echo "若 apt 无此包，用 steer_bot 源码编译（见下）"

# 5.3 参考实现（Nav2 官方推荐阿克曼车）——源码编译
mkdir -p ~/v2x_ws/src && cd ~/v2x_ws/src
git clone https://github.com/srmainwaring/steer_bot.git
cd ~/v2x_ws && rosdep install --from-paths src -i -y
colcon build --symlink-install
```

---

## 阶段 6：项目工作区 + 复用工作区已有模型

```bash
# 6.1 建立项目工作区
mkdir -p ~/v2x_ws/src

# 6.2 让 Gazebo 找到工作区模型库（prius_hybrid / hokuyo / kinect / stop_light 等）
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master' >> ~/.bashrc
source ~/.bashrc

# 6.3 本项目骨架放入工作区后编译（见 README.md）
# cp -r v2x_traffic_sim/src/v2x_demo ~/v2x_ws/src/
# cd ~/v2x_ws && colcon build --symlink-install && source install/setup.bash

# 6.4 每次开发前 source
echo "source ~/v2x_ws/install/setup.bash" >> ~/.bashrc
```

---

## 阶段 7：最小闭环验证（Spike）—— 5 个检查点

> 目标：用最少工作量证明"小车 + 传感器话题 + 导航 + 控制"全链路贯通。建议 3–5 人日完成。
> 本仓库 `v2x_demo` 提供了自定义 demo（prius + 雷达 + 红绿灯 + 十字路口），可直接跑检查点 2–5。

### 检查点 1：Gazebo 启动 + 时钟同步
```bash
ros2 launch gazebo_ros gzserver.launch.py &
ros2 topic echo /clock --once     # 有仿真时间输出 = 桥接正常
```

### 检查点 2：车辆 + 传感器话题输出
```bash
# 方式一（TurtleBot3）：
ros2 launch turtlebot3_gazebo empty_world.launch.py
# 方式二（本项目自定义 prius demo）：
ros2 launch v2x_demo v2x_bringup.launch.py

# 核对协议要求的传感器话题：
ros2 topic list | grep -E "scan|imu|odom|cmd_vel|camera|distance"
ros2 topic hz /scan                # 激光雷达有频率输出
ros2 topic echo /odom --once       # 里程计
ros2 topic echo /imu/data --once   # IMU
```
> 对应协议 §三.(一).2：仿真激光雷达、摄像头、IMU、里程计，通过标准 ROS 2 话题输出。

### 检查点 3：遥控驱动小车（控制指令下行）
```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 或本项目：ros2 launch v2x_demo v2x_teleop.launch.py
```

### 检查点 4：SLAM 建图 + Nav2 自主导航（核心）
```bash
# 4a. 建图
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
# 遥控跑一圈建图后保存：
ros2 run nav2_map_server map_saver_cli -f ~/map

# 4b. 自主导航（RViz 中点 "Nav2 Goal" 设目标点）
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=$HOME/map.yaml
# 本项目：ros2 launch v2x_demo v2x_nav2.launch.py map:=$HOME/map.yaml
```
> 对应协议 §三.(一).3：Nav2 导航验证、A*/Dijkstra、定点巡航、避障。

### 检查点 5：ros2_control / 阿克曼控制器闭环（车辆动力学）
```bash
cd ~/v2x_ws && source install/setup.bash
ros2 launch steer_bot steer_bot.launch.py       # 参考实现
ros2 control list_controllers                   # 核对控制器已加载
ros2 topic echo /cmd_vel --once                  # 下发速度→车辆按运动学行驶
```
> 对应协议 §三.(一).1：物理参数配置（轮距、最大速度、加速度）由控制器/驱动插件管理。

---

## 阶段 8：环境搭建完成度自查表

| 组件 | 安装命令关键字 | 验证方式 | 状态 |
|------|--------------|---------|:---:|
| Ubuntu 22.04 (WSL2) | `wsl --install` | `lsb_release -a` | ☐ |
| ROS 2 Humble Desktop | `ros-humble-desktop` | `ros2 doctor` | ☐ |
| Gazebo Classic 11 | `gazebo11` | `gazebo --version` = 11.x | ☐ |
| gazebo_ros_pkgs 桥接 | `ros-humble-gazebo-ros-pkgs` | `/clock` 话题存在 | ☐ |
| Nav2 | `ros-humble-navigation2` | Nav2 Goal 自主导航成功 | ☐ |
| ros2_control | `ros-humble-ros2-control` | `ros2 control list_controllers` | ☐ |
| gazebo_ros2_control | `ros-humble-gazebo-ros2-control` | 控制器驱动仿真车 | ☐ |
| TurtleBot3 仿真 | `ros-humble-turtlebot3*` | 传感器话题齐全 | ☐ |
| 工作区模型复用 | `GAZEBO_MODEL_PATH` | prius/stop_light 可加载 | ☐ |

---

## 阶段 9：常见坑与规避

| 问题 | 原因 | 解决 |
|------|------|------|
| WSL2 中 Gazebo GUI 黑屏/不显示 | WSLg 未启用或 GPU 驱动缺失 | 更新 Windows + 安装 NVIDIA WSL 驱动；或 `export LIBGL_ALWAYS_SOFTWARE=1` 软渲染 |
| `ros2 topic list` 看不到 Gazebo 话题 | 未 source 或 `ROS_DOMAIN_ID` 不一致 | 确认已 source；统一 `ROS_DOMAIN_ID` |
| `gazebo_ros_pkgs` 找不到 | Humble 下 apt 包名 | 用 `ros-humble-gazebo-ros-pkgs`（勿用 rolling） |
| 模型加载失败 | `GAZEBO_MODEL_PATH` 未设 | 阶段 6.2 写入 .bashrc 后重开终端 |
| Python 版本冲突 | 工作区 rolling 用 Python 3.14 | 路线 A 务必用 Humble 的 Python 3.10，勿混用 rolling 环境 |

---

## ⚠️ 与现有工作区环境的关系

工作区里的 `ros2-rolling`（pixi + Python 3.14 + rolling）与本清单的 **Humble 环境是两套独立栈，不可混用**：
- 另建干净的 Ubuntu 22.04 + Humble 环境（WSL2/独立机），**不要复用 rolling**；
- 工作区的 `gazebo-classic-gazebo11` 源码与 `gazebo_models-master` 模型**可直接复用**（同为 Classic）；
- 若必须用 rolling 源码构建，则与协议"Humble LTS"要求冲突，需回到路线决策重新确认。
