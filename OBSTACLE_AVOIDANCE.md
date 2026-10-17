# 智能小车避障功能

基于 LiDAR 的实时避障控制系统，适用于 ROS2 + Gazebo 仿真环境。

## 功能特性

- **实时避障**：10Hz 控制频率，快速响应障碍物
- **扇区分析**：将 LiDAR 扫描分为 5 个扇区（左前、正前、右前、左、右）
- **智能决策**：
  - 正前方有障碍 → 停止并转向空旷侧
  - 前方较近 → 减速并根据侧方情况微调方向
  - 侧方有障碍 → 向反方向微调
  - 无 obstacle → 直线前进
- **可配置参数**：速度、停止距离、减速距离均可调整

## 快速开始

### 1. 编译项目

```bash
cd ~/v2x_ws
colcon build --packages-select v2x_demo
source install/setup.bash
```

### 2. 启动仿真

```bash
# 启动智能小车仿真
python3 smart_car_app.py start

# 或启动带 Nav2 的仿真
python3 smart_car_app.py start --nav2
```

### 3. 启动避障

```bash
# 方式 1：使用统一入口（推荐）
python3 smart_car_app.py avoid smart_car_1

# 方式 2：自定义参数
python3 smart_car_app.py avoid smart_car_1 --speed 0.3 --stop 1.0

# 方式 3：使用 launch 文件
ros2 launch v2x_demo obstacle_avoidance.launch.py

# 方式 4：使用 shell 脚本
bash run_obstacle_avoidance.sh smart_car_1 0.5 0.8

# 方式 5：直接运行节点
ros2 run v2x_demo obstacle_avoidance.py --ros-args \
  -p car_name:=smart_car_1 \
  -p max_speed:=0.5 \
  -p stop_distance:=0.8
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `car_name` | `smart_car_1` | 小车名称（话题前缀） |
| `max_speed` | `0.5` | 最大前进速度 (m/s) |
| `turn_speed` | `0.8` | 转向速度 (rad/s) |
| `stop_distance` | `0.8` | 停止距离阈值 (m) |
| `slow_distance` | `1.5` | 减速距离阈值 (m) |
| `scan_timeout` | `1.0` | LiDAR 数据超时时间 (s) |

## 话题接口

### 订阅

- `/<car_name>/scan` (`sensor_msgs/LaserScan`)：LiDAR 扫描数据

### 发布

- `/<car_name>/cmd_vel` (`geometry_msgs/Twist`)：速度指令

## 算法详解

### 扇区划分

```
        左前 (FL)    正前 (F)    右前 (FR)
           \          |          /
            \         |         /
             \        |        /
    左 (L) ---\-------|-------/--- 右 (R)
               \      |      /
                \     |     /
                 \    |    /
                  \   |   /
                   \  |  /
                    \ | /
                     \|/
                      车
```

### 决策逻辑

```python
if front < stop_distance:
    # 正前方太近，停止并转向空旷侧
    stop()
    turn_to_open_side()
elif front < slow_distance:
    # 前方较近，减速
    speed = max_speed * (front - stop) / (slow - stop)
    if side_closer:
       微调方向 ()
elif left < stop or right < stop:
    # 侧方有障碍，微调方向
    speed = max_speed * 0.7
    turn_away_from_obstacle()
else:
    # 无 obstacle，直线前进
    speed = max_speed
```

## 使用示例

### 示例 1：基础避障

```bash
# 终端 1：启动仿真
python3 smart_car_app.py start

# 终端 2：启动避障
python3 smart_car_app.py avoid smart_car_1
```

小车会自动前进，遇到障碍物时停止并转向。

### 示例 2：保守避障（更大安全距离）

```bash
python3 smart_car_app.py avoid smart_car_1 --speed 0.3 --stop 1.2
```

### 示例 3：多车避障

```bash
# 终端 1：启动仿真（默认 5 辆车）
python3 smart_car_app.py start

# 终端 2：为每辆车启动避障
python3 smart_car_app.py avoid smart_car_1 --speed 0.4
python3 smart_car_app.py avoid smart_car_2 --speed 0.4
python3 smart_car_app.py avoid smart_car_3 --speed 0.4
```

### 示例 4：结合 Nav2 导航

```bash
# 终端 1：启动仿真 + Nav2
python3 smart_car_app.py start --nav2

# 终端 2：启动避障（作为局部避障层）
python3 smart_car_app.py avoid smart_car_1 --speed 0.5

# 终端 3：发送导航目标
bash send_goal.sh 10.0 5.0
```

## 故障排除

### Q1: 小车不动

**检查 LiDAR 数据**：
```bash
ros2 topic hz /smart_car_1/scan
```

如果无数据，检查 Gazebo 中 LiDAR 插件是否正常。

### Q2: 小车频繁停止

**调整停止距离**：
```bash
python3 smart_car_app.py avoid smart_car_1 --stop 0.5
```

### Q3: 避障反应慢

**检查控制频率**：
```bash
ros2 topic hz /smart_car_1/cmd_vel
```

正常应为 10Hz。如果过低，检查系统负载。

### Q4: 小车撞墙

**检查 LiDAR 安装位置**：
```bash
ros2 topic echo /smart_car_1/scan --once
```

确保 `ranges` 数据有效（非 inf 或 NaN）。

## 扩展开发

### 添加新的避障策略

在 `obstacle_avoidance.py` 中修改 `control_loop()` 方法：

```python
def control_loop(self):
    sectors = self.analyze_scan()
    if sectors is None:
        return

    # 自定义决策逻辑
    if sectors['front'] < 0.5:
        # 紧急停止
        self.publish_velocity(0.0, 0.0)
    else:
        # 自定义行为
        pass
```

### 集成到自定义节点

```python
from v2x_demo.obstacle_avoidance import ObstacleAvoidance

# 在你的节点中使用
avoidance = ObstacleAvoidance()
avoidance.car_name = 'my_robot'
avoidance.max_speed = 0.3
```

## 相关文件

- `src/v2x_demo/v2x_demo/obstacle_avoidance.py` - 避障节点
- `src/v2x_demo/launch/obstacle_avoidance.launch.py` - Launch 文件
- `run_obstacle_avoidance.sh` - 快速启动脚本
- `smart_car_app.py` - 统一入口（avoid 命令）

## 参考资料

- [ROS2 Navigation2](https://navigation.ros.org/)
- [Gazebo LiDAR 传感器](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [TurtleBot3 避障示例](../third_party/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_obstacle_detection/)
