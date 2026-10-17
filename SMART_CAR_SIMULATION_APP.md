# ROS2 智能小车算法仿真应用

基于 Gazebo + ROS2 的智能小车算法仿真平台，提供完整的虚拟环境、传感器仿真、导航算法验证、故障模拟等功能。

## 功能特性

### 1. 多车型可配置

内置多种预设车型，支持自定义物理参数：

| 车型 | 尺寸 (m) | 质量 (kg) | 最大速度 (m/s) | 加速度 (m/s²) |
|------|---------|----------|---------------|--------------|
| delivery_bot | 0.8×0.6×0.35 | 30 | 1.5 | 2.0 |
| medium_car | 1.6×0.8×0.4 | 80 | 3.0 | 4.0 |
| bus | 3.0×1.0×0.6 | 200 | 2.5 | 2.0 |
| patrol_car | 1.4×0.7×0.35 | 60 | 5.0 | 6.0 |
| tb3_burger | 0.14×0.14×0.14 | 0.83 | 0.22 | 1.0 |
| tb3_waffle_pi | 0.265×0.265×0.089 | 1.37 | 0.26 | 1.0 |

**配置参数**：
- 尺寸：长、宽、高
- 轮距、轴距、轮径
- 质量、最大速度、加速度
- 通行优先级
- 颜色

### 2. 虚拟传感器仿真

所有传感器数据通过标准 ROS2 话题输出，可直接供算法调用：

#### 激光雷达 (LiDAR)
- **话题**: `/<vehicle_name>/scan`
- **类型**: `sensor_msgs/LaserScan`
- **参数**:
  - 采样点：360/720 点
  - 范围：0.08~50m
  - 更新率：10~20Hz
  - 噪声：可配置标准差

#### 摄像头 (Camera)
- **话题**: `/<vehicle_name>/camera/image_raw`
- **类型**: `sensor_msgs/Image`
- **参数**:
  - 分辨率：320×240 ~ 800×600
  - 帧率：15~30Hz
  - 视场角：1.0~1.4 rad
  - 远裁剪：50~200m

#### IMU
- **话题**: `/<vehicle_name>/imu/data`
- **类型**: `sensor_msgs/Imu`
- **更新率**: 200~400Hz
- **数据**: 加速度、角速度、方向

#### 里程计 (Odom)
- **话题**: `/<vehicle_name>/odom`
- **类型**: `nav_msgs/Odometry`
- **来源**: gazebo_ros_p3d 插件
- **数据**: 位姿、速度

### 3. Nav2 导航算法验证

兼容 Nav2 导航框架，支持多种路径规划算法：

**启动命令**：
```bash
python3 smart_car_app.py start --nav2
```

**支持功能**：
- A* 路径规划
- Dijkstra 算法
- 定点巡航
- 多车协同行驶
- 动态避障
- 航点跟踪

**Nav2 话题**：
- `/<vehicle_name>/cmd_vel` - 速度指令
- `/<vehicle_name>/goal_pose` - 目标位姿
- `/<vehicle_name>/plan` - 规划路径

### 4. 支持导入自定义 ROS2 包

可以将外部编写的控制代码加载到仿真环境中运行：

**步骤**：

1. **创建自定义包**：
```bash
cd src
ros2 pkg create --build-type ament_python my_controller --dependencies rclpy geometry_msgs
```

2. **编写控制节点**：
```python
# src/my_controller/my_controller/controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.publisher = self.create_publisher(Twist, '/smart_car_1/cmd_vel', 10)
        timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MyController()
    rclpy.spin(node)
```

3. **编译**：
```bash
colcon build --packages-select my_controller
source install/setup.bash
```

4. **运行**：
```bash
ros2 run my_controller controller_node
```

### 5. 多车协同与轨迹记录

支持多车同时运行，模拟车流、拥堵场景，并提供轨迹记录与回放功能。

#### 轨迹记录

```bash
# 开始记录
python3 smart_car_app.py record start

# 停止记录
python3 smart_car_app.py record stop

# 保存轨迹到文件
python3 smart_car_app.py record save

# 清空缓冲
python3 smart_car_app.py record clear
```

**轨迹数据格式** (JSON)：
```json
{
  "meta": {
    "start_time": 1234567890.0,
    "end_time": 1234567950.0,
    "vehicle_count": 5
  },
  "vehicles": {
    "smart_car_1": {
      "samples": [
        [0.0, 0.0, -18.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.1],
        [0.05, 0.025, -17.95, 0.0, ...]
      ]
    }
  }
}
```

#### 轨迹回放

```bash
python3 smart_car_app.py playback ~/smart_car_trajectory.json
```

### 6. 故障模拟

通过 Gazebo 插件模拟传感器失效、数据卡顿、位姿偏移，用于算法容错测试。

#### 故障类型

| 故障类型 | 说明 | 用途 |
|---------|------|------|
| NONE | 无故障 | 恢复正常 |
| SENSOR_FAIL | 传感器失效 | 测试容错能力 |
| DATA_LAG | 数据延迟 | 测试实时性 |
| POSE_DRIFT | 位姿漂移 | 测试鲁棒性 |
| ALL | 所有故障 | 极端测试 |

#### 使用方法

```bash
# 注入传感器失效故障
python3 smart_car_app.py fault smart_car_1 SENSOR_FAIL

# 注入数据延迟故障（0.5秒）
python3 smart_car_app.py fault smart_car_2 DATA_LAG

# 注入位姿漂移故障
python3 smart_car_app.py fault smart_car_3 POSE_DRIFT

# 恢复所有车辆
python3 smart_car_app.py fault smart_car_1 NONE
```

#### 故障参数配置

在 SDF 模型中配置故障参数：
```xml
<plugin name="v2x_fault_sim" filename="libv2x_fault_sim_plugin.so">
  <enabled>false</enabled>
  <fault_type>NONE</fault_type>
  <drift_rate>0.01</drift_rate>        <!-- 位姿漂移速率 m/s -->
  <lag_seconds>0.5</lag_seconds>       <!-- 数据延迟秒数 -->
  <sensor_drop_rate>0.3</sensor_drop_rate>  <!-- 传感器丢包率 -->
</plugin>
```

## 快速开始

### 1. 环境要求

- Ubuntu 22.04 (推荐) 或 Windows 10/11 + WSL2
- ROS2 Humble
- Gazebo Classic 11.x
- Python 3.8+

### 2. 安装依赖

```bash
# 在 WSL2 中
cd v2x_traffic_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. 启动仿真

```bash
# 基本启动
python3 smart_car_app.py start

# 启动 + Nav2
python3 smart_car_app.py start --nav2

# 无界面模式（headless）
python3 smart_car_app.py start --no-gui

# 使用自定义配置
python3 smart_car_app.py start --config /path/to/cars.yaml
```

### 4. 查看状态

```bash
# 查看系统状态
python3 smart_car_app.py status

# 查看车辆配置
python3 smart_car_app.py config

# 编辑配置
python3 smart_car_app.py config --edit
```

## 使用示例

### 示例 1：基础巡航

```bash
# 启动仿真
python3 smart_car_app.py start

# 在另一个终端，发送速度指令
ros2 topic pub /smart_car_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

### 示例 2：Nav2 导航

```bash
# 启动仿真 + Nav2
python3 smart_car_app.py start --nav2

# 在 RViz2 中设置目标点
ros2 run rviz2 rviz2

# 或通过命令行发送目标
ros2 topic pub /smart_car_1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}"
```

### 示例 3：故障测试

```bash
# 启动仿真
python3 smart_car_app.py start

# 开始记录轨迹
python3 smart_car_app.py record start

# 注入故障
python3 smart_car_app.py fault smart_car_1 SENSOR_FAIL

# 运行导航算法...

# 停止记录并保存
python3 smart_car_app.py record stop
python3 smart_car_app.py record save

# 回放轨迹
python3 smart_car_app.py playback ~/smart_car_trajectory.json
```

### 示例 4：多车协同

```bash
# 启动仿真（默认 5 辆车）
python3 smart_car_app.py start

# 查看车辆列表
ros2 topic echo /model_states --field name

# 分别控制不同车辆
ros2 topic pub /smart_car_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.0}}"

ros2 topic pub /smart_car_2/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.8}, angular: {z: 0.2}}"
```

## 项目结构

```
v2x_traffic_sim/
├── smart_car_app.py              # 统一应用入口
├── src/
│   ├── smart_car_sim/            # 主仿真包
│   │   ├── launch/
│   │   │   └── smart_car_sim.launch.py
│   │   ├── config/
│   │   │   ├── smart_car_config.yaml
│   │   │   └── arena_regions.yaml
│   │   └── worlds/
│   │       └── algorithm_arena.world
│   ├── v2x_demo/                 # 演示节点
│   │   ├── v2x_demo/
│   │   │   ├── smart_car_spawner.py
│   │   │   ├── trajectory_recorder.py
│   │   │   ├── trajectory_player.py
│   │   │   └── region_trigger_node.py
│   │   └── models/
│   │       └── smart_car/
│   │           └── model.sdf.template
│   ├── v2x_gazebo_plugins/       # Gazebo 插件
│   │   └── src/
│   │       └── v2x_fault_sim_plugin.cc
│   └── v2x_msgs/                 # 自定义消息
│       └── msg/
│           ├── RegionEvent.msg
│           └── RegionState.msg
└── third_party/
    └── turtlebot3/               # TurtleBot3 模型
```

## 常见问题

### Q1: Gazebo 启动失败

**解决**：
```bash
# 检查 GAZEBO_MODEL_PATH
echo $GAZEBO_MODEL_PATH

# 手动设置
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/v2x_demo/models
```

### Q2: Nav2 无法启动

**解决**：
```bash
# 确保已安装 Nav2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### Q3: 传感器数据不更新

**解决**：
```bash
# 检查话题
ros2 topic hz /smart_car_1/scan

# 检查 Gazebo 插件
gz model -m smart_car_1 -p
```

### Q4: 轨迹文件过大

**解决**：
```bash
# 调整采样间隔
ros2 run v2x_demo trajectory_recorder.py --ros-args -p min_interval:=0.1
```

## 技术文档

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Gazebo 仿真教程](http://gazebosim.org/tutorials)
- [Nav2 导航框架](https://navigation.ros.org/)

## 许可证

MIT License

## 联系方式

如有问题或建议，请联系开发团队。
