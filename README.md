# v2x_traffic_sim —— V2X 智能交通虚拟仿真（路线 A 起点工程）

本目录是 **V2X 智能交通虚拟仿真软件** 的开发起点，采用 **路线 A**：
`Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + gazebo_ros_pkgs + Nav2 + ros2_control`。

- 环境搭建：见 [`SETUP.md`](./SETUP.md)
- 最小工程骨架：`src/v2x_demo`（一个可编译运行的 ROS 2 功能包）

> 本骨架用于验证「检查点 2–5」最小闭环，是后续六大业务模块的开发基座，**并非最终交付物**。

---

## 本仓库的范围（代码 + 文本）

本仓库只收 **代码与文本资源**：四个 ROS 2 功能包的 `*.py / *.cpp / *.msg / *.srv / *.world / *.sdf / *.yaml / launch`、
`tools/` 下的取证与生成脚本、`scripts/` 下的启动脚本。

以下内容由 `.gitignore` 排除，克隆后需自行补齐才能跑起来：

| 缺失项 | 从哪来 |
| --- | --- |
| 沙盘建筑网格 `room_20.obj`、`room_33.obj`（各 471 MB，超过 GitHub 单文件 100 MB 上限） | 资产仓库 **9.26.2**（Git LFS），把其中 `src/smart_car_sim/models/` 覆盖回本工程同名目录 |
| 其余网格 / 贴图（`*.obj *.dae *.stl *.fbx *.glb *.png *.pgm`） | 开发机工作区，或按下方 third_party 说明取上游模型 |
| `third_party/`（simple_ackermann、turtlebot3、turtlebot3_simulations） | 各自是独立上游仓库，需单独 clone 到该目录 |
| `build/ install/ log/` | `colcon build` 产物 |
| `evidence/ *.jsonl *.log` 与录屏 | 运行期产物，不入库 |

编译运行环境（WSL2 + ROS 2 Humble + Gazebo Classic 11）见 [`SETUP.md`](./SETUP.md)；
`tools/` 下的 `pk*_*.py`、`*_*.sh` 是各模块的联调与验收脚本，`tools/restart_clean_stack.sh` 是整栈重启入口。

---

## 一、目录结构

```
v2x_traffic_sim/
├── SETUP.md                       # 环境搭建清单 + 最小闭环验证（阶段 0–9）
├── README.md                      # 本文件
├── sync_build.sh                  # Windows→WSL 源码同步 + colcon 编译
├── start_all.sh                   # 一键 bringup（gzserver+信号灯+多车+记录+区域触发）
├── run_dashboard.sh               # 启动可视化操作界面（PyQt5 大屏）
├── snap_cam.sh / resnap_cam.sh    # 实景3D 抓帧验证（重启/复用 headless 仿真）
├── e2e_regions.sh                 # 区域触发底座真实仿真端到端验证
├── e2e_traffic.sh                 # 多车+轨迹记录回放端到端验证
├── scene_builder.py / v2x_scene.yaml  # 场景配置化生成（v2x_city.world）
├── src/v2x_msgs/                  # 自定义消息：RegionEvent / RegionState
├── src/v2x_gazebo_plugins/        # C++ Gazebo 插件：信号灯视觉点亮 + 故障模拟
└── src/v2x_demo/                  # ROS 2 功能包（起点代码）
    ├── package.xml
    ├── CMakeLists.txt
    ├── worlds/
    │   ├── v2x_intersection.world # 十字路口 + 人行道 + 红绿灯 + 建筑/障碍
    │   ├── v2x_city.world         # 生成物：全城场景（勿手改）
    │   └── map.yaml / map.pgm     # Nav2 空白占位地图（可用 SLAM 替换）
    ├── models/
    │   ├── v2x_prius/             # prius 视觉 + 阿克曼驱动 + 雷达/IMU/相机
    │   ├── v2x_stop_light/        # 可独立测试的信号灯模型（含点亮插件）
    │   ├── smart_car/             # 参数化智能小车 SDF 模板
    │   ├── turtlebot3_burger/     # TurtleBot3 Burger 预置模型
    │   ├── turtlebot3_waffle_pi/  # TurtleBot3 Waffle Pi 预置模型（含相机）
    │   └── turtlebot3_common/     # TurtleBot3 公共网格
    ├── launch/
    │   ├── v2x_bringup.launch.py  # 启动仿真 + 生成车辆 + 信号灯 + 区域触发 + 多车 + 记录
    │   ├── v2x_nav2.launch.py     # SLAM 建图 + Nav2 自主导航
    │   ├── v2x_teleop.launch.py   # 键盘遥控
    │   ├── smart_car.launch.py    # 智能小车算法仿真（含 Nav2 多机器人）
    │   └── nav2_multi_robot.launch.py  # 多机器人 Nav2 导航栈
    ├── config/
    │   ├── traffic_light_params.yaml  # 信号灯配时（groups/presets/时长）
    │   ├── traffic_flow.yaml          # 多车交通流（车辆/起点/路线/速度）
    │   ├── regions.yaml               # 区域触发区定义（ETC/闸机/测速/卡口）
    │   ├── scene_devices.yaml         # 场景设备坐标表（生成物）
    │   ├── smart_car_config.yaml      # 智能小车配置（多车型预设 + 实例）
    │   └── nav2_tb3_params.yaml       # TurtleBot3 Nav2 参数
    └── v2x_demo/
        ├── traffic_light_manager.py   # 多路口信号灯协调（配时/联动/预设/紧急全绿）
        ├── traffic_light_controller.py# 旧版单路口控制器（保留兼容）
        ├── region_trigger_node.py     # 区域触发公共底座（进/出事件）
        ├── traffic_flow_node.py       # 多车交通流（生成+巡航）
        ├── trajectory_recorder.py     # 轨迹记录（/model_states -> JSON）
        ├── trajectory_player.py       # 轨迹回放（幽灵模型+位姿话题）
        ├── v2x_dashboard.py           # 可视化操作界面（PyQt5）
        ├── smart_car_spawner.py       # 智能小车生成管理器（模板+预置模型）
        ├── car_generator.py           # SDF 参数化生成器
        ├── smart_car_demo.py          # 智能小车演示脚本
        └── *_selftest.py              # 各模块合成数据自测（无需 Gazebo）
```

---

## 二、编译

```bash
# 前置：已完成 SETUP.md 阶段 1–6（Humble + Gazebo 11 + Nav2 + ros2_control）
mkdir -p ~/v2x_ws/src
cp -r /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo ~/v2x_ws/src/

# ⚠️ Windows→WSL 必做：清除 CRLF，否则可执行 .py 的 shebang 会变成 python3\r 而崩溃
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r//g' {} +

# 复用工作区模型库（stop_light / prius_hybrid 网格）
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master' >> ~/.bashrc
source ~/.bashrc

cd ~/v2x_ws
rosdep install --from-paths src -i -y     # 安装缺失依赖
colcon build --symlink-install
source install/setup.bash
```

---

## 三、运行（对应 SETUP.md 检查点）

```bash
# 终端 1：一键 bringup（gzserver + prius + 信号灯协调 + 多车 + 轨迹记录 + 区域触发）
bash start_all.sh
#   可选开关：TRAFFIC_FLOW=0 / TRAJECTORY_RECORD=0 关闭多车/记录；SPAWN_X/Y/YAW 改 prius 起点

# 终端 2：可视化操作界面（大屏，F11 全屏；工具条：实景3D(Gazebo)/3D 场景/2D 俯视；
#         3D 内左键旋转/滚轮缩放/右键平移；实景3D 需仿真在跑）
bash run_dashboard.sh

# 终端 2（改为）：键盘遥控（检查点 3）
ros2 launch v2x_demo v2x_teleop.launch.py

# 终端 2（改为）：Nav2 自主导航（检查点 4，在 RViz 点 "Nav2 Goal"）
ros2 launch v2x_demo v2x_nav2.launch.py

# 观察话题（检查点 2）
ros2 topic list | grep -E "scan|imu|odom|cmd_vel|camera|traffic_light|distance"
ros2 topic hz /scan
ros2 topic echo /traffic_light/J1_ns/state   # 分组灯态 GREEN / YELLOW / RED 循环

# 区域触发底座
ros2 topic echo /v2x/region_events        # 车辆进出 ETC/闸机/测速/卡口区域的事件
```

### 实景3D：直接嵌 Gazebo 渲染（贴图/光照/阴影）

界面工具条第三个按钮「实景3D(Gazebo)」显示**真实 Gazebo 渲染画面**（非 QPainter 投影）：

- 世界内由 `scene_builder.py` 生成一个可移动场景相机模型 `v2x_scene_cam`
  （`v2x_scene.yaml` 的 `scene_camera:` 块配置分辨率/视场角/默认轨道），
  挂 `libgazebo_ros_camera.so`，发布 `/<id>/cam/image_raw`（见 `scene_devices.yaml` 的
  `scene_camera.image_topic`）。
- 界面 `SceneCamView` 与 3D 视口共用同一套轨道参数（左键旋转/滚轮缩放/右键平移），
  每帧把轨道参数换算成 look-at 位姿，经 `/set_entity_state` 下发给 `v2x_scene_cam`
  （内部 15Hz 节流），并把订阅到的最新图像帧绘制到视口 → 得到带贴图/光照/阴影的实景 3D。
- 验证：`bash snap_cam.sh`（重启 headless 仿真并抓实景帧）或
  `bash resnap_cam.sh`（对已运行仿真抓帧），产物
  `dashboard_live_preview_cam_scene_cam.png`。

⚠️ 实景3D 的三个实测坑（改启动脚本时务必保留）：
1. **不要加 `-s libgazebo_ros_force_system.so`**：它会阻止相机传感器插件加载，
   导致 `/v2x_scene_cam/cam/image_raw` 不出现（start_all/show_dashboard/snap_cam 已去除）。
2. **gzserver 不要设 `LIBGL_ALWAYS_INDIRECT=1`**：间接 GL 使 OGRE 相机渲染失败
   （该变量只给 GUI 客户端用）。
3. 相机模型的 `<kinematic>`/`<gravity>`/`<inertial>` 必须写在 `<link>` 内（model 级
   kinematic 被忽略 → 相机自由落体），且传感器需 `<always_on>1</always_on>`，
   否则 headless gzserver 不更新传感器。

### 信号灯视觉点亮 + 多路口联动（协议 §三.(二)）

- 视觉点亮：`v2x_gazebo_plugins` 的 `libv2x_traffic_light.so`（Gazebo model 插件）订阅
  `traffic_light/<group>/state`，向 Gazebo 内部 `~/visual` 话题发 emissive 颜色，红/黄/绿球真实点亮。
  需 `GAZEBO_PLUGIN_PATH` 含 `install/v2x_gazebo_plugins/lib`（start_all.sh / launch 已设置）。
- 多路口联动：`traffic_light_manager.py` 以仿真时间为单一时钟；`complementary:true` 的组自动反相
  （offset=红时长，保证永不同时绿），数值 `offset` 实现干线绿波（J2 相对 J1 延迟）。
- 预设方案：向 `traffic_light/preset` 发布 `default/peak/offpeak/night` 运行期切换。
- 紧急全绿：`ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: true}"`。

```bash
# 合成数据自测（相位/反相不变量，无需 Gazebo）
python3 ~/v2x_ws/src/v2x_demo/v2x_demo/traffic_light_selftest.py
# 真实仿真端到端（灯态反相 + 视觉插件加载）
sed 's/\r$//' verify_tl.sh | bash
```

### 多车 + 轨迹记录回放（协议 §三.(一).5）

- 多车：`traffic_flow_node.py` 按 `config/traffic_flow.yaml` 生成多辆 planar-move 小车并沿路点巡航。
- 记录：`trajectory_recorder.py` 订阅 `/model_states` 记录所有 `v2x_*` 车轨迹，`~/save` 存 JSON。
- 回放：`trajectory_player.py` `~/load` 生成半透明幽灵并按时间轴 `~/play` 复现轨迹，
  同时发布 `/v2x/playback/<name>/pose` 供界面绘制。

```bash
ros2 service call /trajectory_recorder/start std_srvs/srv/Trigger "{}"
ros2 service call /trajectory_recorder/save  std_srvs/srv/Trigger "{}"
ros2 service call /trajectory_player/load    std_srvs/srv/Trigger "{}"
ros2 service call /trajectory_player/play    std_srvs/srv/Trigger "{}"
# 合成数据自测 + 真实仿真端到端
python3 ~/v2x_ws/src/v2x_demo/v2x_demo/trajectory_selftest.py
sed 's/\r$//' e2e_traffic.sh | bash
```

### 智能小车算法仿真（协议 §三.(一)）

支持多车型配置、虚拟传感器、Nav2 导航、多车协调、故障模拟：

```bash
# 启动智能小车仿真（5 辆车：3 模板 + 2 TurtleBot3）
ros2 launch v2x_demo smart_car.launch.py

# 同时启动 Nav2 多机器人导航
ros2 launch v2x_demo smart_car.launch.py nav2:=true

# 运行演示脚本（驱动小车按预设路线行驶）
ros2 run v2x_demo smart_car_demo.py --ros-args -p car_name:=smart_car_1 -p pattern:=square

# 故障模拟
ros2 topic pub /smart_car_1/fault_command std_msgs/msg/String "data: 'POSE_DRIFT'"
ros2 service call /smart_car_1/enable_fault std_srvs/srv/SetBool "{data: true}"

# 自测（无需 Gazebo）
python3 ~/v2x_ws/src/v2x_demo/v2x_demo/car_selftest.py
```

**车型预设**（`config/smart_car_config.yaml`）：
- `delivery_bot`: 小型配送机器人（0.8m, 1.5m/s）
- `medium_car`: 中型轿车（1.6m, 3.0m/s）
- `bus`: 大型巴士（3.0m, 2.5m/s）
- `patrol_car`: 高速巡逻车（1.4m, 5.0m/s）
- `tb3_burger`: TurtleBot3 Burger 预置模型
- `tb3_waffle_pi`: TurtleBot3 Waffle Pi 预置模型（含相机）

**传感器话题**（每辆车独立命名空间 `/<car_name>/`）：
- `/<car_name>/cmd_vel` — 速度指令
- `/<car_name>/scan` — LiDAR 激光雷达
- `/<car_name>/imu/data` — IMU 惯性测量
- `/<car_name>/camera/image_raw` — 相机图像（仅部分车型）
- `/<car_name>/odom` — 里程计

### 区域触发底座验证

```bash
# 方式一：合成数据自测（不需要 Gazebo，断言事件序列/高架 z 过滤/占用状态）
source ~/v2x_ws/install/setup.bash
python3 ~/v2x_ws/src/v2x_demo/v2x_demo/region_trigger_selftest.py

# 方式二：真实仿真端到端（先让 start_all.sh 常驻，再执行）
sed 's/\r$//' e2e_regions.sh | bash       # 驱车穿过测速点与 ETC 区并打印事件
```

### 可视化界面验证（无需显示器）

```bash
python3 ~/v2x_ws/src/v2x_demo/v2x_demo/dashboard_selftest.py      # 数据层自测（offscreen）
QT_QPA_PLATFORM=offscreen python3 ~/v2x_ws/src/v2x_demo/v2x_demo/dashboard_gui_smoke.py /tmp/d.png
```

---

## 四、话题接口一览（ROS 2 标准话题）

| 话题 | 类型 | 方向 | 说明 |
|------|------|:---:|------|
| `/cmd_vel` | geometry_msgs/Twist | 入 | 车辆速度指令（阿克曼插件订阅） |
| `/odom` | nav_msgs/Odometry | 出 | 里程计（插件发布，含 TF odom→chassis） |
| `/distance` | std_msgs/Float32 | 出 | 行驶距离 |
| `/scan` | sensor_msgs/LaserScan | 出 | 激光雷达（供 Nav2/避障） |
| `/imu/data` | sensor_msgs/Imu | 出 | IMU |
| `/camera/image_raw` | sensor_msgs/Image | 出 | 前视相机（供车牌识别） |
| `/<car_name>/cmd_vel` | geometry_msgs/Twist | 入 | 智能小车速度指令 |
| `/<car_name>/scan` | sensor_msgs/LaserScan | 出 | 智能小车 LiDAR |
| `/<car_name>/imu/data` | sensor_msgs/Imu | 出 | 智能小车 IMU |
| `/<car_name>/odom` | nav_msgs/Odometry | 出 | 智能小车里程计 |
| `/<car_name>/camera/image_raw` | sensor_msgs/Image | 出 | 智能小车相机（仅部分车型） |
| `/<car_name>/fault_command` | std_msgs/String | 入 | 故障模拟命令（NONE/SENSOR_FAIL/DATA_LAG/POSE_DRIFT/ALL） |
| `/traffic_light/state` | std_msgs/String | 出 | 默认组灯态（兼容旧接口） |
| `/traffic_light/<group>/state` | std_msgs/String | 出 | 分组灯态 GREEN/YELLOW/RED（驱动视觉点亮插件） |
| `/traffic_light/preset` | std_msgs/String | 入 | 切换配时预设（default/peak/offpeak/night） |
| `/v2x/region_events` | v2x_msgs/RegionEvent | 出 | 区域进/出事件流（测速/卡口/停车场/ETC 公共底座） |
| `/v2x/region/<id>/state` | v2x_msgs/RegionState | 出 | 区域占用快照（transient_local；话题名中 id 的非字母数字字符替换为 `_`） |
| `/v2x/playback/<name>/pose` | geometry_msgs/PoseStamped | 出 | 轨迹回放位姿（供界面绘制幽灵） |
| `/model_states` | gazebo_msgs/ModelStates | 入 | 全模型位姿+速度（world 内 `gazebo_ros_state` 插件发布，区域检测/轨迹记录的位姿源） |
| `/v2x_scene_cam/cam/image_raw` | sensor_msgs/Image | 入 | 场景相机视频流（实景3D 视口视频源；话题名见 scene_devices.yaml） |

**服务**：`/traffic_light_manager/emergency`(SetBool 紧急全绿)、
`/trajectory_recorder/{start,stop,save,clear}`(Trigger)、
`/trajectory_player/{load,play,stop}`(Trigger)、
`/<car_name>/enable_fault`(SetBool 故障模拟启用/禁用)、
`/set_entity_state`(gazebo_msgs/SetEntityState，界面驱动实景相机位姿)。

---

## 五、调参与已知事项（务必阅读）

1. **阿克曼 PID 需调参**：`models/v2x_prius/model.sdf` 中
   `linear_velocity_pid_gain` / `*_steering_pid_gain` 为官方示例值，
   实际车重(1326kg)/轮径(0.625m)下若车辆抖动或无力，需调整增益与 `max_speed`。
2. **prius 网格依赖**：`model.sdf` 的 chassis 视觉引用
   `model://prius_hybrid/meshes/Hybrid.obj`，需 `GAZEBO_MODEL_PATH` 含
   `gazebo_models-master`；若网格加载失败，可删除该 `<visual>`，仅保留 box 碰撞体，
   不影响物理与话题闭环。
3. **信号灯视觉点亮依赖 GAZEBO_PLUGIN_PATH**：灯球真实点亮由
   `v2x_gazebo_plugins` 的 `libv2x_traffic_light.so` 完成（订阅分组灯态 → 发 Gazebo
   `~/visual` emissive）。若灯不亮，检查 `GAZEBO_PLUGIN_PATH` 是否含
   `install/v2x_gazebo_plugins/lib`（start_all.sh / launch 已设置）；每个插件实例用唯一
   ROS 节点名，重复节点名会导致 gzserver 段错误。
4. **命名空间**：最小骨架用全局话题（便于 Nav2 开箱即用）；多车/多路口场景
   应在插件加 `<namespace>` 并统一 remap。
5. **⚠️ Windows→WSL 换行符（CRLF）坑**（实测复现）：在 Windows 侧创建的文件是 CRLF，
   `cp` 到 WSL 后，**作为可执行脚本直接运行的 `.py`**（如 `traffic_light_controller.py`）
   shebang 会变成 `#!/usr/bin/env python3\r`，启动即报
   `/usr/bin/env: 'python3\r': No such file or directory`（exit 127）。
   解法：cp 后执行「二、编译」中的 `sed -i 's/\r//g'` 转换（launch/sdf/world/yaml
   的 CRLF 无害，只有 shebang 脚本致命）。
6. **⚠️ `use_sim_time` 重复声明坑**（实测复现）：节点内**不要**写
   `self.declare_parameter('use_sim_time', ...)`——launch 通过 `parameters=[..., {'use_sim_time': ...}]`
   传入时 rclpy 会自动声明它，手动再声明会抛
   `ParameterAlreadyDeclaredException` 使节点退出。业务参数（配时时长等）正常 declare 即可。
7. **⚠️ Git Bash → `wsl.exe` 内联脚本坑**（实测复现）：经 Windows 侧 Git Bash 把
   `VAR=/path` 形式的赋值写进 `wsl -- bash -c '...'` 内联脚本时，MSYS 路径转换会把
   变量值清空（`A=/tmp/xx` → 空），曾导致 `cp -r "$SRC/."` 退化成复制根目录。
   规约：跨 WSL 的多步操作一律落盘为 .sh 再 `sed 's/\r$//' x.sh | bash` 执行（见 `sync_build.sh`）。
8. **区域触发 z 窗口**：`regions.yaml` 中地面区域取 center z=1.0 / size z=3
   （覆盖 -0.5~2.5m），用于排除高架（z≈7.4）行驶车辆的误触发；新增区域时保持该约定。

---

## 六、扩展点（映射协议六大模块）

| 协议模块 | 本骨架已有基座 | 下一步扩展 |
|---------|--------------|-----------|
| (一) 智能小车算法仿真 | **多车型预设（模板+TurtleBot3）**；**虚拟传感器（LiDAR/Camera/IMU/Odom）**；**Nav2 多机器人导航**；**多车协调**；**轨迹记录回放**；**故障模拟插件** | 更真实车辆动力学、SLAM 建图 |
| (二) 信号灯配时 | **视觉点亮(libv2x_traffic_light)**；**多路口联动(反相/绿波)**；**预设方案**；**紧急全绿** | 自适应配时（按流量调绿信比） |
| (三) 卡口车牌识别 | 前视相机 + 区域触发底座（CKPT 区 ENTER 触发抓拍） | 接 HyperLPR、虚拟车牌生成、光照模拟、黑名单 |
| (四) 测速路段 | /odom 速度 + 区域触发底座（SPEED 点 ENTER/EXIT 时间差=区间测速） | 定点/区间测速、超速告警、流量统计 |
| (五) 停车场闸机 | 相机 + 区域触发底座（PKG 闸机区/地感区） | 道闸动力学模型、抬落杆状态机、白名单 |
| (六) ETC 通行 | 区域触发底座（ETC 门架区 ENTER 触发识别） | 虚拟 RSU/OBU 识别、扣费逻辑、跟车闯关 |
| 可视化操作界面 | **PyQt5 大屏界面（实景3D=Gazebo 渲染视频流 + 3D 场景视口 + 2D 俯视 + 参数面板 + 日志窗，v2x_dashboard.py）** | 历史曲线、多屏布局 |

> 区域触发公共底座已落地为 `region_trigger_node`（基于 `/model_states` 的纯 Python
> 几何检测，区域在 `config/regions.yaml` 配置，z 高度窗口可排除高架车辆），
> 无需编译 C++ 插件；若后续需要碰撞级精度，可再换用工作区
> `gazebo-classic-gazebo11/plugins/events/RegionEventBoxPlugin`。

---

## 七、许可证

骨架代码采用 Apache-2.0。所引用的第三方模型（gazebo_models-master）遵循其各自上游许可证，
交付前请按协议 §7.3 核对知识产权合规性。
