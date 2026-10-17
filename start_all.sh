#!/usr/bin/env bash
# ============================================================================
# V2X bringup 编排脚本（绕开 WSL2 下 "ros2 launch 管理 gzserver 卡死" 的问题）
# gzserver 直跑（已验证可靠）-> 等 /spawn_entity 服务 -> spawn prius
#   -> 起 3x 静态 TF(chassis->base_link / chassis->laser / base_link->base_footprint)
#   -> 起信号灯控制器 -> wait 常驻
# 用法: bash start_all.sh          (前台常驻，Ctrl-C 结束)
# ============================================================================
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
export LIBGL_ALWAYS_INDIRECT=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
# 信号灯视觉点亮插件(libv2x_traffic_light.so)所在目录，Gazebo 加载 world 内联插件时需可搜索到
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH

SHARE=~/v2x_ws/install/v2x_demo/share/v2x_demo
WORLD=$SHARE/worlds/v2x_city.world
SDF=$SHARE/models/v2x_prius/model.sdf
TL_PARAMS=$SHARE/config/traffic_light_params.yaml
FLOW_PARAMS=$SHARE/config/traffic_flow.yaml
TRAJ_OUT=${TRAJ_OUT:-$HOME/v2x_trajectory.json}
# 多车交通流 / 轨迹记录 开关（默认开）。例: TRAFFIC_FLOW=0 bash start_all.sh 关闭多车
TRAFFIC_FLOW=${TRAFFIC_FLOW:-1}
TRAJECTORY_RECORD=${TRAJECTORY_RECORD:-1}
# 可选 spawn 点（默认主车道起点）。例: SPAWN_X=15 SPAWN_Y=-15 bash start_all.sh
SPAWN_X=${SPAWN_X:-0.0}
SPAWN_Y=${SPAWN_Y:--15.0}
SPAWN_YAW=${SPAWN_YAW:-3.14159}

# 退出时清理子进程
trap 'echo "[start_all] 收到退出信号，清理..."; pkill -9 -P $$ 2>/dev/null; pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; exit 0' INT TERM

echo "[start_all] 1) 直跑 gzserver（含 ROS 插件，输出重定向到 /tmp/gzserver.log 防刷屏卡终端）..."
# ⚠️ 不要加 --verbose 且必须重定向输出：后台终端里 gzserver 持续刷屏会撑爆终端捕获，
#    导致 IDE 端 GetTerminalOutput/命令全部超时（实测复现）。
# ⚠️ 不要加 libgazebo_ros_force_system.so：实测它会阻止相机传感器插件
#    (libgazebo_ros_camera) 加载，导致实景3D 无图像；本流程无人使用 force 服务。
gzserver "$WORLD" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so > /tmp/gzserver.log 2>&1 &
GZ=$!
echo "[start_all]    gzserver pid=$GZ"

echo "[start_all] 2) 等待 /spawn_entity 服务就绪（最多 40s）..."
READY=0
for i in $(seq 1 40); do
  if ros2 service list 2>/dev/null | grep -q '/spawn_entity'; then
    echo "[start_all]    /spawn_entity 就绪 (${i}s)"; READY=1; break
  fi
  kill -0 $GZ 2>/dev/null || { echo "[start_all] gzserver 已退出，终止"; exit 1; }
  sleep 1
done
[ "$READY" = "1" ] || { echo "[start_all] 等待超时，仍尝试 spawn"; }

echo "[start_all] 3) spawn prius（($SPAWN_X,$SPAWN_Y) yaw=$SPAWN_YAW；模型车头为 body -Y）..."
ros2 run gazebo_ros spawn_entity.py -entity v2x_prius -file "$SDF" \
  -x "$SPAWN_X" -y "$SPAWN_Y" -z 0.4 -Y "$SPAWN_YAW"

echo "[start_all] 4) 起静态 TF + 信号灯控制器（输出重定向日志） ..."
# v2x_prius 模型的车头是 body -Y（前轮在 y=-1.41、轮转轴沿 X）。
# ROS 约定 base_link+X = 车头，故 chassis->base_link 需加 yaw -90°，
# 使 base_link+X 指向 chassis -Y（车头）；否则车会侧向横移、Nav2 几何全错。
# chassis->laser 保持零旋转：scan 角0 = 模型+X（车右侧），这是物理真实姿态。
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 0 chassis base_link \
  --ros-args -r __node:=base_link_broadcaster -p use_sim_time:=true > /tmp/tf_base_link.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 1.5 0 0 0 chassis laser \
  --ros-args -r __node:=laser_broadcaster -p use_sim_time:=true > /tmp/tf_laser.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint \
  --ros-args -r __node:=base_footprint_broadcaster -p use_sim_time:=true > /tmp/tf_base_footprint.log 2>&1 &
ros2 run v2x_demo traffic_light_manager.py \
  --ros-args -p config_file:="$TL_PARAMS" -p use_sim_time:=true > /tmp/traffic_light.log 2>&1 &
# 区域触发公共底座（/model_states -> /v2x/region_events），日志独立防刷屏
ros2 run v2x_demo region_trigger_node.py \
  --ros-args -p use_sim_time:=true > /tmp/region_trigger.log 2>&1 &

if [ "$TRAFFIC_FLOW" = "1" ]; then
  echo "[start_all] 5) 起多车交通流（traffic_flow_node，$FLOW_PARAMS）..."
  ros2 run v2x_demo traffic_flow_node.py \
    --ros-args -p config_file:="$FLOW_PARAMS" -p use_sim_time:=true > /tmp/traffic_flow.log 2>&1 &
fi
if [ "$TRAJECTORY_RECORD" = "1" ]; then
  echo "[start_all] 6) 起轨迹记录器（trajectory_recorder -> $TRAJ_OUT）..."
  ros2 run v2x_demo trajectory_recorder.py \
    --ros-args -p output_file:="$TRAJ_OUT" -p use_sim_time:=true > /tmp/trajectory_recorder.log 2>&1 &
fi

echo "[start_all] ✅ bringup 完成，gzserver 常驻中（本终端保持运行，日志在 /tmp/*.log）。"
echo "[start_all] 下一步在另一终端: bash run_nav2.sh"
wait
