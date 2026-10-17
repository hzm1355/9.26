#!/usr/bin/env bash
# 让可视化界面真实弹出（走 WSLg 显示到 Windows 桌面），并起实时仿真供其取数。
# 不覆盖 DISPLAY（WSLg 默认 :0）；若你用 VcXsrv 请自行 export DISPLAY=<网关>:0.0。
# 进程常驻，用 stop_all.sh / 手动 kill 结束。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export LIBGL_ALWAYS_INDIRECT=1
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
SHARE=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo

echo "[show] DISPLAY=$DISPLAY (WSLg)"
echo "[show] 1) gzserver headless..."
setsid nohup gzserver "$SHARE/worlds/v2x_city.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/show_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break; kill -0 $GZ 2>/dev/null || { echo "gz died"; exit 1; }; sleep 1; done

echo "[show] 2) 信号灯 + 多车 + 记录器..."
setsid nohup ros2 run v2x_demo traffic_light_manager.py --ros-args -p config_file:="$SHARE/config/traffic_light_params.yaml" -p use_sim_time:=true > /tmp/show_tl.log 2>&1 &
setsid nohup ros2 run v2x_demo traffic_flow_node.py --ros-args -p config_file:="$SHARE/config/traffic_flow.yaml" -p use_sim_time:=true > /tmp/show_flow.log 2>&1 &
setsid nohup ros2 run v2x_demo trajectory_recorder.py --ros-args -p output_file=/tmp/show_traj.json -p use_sim_time:=true > /tmp/show_rec.log 2>&1 &
sleep 6

echo "[show] 3) 弹出可视化界面（GUI 常驻）..."
rm -f /tmp/show_dash.log
setsid nohup ros2 run v2x_demo v2x_dashboard.py --ros-args -p use_sim_time:=true > /tmp/show_dash.log 2>&1 &
DASH=$!
sleep 6
if kill -0 $DASH 2>/dev/null; then
  echo "[show] ✅ 界面进程存活 pid=$DASH（应已显示在桌面，F11 全屏）"
else
  echo "[show] ❌ 界面进程已退出，日志如下："
  tail -20 /tmp/show_dash.log
fi
echo "SHOW_PIDS gz=$GZ dash=$DASH"
