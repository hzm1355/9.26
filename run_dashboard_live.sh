#!/usr/bin/env bash
# 可视化界面 真实仿真集成验证（headless + offscreen）：
# 起 gzserver + 信号灯 + 多车 + 记录器，然后以 V2X_DASH_LIVE 模式跑 dashboard 抓图，
# 验证界面能接到真实 /model_states 与灯态话题。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH
SHARE=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
OUT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/_smoke_live.png

echo "[live] gzserver headless..."
gzserver "$SHARE/worlds/v2x_city.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/live_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break; kill -0 $GZ 2>/dev/null || { echo "gz died"; exit 1; }; sleep 1; done

echo "[live] traffic_light_manager + traffic_flow + recorder..."
ros2 run v2x_demo traffic_light_manager.py --ros-args -p config_file:="$SHARE/config/traffic_light_params.yaml" -p use_sim_time:=true > /tmp/live_tl.log 2>&1 &
TL=$!
ros2 run v2x_demo traffic_flow_node.py --ros-args -p config_file:="$SHARE/config/traffic_flow.yaml" -p use_sim_time:=true > /tmp/live_flow.log 2>&1 &
FLOW=$!
ros2 run v2x_demo trajectory_recorder.py --ros-args -p output_file=/tmp/live_traj.json -p use_sim_time:=true > /tmp/live_rec.log 2>&1 &
REC=$!
sleep 8

echo "[live] dashboard LIVE 抓图（8s）..."
rm -f "$OUT"
V2X_DASH_LIVE=1 V2X_DASH_LIVE_SECS=8 python3 "$D/dashboard_gui_smoke.py" "$OUT" 2>&1 | grep -i "status bar\|saved\|SMOKE"
echo "LIVE_SMOKE_EXIT=$?"

kill -9 $TL $FLOW $REC $GZ 2>/dev/null
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f traffic_flow 2>/dev/null; pkill -9 -f trajectory_ 2>/dev/null; pkill -9 -f traffic_light_manager 2>/dev/null
ls -la "$OUT" 2>/dev/null
echo "LIVE_DONE"
