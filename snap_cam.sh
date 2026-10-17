#!/usr/bin/env bash
# 实景3D 验证：以新世界（含 v2x_scene_cam）重启 headless 仿真，
# 等待 /v2x_scene_cam/image_raw 出图，再用 offscreen 界面抓一张真实 Gazebo 渲染帧。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export QT_QPA_PLATFORM=offscreen
# 注意：不要设 LIBGL_ALWAYS_INDIRECT=1 —— 间接 GL 会让 gzserver 的 OGRE 相机渲染失败
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
SHARE=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo

echo "[cam] 0) 清理旧仿真进程..."
pkill -9 -f '[g]zserver' 2>/dev/null
pkill -9 -f '[t]raffic_light_manager' 2>/dev/null
pkill -9 -f '[t]raffic_flow_node' 2>/dev/null
pkill -9 -f '[t]rajectory_recorder' 2>/dev/null
pkill -9 -f '[r]egion_trigger_node' 2>/dev/null
sleep 3

echo "[cam] 1) gzserver headless（新世界，含场景相机）..."
setsid nohup gzserver "$SHARE/worlds/v2x_city.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  > /tmp/cam_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do
  ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break
  kill -0 $GZ 2>/dev/null || { echo "[cam] gz died"; tail -20 /tmp/cam_gz.log; exit 1; }
  sleep 1
done

echo "[cam] 2) 信号灯 + 多车..."
setsid nohup ros2 run v2x_demo traffic_light_manager.py --ros-args -p config_file:="$SHARE/config/traffic_light_params.yaml" -p use_sim_time:=true > /tmp/cam_tl.log 2>&1 &
setsid nohup ros2 run v2x_demo traffic_flow_node.py --ros-args -p config_file:="$SHARE/config/traffic_flow.yaml" -p use_sim_time:=true > /tmp/cam_flow.log 2>&1 &
sleep 5

echo "[cam] 3) 等待 /v2x_scene_cam/cam/image_raw 出图（最多 30s）..."
GOT=0
for i in $(seq 1 30); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -q '/v2x_scene_cam/cam/image_raw'; then
    GOT=1; echo "[cam]    图像话题已就绪 (${i}s)"; break
  fi
  sleep 1
done
[ "$GOT" = "1" ] || echo "[cam]    ⚠️ 30s 内未收到图像，继续尝试抓图"

echo "[cam] 4) offscreen 界面抓实景帧..."
OUT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/dashboard_live_preview_cam.png
rm -f "$OUT"
V2X_DASH_LIVE=1 V2X_DASH_LIVE_SECS=8 python3 "$D/dashboard_gui_smoke.py" "$OUT" 2>&1 | grep -iE 'status bar|saved|SMOKE'
ls -la "$OUT" "${OUT%.png}_scene_cam.png" 2>/dev/null
echo "CAM_SNAP_DONE"
