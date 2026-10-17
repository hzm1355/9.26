#!/usr/bin/env bash
# 信号灯视觉点亮 + 多路口联动 端到端验证（headless gzserver + manager）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH

WORLD=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_city.world
TLCFG=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/traffic_light_params.yaml

echo "[verify] gzserver headless (v2x_city.world)..."
gzserver "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so > /tmp/verify_tl_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break; kill -0 $GZ 2>/dev/null || { echo "gzserver died"; tail -20 /tmp/verify_tl_gz.log; exit 1; }; sleep 1; done

echo "[verify] start traffic_light_manager..."
ros2 run v2x_demo traffic_light_manager.py --ros-args -p config_file:="$TLCFG" -p use_sim_time:=true > /tmp/verify_tl_mgr.log 2>&1 &
MGR=$!
sleep 6

echo "=== (A) 插件加载数量（应=6，且 visuals=1） ==="
grep -c "v2x_traffic_light] model=" /tmp/verify_tl_gz.log
echo "--- 明细 ---"
grep "v2x_traffic_light] model=" /tmp/verify_tl_gz.log

echo "=== (B) 各组灯态话题存在性 ==="
ros2 topic list 2>/dev/null | grep "traffic_light/"

echo "=== (C) 反相校验：J1_ns 与 J1_ew 采样 12 次，二者不得同时 GREEN ==="
BOTHGREEN=0
for k in $(seq 1 12); do
  A=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  B=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  echo "  sample $k: J1_ns=$A J1_ew=$B"
  [ "$A" = "GREEN" ] && [ "$B" = "GREEN" ] && BOTHGREEN=$((BOTHGREEN+1))
  sleep 1
done
echo "BOTHGREEN_COUNT=$BOTHGREEN (期望 0)"

echo "=== (D) 预设切换 peak（周期应变长）==="
ros2 topic pub --once /traffic_light/preset std_msgs/msg/String "{data: 'peak'}" >/dev/null 2>&1
sleep 2
grep "preset ->" /tmp/verify_tl_mgr.log | tail -3

echo "=== (E) 急救优先全绿服务 ==="
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: true}" 2>/dev/null | grep -i "message\|success" | head
sleep 2
EG=$(timeout 2 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
echo "emergency 时 J1_ew=$EG (期望 GREEN)"
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1

echo "=== (F) manager 日志尾部（查错）==="
tail -8 /tmp/verify_tl_mgr.log
echo "=== gzserver 日志错误检查 ==="
grep -i "error\|segfault\|terminate\|exception" /tmp/verify_tl_gz.log | grep -vi "no error" | head

kill -9 $MGR $GZ 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f traffic_light_ 2>/dev/null
echo "VERIFY_TL_DONE bothgreen=$BOTHGREEN emergency_ew=$EG"
