#!/usr/bin/env bash
# 二分定位：哪个 GazeboRosPaths env 变量导致 gzserver 卡死
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f 'ros2 launch' 2>/dev/null
sleep 2; rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
GOOD_MODEL=/mnt/d/desktop1/9.26.1/gazebo_models-master

run_case () {
  local name="$1"; shift
  pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; sleep 1
  rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
  env "$@" timeout -k 3 12 gzserver --verbose "$WORLD" -s libgazebo_ros_init.so > /tmp/case.log 2>&1
  local ok=$(grep -cE 'Loading world|Connected to gazebo master' /tmp/case.log)
  if [ "$ok" -gt 0 ]; then echo "[$name] 正常加载 world ✅"; else echo "[$name] 卡死/未加载 world ❌"; fi
}

echo "===== 基准：仅 MODEL_PATH 含 gazebo_models（已知可用）====="
run_case "baseline" GAZEBO_MODEL_PATH="$GOOD_MODEL"

echo "===== A: 额外设 GAZEBO_PLUGIN_PATH=空 ====="
run_case "plugin_empty" GAZEBO_MODEL_PATH="$GOOD_MODEL" GAZEBO_PLUGIN_PATH=""

echo "===== B: 额外设 GAZEBO_RESOURCE_PATH=空 ====="
run_case "resource_empty" GAZEBO_MODEL_PATH="$GOOD_MODEL" GAZEBO_RESOURCE_PATH=""

echo "===== C: MODEL_PATH 只含 v2x models（不含 gazebo_models）====="
run_case "model_v2x_only" GAZEBO_MODEL_PATH="/home/hzm/v2x_ws/install/v2x_demo/share/v2x_demo/models"

pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null
