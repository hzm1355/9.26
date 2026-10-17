#!/usr/bin/env bash
# 隔离测试：仅用官方 gzserver.launch.py 起 gzserver（无 spawn/TF/traffic），看是否崩
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f 'ros2 launch' 2>/dev/null
sleep 2
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
echo "===== 官方 gzserver.launch.py 单独跑 (25s) ====="
ros2 launch gazebo_ros gzserver.launch.py world:="$WORLD" verbose:=true > /tmp/gz_launch.log 2>&1 &
LPID=$!
for i in $(seq 1 25); do
  sleep 1
  if ! pgrep -f 'gzserver .*v2x_intersection' >/dev/null 2>&1; then
    if grep -q 'process has died' /tmp/gz_launch.log 2>/dev/null; then
      echo ">>> 第 ${i}s: gzserver 已崩溃 <<<"; break
    fi
  fi
done
echo "===== gzserver 是否存活 ====="
pgrep -af 'gzserver .*v2x_intersection' | grep -v 'bash -c' || echo "(gzserver 不在)"
echo "===== /tmp/gz_launch.log 末尾 ====="
tail -25 /tmp/gz_launch.log
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f 'ros2 launch' 2>/dev/null
