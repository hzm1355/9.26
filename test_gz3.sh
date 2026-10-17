#!/usr/bin/env bash
# 停 launch + ros2 daemon + 清理，然后直跑带插件 gzserver（实时输出，无管道无timeout）
pkill -9 -f 'ros2 launch'      2>/dev/null
pkill -9 -f gzserver           2>/dev/null
pkill -9 -f gzclient           2>/dev/null
pkill -9 -f gzmaster           2>/dev/null
pkill -9 -f static_transform   2>/dev/null
pkill -9 -f traffic_light      2>/dev/null
pkill -9 -f spawn_entity       2>/dev/null
sleep 2
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
echo "===== 停止 ros2 daemon ====="
ros2 daemon stop 2>&1 | head -3
sleep 1
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
echo "===== 直跑 gzserver 带插件（后台 25s 后自动 -9） ====="
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
gzserver --verbose "$WORLD" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so > /tmp/gz_direct.log 2>&1 &
GZPID=$!
echo "gzserver pid=$GZPID，等待 25s..."
for i in $(seq 1 25); do
  sleep 1
  if ! kill -0 $GZPID 2>/dev/null; then
    echo ">>> gzserver 在第 ${i}s 已退出 <<<"
    break
  fi
done
echo "===== gzserver 是否存活 ====="
kill -0 $GZPID 2>/dev/null && echo "存活(pid=$GZPID)" || echo "已死"
echo "===== /tmp/gz_direct.log 末尾 ====="
tail -30 /tmp/gz_direct.log
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null
