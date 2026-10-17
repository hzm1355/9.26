#!/usr/bin/env bash
# 用"仅UDP"FastDDS profile 跑官方 gzserver.launch.py，验证是否解决 launch 卡死
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null
sleep 2; rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/fastdds_udp_only.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
sed -i 's/\r$//' "$FASTRTPS_DEFAULT_PROFILES_FILE"
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
echo "PROFILES=$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "===== 官方 gzserver.launch.py + 仅UDP profile (最多 20s) ====="
ros2 launch gazebo_ros gzserver.launch.py world:="$WORLD" verbose:=true > /tmp/gz_udp.log 2>&1 &
for i in $(seq 1 20); do
  sleep 1
  if grep -qE 'Loading world|Connected to gazebo master' /tmp/gz_udp.log 2>/dev/null; then
    echo ">>> 第 ${i}s: gzserver 已连 master/加载 world ✅ <<<"; break
  fi
done
echo "world加载标记计数: $(grep -cE 'Loading world|Connected to gazebo master|OBJLoader' /tmp/gz_udp.log)"
echo "gzserver 存活? $(pgrep -f 'gzserver .*v2x_intersection' >/dev/null && echo yes || echo no)"
echo "===== 日志末尾 ====="
tail -12 /tmp/gz_udp.log
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null
