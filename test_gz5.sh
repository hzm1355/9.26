#!/usr/bin/env bash
# 官方 launch 起 gzserver，等其彻底初始化(20s)后再加入节点，判断是否"晚加入即安全"
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f static_transform 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
sleep 2
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
SDF=~/v2x_ws/install/v2x_demo/share/v2x_demo/models/v2x_prius/model.sdf

MARK=$(dmesg 2>/dev/null | wc -l)
echo "===== 官方 gzserver.launch.py，先独占 20s ====="
ros2 launch gazebo_ros gzserver.launch.py world:="$WORLD" verbose:=true > /tmp/gz5.log 2>&1 &
sleep 20
echo "20s 后 gzserver 存活? $(pgrep -f 'gzserver .*v2x_intersection' >/dev/null && echo yes || echo no)"
echo "gzserver 是否已加载 world(找 Loading world / OBJLoader):"
grep -cE 'Loading world|OBJLoader|Connected to gazebo master' /tmp/gz5.log

echo "===== 现在加入 static TF + spawn prius ====="
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 chassis base_link --ros-args -r __node:=late_tf > /tmp/late_tf.log 2>&1 &
sleep 2
ros2 run gazebo_ros spawn_entity.py -entity v2x_prius -file "$SDF" -x 0 -y -15 -z 0.4 -Y 1.5708 > /tmp/late_sp.log 2>&1 &
for i in $(seq 1 15); do
  sleep 1
  pgrep -f 'gzserver .*v2x_intersection' >/dev/null || { echo ">>> gzserver 在晚加入后第 ${i}s 崩溃 <<<"; break; }
done
echo "gzserver 最终存活? $(pgrep -f 'gzserver .*v2x_intersection' >/dev/null && echo yes || echo NO-crashed)"
echo "===== spawn 日志末尾 ====="
tail -6 /tmp/late_sp.log
echo "===== 新增 dmesg segfault ====="
dmesg 2>/dev/null | tail -n +$((MARK+1)) | grep -iE 'gzserver|segfault|traps' | tail -6 || echo "(无)"
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f static_transform 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null
