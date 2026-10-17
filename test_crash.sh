#!/usr/bin/env bash
# 复现 gzserver 崩溃并抓内核 segfault 记录（定位崩溃的 .so 库）
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f static_transform 2>/dev/null
pkill -9 -f traffic_light 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null
sleep 2
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master

echo "===== dmesg 是否可读 ====="
dmesg >/dev/null 2>&1 && echo "可读" || echo "不可读(需权限)"

MARK=$(dmesg 2>/dev/null | wc -l)
echo "===== 复现：gzserver + 5s后并发一个 static_transform_publisher ====="
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
gzserver --verbose "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/gz_c.log 2>&1 &
GZ=$!
sleep 5
echo "gzserver(单独)存活? $(kill -0 $GZ 2>/dev/null && echo yes || echo no)"
# 5s 后加入一个并发 ROS 节点
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 chassis base_link --ros-args -r __node:=probe_tf > /tmp/probe.log 2>&1 &
PROBE=$!
for i in $(seq 1 15); do
  sleep 1
  kill -0 $GZ 2>/dev/null || { echo ">>> gzserver 在并发节点加入后第 ${i}s 崩溃 <<<"; break; }
done
echo "gzserver 最终存活? $(kill -0 $GZ 2>/dev/null && echo yes || echo NO-crashed)"
echo "===== gzserver 日志末尾 ====="
tail -8 /tmp/gz_c.log
echo "===== 新增 dmesg (segfault) ====="
dmesg 2>/dev/null | tail -n +$((MARK+1)) | grep -iE 'gzserver|segfault|traps' | tail -10 || echo "(无 segfault 记录或 dmesg 不可读)"
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f static_transform 2>/dev/null
