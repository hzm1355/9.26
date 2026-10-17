#!/usr/bin/env bash
# 验证：spawn 带 camera 的 prius 是否让 gzserver 段错误 + 崩溃库名 + X display 可达性
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f static_transform 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
sleep 2
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/mnt/d/desktop1/9.26.1/gazebo_models-master
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0
echo "DISPLAY=$DISPLAY"

echo "===== X display 可达性 (xdpyinfo) ====="
if command -v xdpyinfo >/dev/null 2>&1; then
  timeout 5 xdpyinfo >/dev/null 2>&1 && echo "X 可达 ✅" || echo "X 不可达 ❌ (VcXsrv 未运行?)"
else
  echo "(未装 xdpyinfo，用 python 探测)"
  timeout 5 bash -c "echo > /dev/tcp/${DISPLAY%%:*}/${DISPLAY##*:}" 2>/dev/null
  # 上面写法不稳，改用 X 端口探测
  (exec 3<>/dev/tcp/$(echo $DISPLAY | sed 's/:.*//')/6000) 2>/dev/null && echo "X 端口6000可连 ✅" || echo "X 端口6000不可连 ❌"
fi

MARK=$(dmesg 2>/dev/null | wc -l)
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
SDF=~/v2x_ws/install/v2x_demo/share/v2x_demo/models/v2x_prius/model.sdf
echo "===== 起 gzserver(带factory)，5s后 spawn prius ====="
gzserver --verbose "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/gz_sp.log 2>&1 &
GZ=$!
sleep 6
echo "spawn 前 gzserver 存活? $(kill -0 $GZ 2>/dev/null && echo yes || echo no)"
ros2 run gazebo_ros spawn_entity.py -entity v2x_prius -file "$SDF" -x 0 -y -15 -z 0.4 -Y 1.5708 > /tmp/sp.log 2>&1 &
for i in $(seq 1 20); do
  sleep 1
  kill -0 $GZ 2>/dev/null || { echo ">>> gzserver 在 spawn 后第 ${i}s 崩溃 <<<"; break; }
done
echo "gzserver 最终存活? $(kill -0 $GZ 2>/dev/null && echo yes || echo NO-crashed)"
echo "===== spawn 日志末尾 ====="
tail -12 /tmp/sp.log
echo "===== 新增 dmesg segfault（看崩溃库名）====="
dmesg 2>/dev/null | tail -n +$((MARK+1)) | grep -iE 'gzserver|segfault|traps|general protection' | tail -8 || echo "(无记录)"
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null
