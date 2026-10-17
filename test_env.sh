#!/usr/bin/env bash
# 打印 GazeboRosPaths 产出的 env，并用该 env 直跑 gzserver，看是否复现卡死
pkill -9 -f 'ros2 launch' 2>/dev/null; pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzmaster 2>/dev/null
sleep 2
rm -rf /dev/shm/* /tmp/gazebo-* 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "===== GazeboRosPaths.get_paths() 产出 ====="
python3 - <<'PY'
import sys
sys.path.insert(0, '/opt/ros/humble/share/gazebo_ros/launch')
try:
    from scripts import GazeboRosPaths
    m,p,r = GazeboRosPaths.get_paths()
    print("MODEL_PATH   =", m[:300])
    print("PLUGIN_PATH  =", p[:500])
    print("RESOURCE_PATH=", r[:300])
    print("PLUGIN 条目数 =", len(p.split(':')))
    print("MODEL 条目数  =", len(m.split(':')))
except Exception as e:
    print("读取失败:", e)
PY

echo ""
echo "===== 用 GazeboRosPaths env 直跑 gzserver (最多 18s) ====="
export GAZEBO_MODEL_PATH=$(python3 -c "import sys;sys.path.insert(0,'/opt/ros/humble/share/gazebo_ros/launch');from scripts import GazeboRosPaths;print(GazeboRosPaths.get_paths()[0])")
export GAZEBO_PLUGIN_PATH=$(python3 -c "import sys;sys.path.insert(0,'/opt/ros/humble/share/gazebo_ros/launch');from scripts import GazeboRosPaths;print(GazeboRosPaths.get_paths()[1])")
export GAZEBO_RESOURCE_PATH=$(python3 -c "import sys;sys.path.insert(0,'/opt/ros/humble/share/gazebo_ros/launch');from scripts import GazeboRosPaths;print(GazeboRosPaths.get_paths()[2])")
WORLD=~/v2x_ws/install/v2x_demo/share/v2x_demo/worlds/v2x_intersection.world
timeout 18 gzserver --verbose "$WORLD" -s libgazebo_ros_init.so > /tmp/gz_env.log 2>&1
echo "退出码(124=存活被timeout/其它=崩或退): $?"
echo "----- 是否加载 world -----
"
grep -cE 'Loading world|Connected to gazebo master' /tmp/gz_env.log
echo "----- 日志末尾 -----
"
tail -6 /tmp/gz_env.log
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzmaster 2>/dev/null
