#!/bin/bash
# 同步并重建 v2x_gazebo_plugins（卡口抓拍插件改了 R/B 字节序），然后整栈重启。
# gzserver 按 .so 文件名缓存插件，运行时重新 spawn 不会重载，所以必须重启。
set -e -o pipefail
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

rm -rf ~/v2x_ws/src/v2x_gazebo_plugins
cp -r "$SIM/src/v2x_gazebo_plugins" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_gazebo_plugins -name '*.cc' -exec sed -i 's/\r//g' {} +
find ~/v2x_ws/src/v2x_gazebo_plugins -name '*.hh' -exec sed -i 's/\r//g' {} +

cd ~/v2x_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
colcon build --symlink-install --packages-select v2x_gazebo_plugins 2>&1 | tail -6
ls -la ~/v2x_ws/install/v2x_gazebo_plugins/lib/libv2x_gate.so

bash "$SIM/tools/restart6.sh"
