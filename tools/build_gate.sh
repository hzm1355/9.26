#!/bin/bash
# 只重建卡口插件，编译失败就停在这里——不重启栈。
# 上一版把 colcon 的输出接到 tail 里，失败被吞掉，结果 restart6.sh 照样跑了一遍，
# 栈里留下的还是旧 .so，测出来的数据全按"已修复"解读。这里显式检查退出码。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

rm -rf ~/v2x_ws/src/v2x_gazebo_plugins
cp -r "$SIM/src/v2x_gazebo_plugins" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_gazebo_plugins -name '*.cc' -exec sed -i 's/\r//g' {} +
find ~/v2x_ws/src/v2x_gazebo_plugins -name '*.hh' -exec sed -i 's/\r//g' {} +

cd ~/v2x_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
if colcon build --symlink-install --packages-select v2x_gazebo_plugins > /tmp/v2x_build.log 2>&1; then
  echo "BUILD_OK"
else
  echo "BUILD_FAILED"
  grep -E 'error:|Error' /tmp/v2x_build.log | head -20
  exit 1
fi
ls -la ~/v2x_ws/install/v2x_gazebo_plugins/lib/libv2x_gate.so
