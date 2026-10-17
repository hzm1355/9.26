#!/usr/bin/env bash
# 同步 Windows 侧 v2x 源码到 WSL 工作区并编译（在 WSL 内运行）。
# 用法: wsl -d Ubuntu-22.04 -- bash -c "sed 's/\r$//' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/sync_build.sh | bash"
# 注：~/v2x_ws/src 下各包是 Windows 源码的派生副本，本脚本整包替换。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PKGS="v2x_msgs v2x_demo v2x_gazebo_plugins"
rm -rf ~/v2x_ws/src/v2x_demo ~/v2x_ws/src/v2x_msgs ~/v2x_ws/src/v2x_gazebo_plugins
cp -r "$SIM/src/v2x_demo" ~/v2x_ws/src/
cp -r "$SIM/src/v2x_msgs" ~/v2x_ws/src/
cp -r "$SIM/src/v2x_gazebo_plugins" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo ~/v2x_ws/src/v2x_msgs -name '*.py' -exec sed -i 's/\r//g' {} +
# C++/CMake 源同样可能带 CRLF，去除以免编译告警/失败
find ~/v2x_ws/src/v2x_gazebo_plugins \( -name '*.cc' -o -name '*.cpp' -o -name '*.txt' -o -name '*.xml' \) -exec sed -i 's/\r//g' {} +
cd ~/v2x_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select $PKGS
echo '[sync_build] OK'
