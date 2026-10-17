#!/bin/bash
# 卡口链路整栈重建：v2x_demo（识别器）+ v2x_gazebo_plugins（抓拍插件）都要重编，
# 然后整栈重启。插件改了 .so 必须重启 gzserver（它按文件名缓存插件）。
#
# restart6.sh 第 2 步会清空 /tmp/v2x_gates，那里头存着已经复现过问题的真实抓拍帧
# （gate_j2_south-20 的"6 读成 S"、-23 的空帧），它们是 blur_sweep / dbg_rectify
# 的回归输入，删了就再也拿不回来，所以先整目录挪去备份再重启。
set -e -o pipefail
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

if [ -d /tmp/v2x_gates ] && [ -n "$(ls -A /tmp/v2x_gates 2>/dev/null)" ]; then
  BK=/tmp/v2x_gates_$(date +%m%d_%H%M%S)
  cp -r /tmp/v2x_gates "$BK"
  echo "旧抓拍帧已备份到 $BK（$(ls "$BK" | wc -l) 张）"
fi

echo '### 同步并重建 v2x_demo'
rm -rf ~/v2x_ws/src/v2x_demo
cp -r "$SIM/src/v2x_demo" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r//g' {} +

echo '### 同步并重建 v2x_gazebo_plugins'
rm -rf ~/v2x_ws/src/v2x_gazebo_plugins
cp -r "$SIM/src/v2x_gazebo_plugins" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_gazebo_plugins \( -name '*.cc' -o -name '*.hh' \) -exec sed -i 's/\r//g' {} +

cd ~/v2x_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
colcon build --symlink-install --packages-select v2x_demo v2x_gazebo_plugins 2>&1 | tail -8

echo '### 整栈重启'
bash "$SIM/tools/restart6.sh"
