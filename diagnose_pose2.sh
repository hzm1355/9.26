#!/usr/bin/env bash
# 车状态深查：chassis link 真实位姿 + odom 连续 + 里程 + scan
export GAZEBO_MASTER_URI=http://localhost:11345
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '--- chassis link pose（真实几何位姿） ---'
gz model -m v2x_prius -i 2>&1 | grep -A9 'name: "v2x_prius::chassis"' | head -13

echo '--- odom 连续 3 条（看是否在动） ---'
for i in 1 2 3; do
  P=$(timeout 3 ros2 topic echo /odom --once --field pose.pose 2>/dev/null \
      | grep -E 'x:|y:|z:|w:' | tr -d ' ' | tr '\n' ' ')
  echo "  #$i  $P"
done

echo '--- distance 里程（插件累计） ---'
timeout 3 ros2 topic echo /distance --once 2>&1 | head -4

echo '--- scan 是否还在发（车传感器活性） ---'
timeout 4 ros2 topic echo /scan --once --field ranges 2>/dev/null | head -3
