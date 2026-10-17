#!/usr/bin/env bash
# 新场景 v2x_city.world 加载验证：模型清单/分类计数/报错/话题/车位姿
export GAZEBO_MASTER_URI=http://localhost:11345
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '== 1) gzserver 进程 =='
pgrep -a -f '[g]zserver' | head -1 | cut -c1-80

echo '== 2) 模型总数 =='
gz model -l 2>/dev/null | wc -l

echo '== 3) 分类计数 =='
LIST=$(gz model -l 2>/dev/null)
for pat in road_ zebra_ stopline_ dash_ median_ edge_ ETC- VIADUCT- PKG-NW SCHOOL TL-J RSU- TREES- building_ obstacle; do
  n=$(echo "$LIST" | grep -c "$pat")
  printf '  %-12s %s\n' "$pat" "$n"
done

echo '== 4) gzserver.log 错误检查 =='
grep -ci 'error' /tmp/gzserver.log || true
grep -i 'error' /tmp/gzserver.log | grep -iv 'errno|no error' | head -5 || echo '  无 error 行'

echo '== 5) 信号灯 / TF / 话题 =='
timeout 4 ros2 topic list 2>/dev/null | grep -E '^(traffic_light/state|/scan|/odom|/cmd_vel)$'
timeout 4 ros2 topic echo /traffic_light/state --once 2>/dev/null | head -2 || true

echo '== 6) 车位姿 =='
timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr -d ' ' | tr '\n' ' '
echo ''
echo '[verify] 完成'
