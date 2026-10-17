#!/usr/bin/env bash
# 只重启 gzserver/gzclient 重载 world，不动 ROS 侧的 plate_gen / recognizer / recorder。
# 用途：改的只是 .world（没碰 C++），没必要走 rebuild_all_restart.sh 的 4 分钟编译。
# 注意 world 里的卡口插件在 gzserver 进程内，所以相机抓拍话题会随重启断一次再自动恢复。
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID

echo '### 1/3 停掉 gzserver/gzclient'
for i in 1 2 3; do
  P="$(pgrep -x gzserver || true) $(pgrep -x gzclient || true)"
  [ "${P// /}" = "" ] && break
  kill -9 $P 2>/dev/null || true
  sleep 3
done
echo "残留: server='$(pgrep -x gzserver || true)' client='$(pgrep -x gzclient || true)'"

echo '### 2/3 启动 gzserver + gzclient'
cd $D
bash scripts/start_gazebo_city.sh 2>&1 | tail -2
sleep 8
bash scripts/start_gzclient.sh 2>&1 | tail -2

echo '### 3/3 等世界加载完（约 60 s），期间不要发指令'
sleep 60
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
echo "gate 话题数: $(ros2 topic list 2>/dev/null | grep -c gate)"
echo "光照插件: $(timeout -k 3 10 ros2 topic echo /gate/lighting/current --once 2>/dev/null | tr -d '\r' || echo '(还没回读)')"
echo '[restart_gazebo_only] DONE'
