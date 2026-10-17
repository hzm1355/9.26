#!/bin/bash
# 用隔离的 master 端口 + 独立 ROS domain headless 加载改了的世界，
# 只看"能不能起来、有没有 parse/exception、新模型在不在"，不碰正在跑的 11345 会话。
# 注意：脚本内部用普通 & 起 gzserver（$! 就是它的 PID），结束时自己杀掉，不用 setsid。
set -e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
export GAZEBO_MASTER_URI=http://localhost:11346
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH=/home/hzm/v2x_models:/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/models
unset DISPLAY
unset WAYLAND_DISPLAY
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

LOG=/tmp/gzverify.log
rm -f "$LOG"
gzserver --verbose \
  /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so >"$LOG" 2>&1 &
PID=$!
echo "gzserver pid=$PID"
sleep 75
if kill -0 "$PID" 2>/dev/null; then echo ALIVE=yes; else echo ALIVE=no; fi
wc -l "$LOG"

echo '--- 世界加载与仿真时序 ---'
grep -aE 'Entering the update loop|Physics: 20000' "$LOG" | tail -2 || true
echo '--- 致命类报错（parse/exception/mesh）---'
grep -aiE 'err|exception|failed to parse|not found|unable' "$LOG" \
  | grep -aviE "can't open display|unable to create x window|render path type is none|x11 output" \
  | tail -12 || true
echo '--- 新等效盒是否载入 ---'
timeout 20 gz model -i -m sandbox_fixtures_coll 2>&1 | head -6 || true
echo '--- 车辆模型仍在 ---'
timeout 20 gz model -i -m car_1 2>&1 | grep -aE 'name|position' | head -4 || true
echo '--- 收尾 ---'
kill -9 "$PID" 2>/dev/null || true
sleep 2
if kill -0 "$PID" 2>/dev/null; then echo "残留 pid=$PID"; else echo "已清理 pid=$PID"; fi
cp "$LOG" /mnt/d/desktop1/9.26.1/_gzverify.log
echo DONE
