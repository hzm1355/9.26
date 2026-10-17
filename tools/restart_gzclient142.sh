#!/bin/bash
# 只重启 gzclient，不动 gzserver —— 世界、仿真时钟、所有车的位置都不受影响。
#
# 为什么客户端会"看不到 car_6"：car_6 是运行时 /delete_entity + /spawn_entity 重生的，
# 客户端的模型树与 OGRE 场景靠 CreateModel/DeleteModel 消息同步；#142 实测那棵树连
# 名字都停在旧版（lj_j1_ns，服务端早就是 tl_j1_ns），所以重启客户端让它重新拉一遍
# 完整场景是唯一干净的办法。
#
# 关键约束（memory「城市沙盘 GUI 来源与重开」）：客户端必须和服务端共用同一套
# GAZEBO_MODEL_PATH，否则解析不到 model://room_33 的网格，会在 "Preparing your world..."
# 卡两三分钟后静默退出，日志还几乎为空 —— 所以这里是从 /proc/<server>/environ 抄 env，
# 不是自己拼一份。
set +e
DISPLAY_IN="${1:-172.26.32.1:1}"

S=$(pgrep -x gzserver | head -1)
if [ -z "$S" ]; then
  echo "!! 没有在跑的 gzserver —— 客户端连不上东西，先整栈重启（那会打断仿真，需另行确认）"
  exit 1
fi
echo "== gzserver pid=$S，运行时长 $(ps -o etimes= -p $S | tr -d ' ') s =="
echo "   cwd: $(ls -l /proc/$S/cwd 2>&1 | sed 's/.*-> //')"
SMP=$(tr '\0' '\n' < /proc/$S/environ | grep -a '^GAZEBO_MODEL_PATH=' | cut -d= -f2-)
SPP=$(tr '\0' '\n' < /proc/$S/environ | grep -a '^GAZEBO_PLUGIN_PATH=' | cut -d= -f2-)
SMU=$(tr '\0' '\n' < /proc/$S/environ | grep -a '^GAZEBO_MASTER_URI=' | cut -d= -f2-)
echo "   GAZEBO_MODEL_PATH=$SMP"
echo "   GAZEBO_PLUGIN_PATH=$SPP"
echo "   GAZEBO_MASTER_URI=${SMU:-默认 localhost:11345}"

for P in $(pgrep -x gzclient); do
  echo "== 杀掉旧 gzclient pid=$P =="
  kill -9 "$P" 2>/dev/null
done
sleep 2
echo "   剩余 gzclient 实例：$(pgrep -xc gzclient)"

export DISPLAY="$DISPLAY_IN"
unset WAYLAND_DISPLAY
unset LIBGL_ALWAYS_INDIRECT
[ -n "$SMP" ] && export GAZEBO_MODEL_PATH="$SMP"
[ -n "$SPP" ] && export GAZEBO_PLUGIN_PATH="$SPP"
[ -n "$SMU" ] && export GAZEBO_MASTER_URI="$SMU"
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1

: > /tmp/gzclient_run.log
setsid nohup gzclient --verbose > /tmp/gzclient_run.log 2>&1 < /dev/null &
echo "== 已拉起 gzclient（DISPLAY=$DISPLAY_IN），等 60 s 让它拉完沙盘网格 =="
sleep 60

P=$(pgrep -x gzclient | head -1)
if [ -z "$P" ]; then
  echo "!! 60 s 后 gzclient 不在了，日志尾："
  tail -15 /tmp/gzclient_run.log | tr -d '\r' | sed 's/^/   /'
  exit 1
fi
echo "== 存活 pid=$P，RSS $(ps -o rss= -p $P | tr -d ' ') KB（约 2.5 GB 才说明 OGRE 载完了场景）=="
echo "   X 连接数：$(ss -tn 2>/dev/null | grep -ac ':6001')"
echo "   日志尾："
tail -8 /tmp/gzclient_run.log 2>/dev/null | tr -d '\r' | cut -c1-160 | sed 's/^/     /'
echo "   car_6 位姿（服务端）：$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)"
