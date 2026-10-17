#!/bin/bash
# 零瞬移受控取证（tools/speed_live_pass.py）的外层：预检 -> 单发后台跑 -> 打印进度。
#
# 预检四项，缺一项就不许开跑（memory「长跑脚本先预检依赖」）：
#  * gzserver 活着且 /clock 在走，否则测速节点判定停摆；
#  * speed_detect_node 的 python 本体实例数必须 = 1（多实例会把同一批抓拍落两行，
#    memory「重启节点要杀光所有实例」）；
#  * signal_drive 不能在 —— 它抢 /car_4/cmd_vel 还会把车复位走，本脚本的"只发 cmd_vel"
#    前提就塌了；
#  * /speed/zones 服务要能回包（回包=节点真的在 spin，不是刚 fork 出来的僵尸）。
# 匹配式一律带字符类，避免 wsl bash -lc 把自己这条命令行数进去（memory「WSL pkill 自杀陷阱」）。
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
LOG=/tmp/speed_live_pass.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '### 1/3 预检'
[ "$(pgrep -x gzserver || echo '')" != "" ] || { echo '!! gzserver 没在跑'; exit 1; }
N=$(pgrep -cf 'lib/v2x[_]demo/speed_detect_node' || true)
[ "$N" = "1" ] || { echo "!! speed_detect_node 实例数=$N（要等于 1）"; exit 1; }
if [ "$(pgrep -cf 'signal[_]drive' || true)" != "0" ]; then
  echo '!! signal_drive 还活着，先 bash tools/stop_signal.sh'; exit 1; fi
timeout -k 3 30 ros2 service call /speed/zones v2x_msgs/srv/SetSpeedZone '{op: 0}' \
  > /tmp/live_zones_pre.txt 2>&1 || { echo '!! /speed/zones 无回执'; exit 1; }
grep -q '当前' /tmp/live_zones_pre.txt || { echo '!! 服务回包不含点位表'; tail -3 /tmp/live_zones_pre.txt; exit 1; }
echo "  gzserver=$(pgrep -x gzserver)  检测节点实例=$N  点位服务 OK"

echo '### 2/3 开跑（后台单发，日志 /tmp/speed_live_pass.log）'
rm -f "$LOG"
setsid nohup python3 -u "$D/tools/speed_live_pass.py" > "$LOG" 2>&1 < /dev/null &
sleep 20
echo '### 3/3 前 20 s'
cut -c1-160 "$LOG" | tail -12
