#!/bin/bash
# 拉起信号灯管理器（车队能正常跑环路的前提）。
#
# 为什么单独一个脚本：管理器不在，灯态就停在最后一次广播上不再轮转，signal_drive 的
# 车会一直在红灯前等 → 触发它的"卡死脱困"复位 → 车被瞬移到路线起点甚至落到界外，
# 现场就变成复位风暴（实测 20 s 内位移 30~49 m、测速窗口均速 6~12 km/h 的废读数）。
# 参数与 tools/vid_common.sh 的 mgr_restart() 保持一致，两处改动要同步。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TL_PARAMS=$SIM/src/v2x_demo/config/traffic_light_params.yaml
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

N=$(pgrep -cf 'traffic_light_manager[.]py' || true)
echo "[tlmgr] 现有实例 $N"
if [ "$N" != "0" ]; then
  echo "[tlmgr] 先杀光旧实例（双实例会各自广播配时，把灯态搅成随机）"
  for P in $(pgrep -f 'traffic_light_manager[.]py'); do kill -9 "$P" 2>/dev/null || true; done
  sleep 2
fi
[ -s "$TL_PARAMS" ] || { echo "!! 找不到参数文件 $TL_PARAMS"; exit 1; }

setsid nohup ros2 run v2x_demo traffic_light_manager.py --ros-args \
  -p use_sim_time:=true -p enable_statistics:=true -p enable_time_schedule:=true \
  -p config_file:="$TL_PARAMS" > /tmp/tlmgr_vid.log 2>&1 < /dev/null &
sleep 16
echo "[tlmgr] 拉起后实例 $(pgrep -cf 'traffic_light_manager[.]py' || true)"
echo "[tlmgr] PID $(pgrep -f 'lib/v2x[_]demo/traffic_light_manager' | tr '\n' ' ')"
tail -4 /tmp/tlmgr_vid.log | cut -c1-140
exit 0
