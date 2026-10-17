#!/bin/bash
# 只读体检：谁在跑、跑多久、五辆车现在在哪。取证开跑前先跑这个，别靠猜。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/v2x_ws/install/setup.bash 2>/dev/null

echo "--- 相关进程（排除本脚本自己：模式都带字符类）---"
for PAT in 'gzserv[e]r' 'gzclie[n]t' 'signal[_]drive' 'speed[_]detect[_]node' 'speed[_]recorder[_]node' 'recover[_]fleet' 'free[_]roll' 'speed[_]ctrl'; do
  for PID in $(pgrep -f "$PAT" 2>/dev/null); do
    ET=$(ps -o etimes= -p "$PID" 2>/dev/null | tr -d ' ')
    CMD=$(tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-120)
    echo "  pid=$PID ${ET}s $CMD   <=$PAT"
  done
done

echo "--- 五车实时位姿（3 s 采样，看是否还在滑）---"
timeout -k 3 40 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/fleet_peek.py 6 2>&1 | tail -14
