#!/bin/bash
# 启动 gzclient 并采样内存，确认是否 OOM
source /opt/ros/humble/setup.bash
export DISPLAY=$(ip route show default | awk '{print $3}'):1
unset LIBGL_ALWAYS_INDIRECT

setsid nohup gzclient > /tmp/gzclient.log 2>&1 < /dev/null &
CPID=$!
echo "gzclient pid $CPID"

for i in $(seq 1 45); do
    sleep 2
    S=$(pgrep -x gzserver >/dev/null && echo S || echo -)
    C=$(pgrep -x gzclient >/dev/null && echo C || echo -)
    MEM=$(free -m | awk '/Mem:/ {print "used="$3" avail="$7}')
    echo "t=$((i*2))s [$S$C] $MEM"
    if [ "$S" = "-" ] && [ "$C" = "-" ]; then
        echo "BOTH DEAD at t=$((i*2))s"
        break
    fi
    if [ "$C" = "-" ]; then
        echo "CLIENT DEAD at t=$((i*2))s"
    fi
done
echo "--- dmesg oom ---"
dmesg 2>/dev/null | grep -iE "oom|out of memory|killed process" | tail -8
