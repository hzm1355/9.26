#!/bin/bash
PID=$(pgrep -x gzserver | head -1)
echo "gzserver pid=$PID"
if [ -n "$PID" ]; then
    echo "--- gzserver env (ROS/DDS related) ---"
    tr '\0' '\n' < /proc/$PID/environ | grep -iE 'ROS|DOMAIN|DISPLAY|FASTRTPS|RMW' || echo "(none)"
fi
echo "--- ros/gz processes ---"
ps aux | grep -E 'gzserver|gzclient|traffic_light|v2x_|ros2' | grep -v grep | cut -c1-140
