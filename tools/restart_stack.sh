#!/bin/bash
set -e
unset ROS_DOMAIN_ID

pkill -f '^python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/loop_drive.py' 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
sleep 2
pkill -x gzserver 2>/dev/null || true
sleep 3
kill -9 $(pgrep -x gzserver) 2>/dev/null || true
kill -9 $(pgrep -x gzclient) 2>/dev/null || true
sleep 1
echo "killed, remaining gz: $(pgrep -x gzserver | wc -l) $(pgrep -x gzclient | wc -l)"

bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/start_gazebo_city.sh
sleep 2
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/start_gzclient.sh
echo "waiting for world..."
sleep 28

echo "=== car_1 pose ==="
gz model -m car_1 -p || true
echo "=== topics ==="
source /opt/ros/humble/setup.bash
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/install/setup.bash 2>/dev/null || true
ros2 topic list | tee /tmp/topics.txt | wc -l
