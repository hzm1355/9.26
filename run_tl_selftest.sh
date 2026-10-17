#!/usr/bin/env bash
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
D=~/v2x_ws/src/v2x_demo/v2x_demo
cp "$SIM/src/v2x_demo/v2x_demo/traffic_light_manager.py" "$D/"
cp "$SIM/src/v2x_demo/v2x_demo/traffic_light_selftest.py" "$D/"
sed -i 's/\r//g' "$D/traffic_light_manager.py" "$D/traffic_light_selftest.py"
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
python3 "$D/traffic_light_selftest.py"
