#!/usr/bin/env bash
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
D=~/v2x_ws/src/v2x_demo/v2x_demo
for f in traffic_flow_node.py trajectory_recorder.py trajectory_player.py trajectory_selftest.py; do
  cp "$SIM/src/v2x_demo/v2x_demo/$f" "$D/"
done
sed -i 's/\r//g' "$D"/traffic_flow_node.py "$D"/trajectory_recorder.py "$D"/trajectory_player.py "$D"/trajectory_selftest.py
cp "$SIM/src/v2x_demo/config/traffic_flow.yaml" ~/v2x_ws/src/v2x_demo/config/
sed -i 's/\r//g' ~/v2x_ws/src/v2x_demo/config/traffic_flow.yaml
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
python3 "$D/trajectory_selftest.py"
