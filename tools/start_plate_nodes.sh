#!/bin/bash
# 同步并重建 v2x_demo（含新的 plate_recognizer_node.py），然后拉起车牌生成 + 参考识别。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID

rm -rf ~/v2x_ws/src/v2x_demo
cp -r "$SIM/src/v2x_demo" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r//g' {} +
cd ~/v2x_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select v2x_demo 2>&1 | tail -3

source ~/v2x_ws/install/setup.bash
ls ~/v2x_ws/install/v2x_demo/lib/v2x_demo/ | grep plate_

pkill -f 'plate_gen_node' 2>/dev/null || true
pkill -f 'plate_recognizer_node' 2>/dev/null || true
sleep 1
setsid nohup ros2 run v2x_demo plate_gen_node.py > /tmp/plate_gen.log 2>&1 < /dev/null &
setsid nohup ros2 run v2x_demo plate_recognizer_node.py > /tmp/plate_rec.log 2>&1 < /dev/null &
sleep 8
echo "--- 存活 ---"
pgrep -af 'plate_gen_node|plate_recognizer_node' | cut -c1-90 || echo "(都没起来)"
echo "--- 识别器日志 ---"
tail -6 /tmp/plate_rec.log || true
echo "--- 生成器日志 ---"
tail -3 /tmp/plate_gen.log || true
