#!/bin/bash
# 同步并重建 v2x_msgs + v2x_demo（测速模块新增的 4 个 msg / 1 个 srv / 2 个节点）。
# 编译失败就停在这里：colcon 的输出必须自己判退出码，接到 tail 里会把失败吞掉，
# 于是下一环节拿旧接口跑数据，全按"已修复"解读（build_gate.sh 的头就为这个写的）。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

for PKG in v2x_msgs v2x_demo; do
  rm -rf ~/v2x_ws/src/$PKG
  cp -r "$SIM/src/$PKG" ~/v2x_ws/src/
done
find ~/v2x_ws/src/v2x_msgs ~/v2x_ws/src/v2x_demo \
  \( -name '*.py' -o -name '*.msg' -o -name '*.srv' -o -name '*.yaml' \) -exec sed -i 's/\r//g' {} +

cd ~/v2x_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
# 顺序不能反：v2x_demo 的节点 import 生成码，msgs 没先出接口就是假绿。
if colcon build --symlink-install --packages-select v2x_msgs v2x_demo > /tmp/v2x_speed_build.log 2>&1; then
  echo "BUILD_OK"
else
  echo "BUILD_FAILED"
  grep -nE 'error|Error|失败' /tmp/v2x_speed_build.log | head -30
  exit 1
fi
echo "--- 新增接口 ---"
python3 -c "
from v2x_msgs.msg import SpeedCapture, SpeedSegment, SpeedAlert, SpeedStats
from v2x_msgs.srv import SetSpeedZone
print('import ok: SpeedCapture/SpeedSegment/SpeedAlert/SpeedStats/SetSpeedZone',
      'MODE_SEGMENT=%d KIND_INSTANT=%d OP_ADD=%d' % (
          SpeedStats.MODE_SEGMENT, SpeedAlert.KIND_INSTANT, SetSpeedZone.Request.OP_ADD))
"
echo "--- 新增节点 ---"
ls ~/v2x_ws/install/v2x_demo/lib/v2x_demo/ | grep speed
echo "--- 配置 ---"
ls -la ~/v2x_ws/install/v2x_demo/share/v2x_demo/config/speed_zones.yaml
