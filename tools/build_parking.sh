#!/bin/bash
# 同步并重建停车场模块（任务五）涉及的包：v2x_msgs（3 msg + 4 srv）、v2x_demo
# （3 个节点 + parking_zones.yaml），需要时再带 v2x_gazebo_plugins（道闸插件）。
#
# 沿用 build_speed.sh 的两条硬规矩：
#   1) 先删后拷 + 去 CR —— D 盘写出来的文件是 CRLF，colcon 与 python 都会栽在 \r 上；
#   2) 编译退出码自己判，不把 colcon 输出直接接进 tail，否则失败被吞，
#      下一环节拿旧接口跑数据，全按"已修复"解读。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PKGS="v2x_msgs v2x_demo"
[ "$1" = "--with-plugin" ] && PKGS="$PKGS v2x_gazebo_plugins"

for PKG in $PKGS; do
  rm -rf ~/v2x_ws/src/$PKG
  cp -r "$SIM/src/$PKG" ~/v2x_ws/src/
done
for FPKG in $PKGS; do find ~/v2x_ws/src/$FPKG \
  \( -name '*.py' -o -name '*.msg' -o -name '*.srv' -o -name '*.yaml' \
     -o -name '*.cc' -o -name '*.hh' -o -name 'CMakeLists.txt' \) -exec sed -i 's/\r//g' {} +; done

cd ~/v2x_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
# 顺序不能反：v2x_demo 的节点 import 生成码，msgs 没先出接口就是假绿。
if colcon build --symlink-install --packages-select $PKGS > /tmp/v2x_parking_build.log 2>&1; then
  echo "BUILD_OK"
else
  echo "BUILD_FAILED"
  grep -nE 'error|Error|失败' /tmp/v2x_parking_build.log | head -40
  exit 1
fi

echo "--- 新增接口 ---"
python3 -c "
from v2x_msgs.msg import BarrierState, ParkingPass, ParkingAction
from v2x_msgs.srv import SetParkingGate, SetParkingWhitelist, ParkingQuery, ParkingExport
print('import ok: BarrierState/ParkingPass/ParkingAction + 4 srv')
print('  BarrierState.STATE_FAULT=%d FAULT_STUCK=%d SRC_MANUAL=%d' % (
    BarrierState.STATE_FAULT, BarrierState.FAULT_STUCK, BarrierState.SRC_MANUAL))
print('  ParkingPass.DIR_OUT=%d KIND_TEMP=%d RESULT_DENIED=%d' % (
    ParkingPass.DIR_OUT, ParkingPass.KIND_TEMP, ParkingPass.RESULT_DENIED))
print('  ParkingAction.ACT_HOLD_OPEN=%d ACT_DEGRADE=%d' % (
    ParkingAction.ACT_HOLD_OPEN, ParkingAction.ACT_DEGRADE))
print('  SetParkingGate.OP_FAULT=%d SetParkingWhitelist.OP_ISSUE_TEMP=%d' % (
    SetParkingGate.Request.OP_FAULT, SetParkingWhitelist.Request.OP_ISSUE_TEMP))
print('  ParkingQuery.DIR_ANY=%d ParkingExport 请求字段=%s' % (
    ParkingQuery.Request.DIR_ANY, sorted(ParkingExport.Request.get_fields_and_field_types())))
"
echo "--- 配置 ---"
ls -la ~/v2x_ws/install/v2x_demo/share/v2x_demo/config/parking_zones.yaml 2>/dev/null || echo "（parking_zones.yaml 尚未安装）"
if [ "$1" = "--with-plugin" ]; then
  echo "--- 插件 ---"
  ls -la ~/v2x_ws/install/v2x_gazebo_plugins/lib/libv2x_barrier.so 2>/dev/null || echo "（libv2x_barrier.so 尚未产出）"
fi
