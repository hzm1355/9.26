#!/bin/bash
# 单独重 spawn car_6，清掉 diff_drive 插件实例缓存（memory「gz-diffdrive-instance-wedge」：
# 实例卡死后 reset_world / 摆位都盖不掉缓存的轮速目标，只有重 spawn 该模型能复位）。
#
# 本轮的实测病状：灯杆挪走后四段激励（±v、+0.35/+1.20 ω，每段 800+ 帧）位移与转向
# 全部 0.0000，而全场另外五车照常 —— 是"插件不出力"，不是"被顶住"。
#
# car_6 没有相机 sensor，/delete_entity 安全（禁删的是挂了 camera+SensorPlugin 的模型）。
# 重 spawn 落回 entry_route[0]=(12.00,-13.95,1.5708)，z=7.0 与原 world 出生高度一致，
# 0.3 m 自由落体到台面，不走"运行期瞬移"那条弹射路径。
#
# 步骤：前置核对（shuttle 0 实例）→ 抽 SDF → 删 → 生 → 位姿复核 → 实测速度闸 → 补牌。
# 速度闸 FAIL 就停在这里，别把节点拉起来接手一辆不动的车。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 0) 前置：shuttle 必须 0 实例（否则删车瞬间它还在发 cmd_vel）==='
N=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')
echo "  shuttle 实例 $N"
[ "$N" = "0" ] || { echo '  !! 先跑 unwedge 的杀进程段再来'; exit 1; }

echo '=== 1) 从 world 抽 car_6 模型块成独立 SDF ==='
awk '/<model name="car_6">/{f=1} f{print} f&&/<\/model>/{exit}' "$W" > /tmp/car6_body_155.xml
WC=$(wc -l < /tmp/car6_body_155.xml)
echo "  抽到 $WC 行"
[ "$WC" -gt 100 ] || { echo '  !! 抽取行数不对，world 里 car_6 定义变了'; exit 1; }
{ echo '<sdf version="1.6">'; cat /tmp/car6_body_155.xml; echo '</sdf>'; } > /tmp/car6_155.sdf

echo '=== 2) /delete_entity car_6 ==='
# Humble 的 gazebo_msgs/DeleteEntity 字段就叫 name（写成 model_name/entity_name 都会
# "Failed to populate field"，删不掉 yet 退出码 0，后面全是空转）
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'car_6'}" 2>&1 | tail -2
sleep 2
if gz model -m car_6 -p 2>&1 | head -1 | grep -aq 'Unable'; then
  echo '  确认：world 里已无 car_6'
else
  echo '  !! 删除没生效，car_6 还在，spawn 会撞名，停在这里'
  exit 1
fi

echo '=== 3) spawn 回入口等待位 (12.00,-13.95) 正北 ==='
timeout -k 5 40 ros2 run gazebo_ros spawn_entity.py -entity car_6 \
  -file /tmp/car6_155.sdf -x 12.00 -y -13.95 -z 7.0 -Y 1.5708 2>&1 | tail -3
sleep 5

echo '=== 4) 位姿复核（应≈12.00,-13.95，z 稳在台面）==='
# 用仓库里那份 pose155.py：/tmp 会被 WSL 回收，昨天留下的副本今天不一定还在，
# 而这一步 FAIL 的话后面全是空转。
timeout -k 5 25 python3 $SIM/tools/pose155.py car_6 2>&1 | tail -2

echo '=== 5) 实测速度闸：v=0.10 发 5 s，位移须 >= 理论 0.5 m 的 50% ==='
timeout -k 20 60 python3 $SIM/tools/launch_guard155.py car_6 3 0.10 5 0.30
RC=$?
echo "  launch_guard rc=$RC"
[ "$RC" = "0" ] || { echo '  !! 重 spawn 后仍然不动，这不是插件缓存问题，停在这里查'; exit "$RC"; }

echo '=== 6) 重 spawn 掉回了占位牌纹理，补发默认车牌 ==='
bash $SIM/tools/apply_default_plates.sh 2>&1 | tail -3

echo 'DONE：car_6 已重 spawn 并通过速度闸'
exit 0
