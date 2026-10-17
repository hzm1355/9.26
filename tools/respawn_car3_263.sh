#!/bin/bash
# 单独重 spawn car_3，清掉 diff_drive 插件实例缓存（memory「gz-diffdrive-instance-wedge」：
# 实例卡死后 reset_world / 摆位都盖不掉缓存的轮速与轮位目标，只有重 spawn 该模型能复位）。
#
# 本轮的实测病状：car_3 与 car_2 在入口车道角对角贴死过（等待位原地转的绕轮轴扫掠圆
# 半径 1.265 m，与 car_2 排队位只剩 0.15 m 余量）。贴死期间插件把轮位目标误差攒下来，
# 分小跳摆位（tools/pk144_hopsettle262.sh）把车分开、每跳后静止体检全过，可起步试探
# 仍然复现同一发弹射：v=0.10 给 4 s → 位移 1.830 m、峰值实测 1.747 m/s（理论约 0.40 m），
# 且落点与上一次一模一样。同一时间 car_2 走同一套分小跳流程试探是 0.267 m / 峰值 0.102 m/s
# （增益≈1）。差别只在 car_3 经历过楔住 —— 所以是"缓存的轮位目标在往外放"，不是摆位
# 弹射，也不是算法没修好（算法层已离线证明收敛：自测 40 项 FAIL 0，闭环 7.0 s 收敛）。
#
# car_3 没有相机 sensor，/delete_entity 安全（禁删的是挂了 camera+SensorPlugin 的模型）。
# 实测核对：world 里 car_3 块 161 行、只有 libgazebo_ros_diff_drive.so 插件、
# grep '<sensor|camera|ray' = 0 处。
#
# 重 spawn 落回 world 里 car_3 的**出生位姿** (12.00,-12.80,7.0, yaw=1.5708)（也就是
# parking_zones.yaml 的 staging_point 等待位），0.3 m 自由落体到台面 z≈6.699，
# 不走"运行期瞬移"那条弹射路径。
#
# 步骤：前置核对（shuttle 0 实例 + 目标点净空）→ 抽 SDF → 删 → 生 → 位姿复核 →
# 实测速度闸 → 精摆回等待位 → 静止体检 → 净空复核 → 补牌。
# 速度闸 FAIL 就停在这里，别把节点拉起来接手一辆不动的车。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
PX=12.00
PY=-12.80
PYAW=1.5708
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 0) 前置：shuttle 必须 0 实例（否则删车瞬间它还在发 cmd_vel）==='
N=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')
echo "  shuttle 实例 $N"
[ "$N" = "0" ] || { echo '  !! 先跑 pk134_shuttle_ctl247.sh stop 再来'; exit 1; }

echo '=== 1) 从 world 抽 car_3 模型块成独立 SDF ==='
awk '/<model name="car_3">/{f=1} f{print} f&&/<\/model>/{exit}' "$W" > /tmp/car3_body_263.xml
WC=$(wc -l < /tmp/car3_body_263.xml)
echo "  抽到 $WC 行"
[ "$WC" -gt 100 ] || { echo '  !! 抽取行数不对，world 里 car_3 定义变了'; exit 1; }
# 删车前最后一道安全闸：这一坨里绝不能有 sensor，否则 /delete_entity 会把
# 挂在相机上的 SensorPlugin 一起带走、gzserver 有段错误风险。
NS=$(grep -cE '<sensor|camera|ray' /tmp/car3_body_263.xml)
echo "  sensor/camera/ray 出现 $NS 处"
[ "$NS" = "0" ] || { echo '  !! car_3 定义里出现传感器，别删，停在这里'; exit 1; }
{ echo '<sdf version="1.6">'; cat /tmp/car3_body_263.xml; echo '</sdf>'; } > /tmp/car3_263.sdf

echo '=== 2) /delete_entity car_3 ==='
# Humble 的 gazebo_msgs/DeleteEntity 字段就叫 name（写成 model_name/entity_name 都会
# "Failed to populate field"，删不掉 yet 退出码 0，后面全是空转）
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'car_3'}" 2>&1 | tail -2
sleep 2
if gz model -m car_3 -p 2>&1 | head -1 | grep -aq 'Unable'; then
  echo '  确认：world 里已无 car_3'
else
  echo '  !! 删除没生效，car_3 还在，spawn 会撞名，停在这里'
  exit 1
fi

echo '=== 3) spawn 回出生位姿＝等待位 (12.00,-12.80) 正北，z=7.0 自由落体 ==='
timeout -k 5 40 ros2 run gazebo_ros spawn_entity.py -entity car_3 \
  -file /tmp/car3_263.sdf -x $PX -y $PY -z 7.0 -Y $PYAW 2>&1 | tail -3
sleep 5

echo '=== 4) 位姿复核（应≈12.00,-12.80，z 稳在台面 ≈6.699）==='
# 用仓库里那份 pose155.py：/tmp 会被 WSL 回收，不能依赖上一次留下的副本，
# 而这一步 FAIL 的话后面全是空转。
timeout -k 5 25 python3 $SIM/tools/pose155.py car_3 2>&1 | tr -d '\r' | tail -3 | sed 's/^/  /'

echo '=== 5) 实测速度闸：刹轮 8 s → v=0.10 发 4 s，峰值须 <= 0.40 m/s ==='
# 静止复检不算证据（memory「生效要用效果复核」）：必须主动给一小脚油门看实测峰值。
# car_2 走同一闸的实测是 0.267 m / 峰值 0.102 m/s，那一行就是"通过长什么样"的对照。
timeout -k 8 90 python3 -u $SIM/tools/launch_guard155.py car_3 8 0.10 4 0.40 2>&1 | tr -d '\r' | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "  速度闸退出码 $RC（0 通过 / 2 残余误差还在放 / 3 给油不动）"
[ "$RC" = "0" ] || { echo '  !! 重 spawn 后仍然弹射或不动，这不是插件缓存问题，停在这里查'; exit "$RC"; }

echo '=== 6) 过了闸：试探把车往前推了约 0.4 m，刹轮后精摆回等待位 ==='
timeout -k 5 25 python3 $SIM/tools/hold_zero141.py car_3 1.5 2>&1 | tr -d '\r' | tail -1 | sed 's/^/  /'
gz model -m car_3 -x "$PX" -y "$PY" -z 6.70 -R 0 -P 0 -Y "$PYAW" 2>&1 | tr -d '\r' | sed 's/^/  /'
D=$(gz model -m car_3 -p 2>&1 | tr -d '\r' | head -1)
echo "  复摆后：$D"
sleep 5
E=$(gz model -m car_3 -p 2>&1 | tr -d '\r' | head -1)
echo "  5 s 后：$E"
python3 $SIM/tools/pose_delta144.py "$D" "$E"
RC=$?
echo "  收尾体检退出码 $RC"
if [ "$RC" != "0" ]; then
  echo 'FAIL 复摆后又开始动 —— 说明有别的驱动者在发指令（不是 shuttle，它实例数=0）'
  exit "$RC"
fi

echo '=== 7) 净空复核（绕轮轴扫掠半径 1.265 m，3 m 窗口内看有谁）==='
timeout -k 5 30 python3 $SIM/tools/nearby_live.py car_3 3.0 2>&1 | tr -d '\r' | sed 's/^/  /'

echo '=== 8) 重 spawn 掉回了占位牌纹理，补发默认车牌（否则卡口拍到空号）==='
bash $SIM/tools/apply_default_plates.sh 2>&1 | tr -d '\r' | tail -3 | sed 's/^/  /'

echo 'DONE：car_3 已重 spawn 并通过速度闸，位在等待 (12.00,-12.80)，可以起摆渡车'
exit 0
