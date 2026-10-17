#!/bin/bash
# 把弹射到东北角的 car_3 收回等待位，补牌，只重起摆渡车，再盯一个完整进场周期并对账。
#
# 背景（都是这一轮实测，不是推测）：
#   * car_2 归队后（tools/pk181_car2home308.sh 的 A6 实测：位移 4.743 m、已离开 y<-14.2），
#     同一位形的三态探针立刻读出**大出力**：ω=0.6 给 6 s -> Δyaw -74.09°、轮关节点峰值
#     2.250 rad/s、峰值线速 2.347 m/s。判据脚本 DECIDE=RESPONDED -> **插件没卡死，不需要
#     /delete_entity**，上一轮"给什么都不转"是被 car_2 顶死时的读数（tools/pk180_obb308.py：
#     复位格的扫掠 29 档里 28 档与 car_2 重叠、最深 0.257 m）。
#   * 但解锁那一拍是**带弹射的**：峰值线速 2.347 m/s 远超指令（纯 ω 指令下理论不该有平动），
#     探针收尾读到 (14.37,-11.05)，之后一路滑到 (24.47,-5.78)（台面东边缘 x=25，只剩 0.5 m），
#     途中压过 gate_in 地感，撞出一条 `P0016-gate_in 进场 牌=无 未登记 挂起｜识别无结果`。
#     这条挂起记录不会挡住下一轮到场（reject() 清 g.cycle），但**牌要重贴**：车被推搡过，
#     而且 2.35 m/s 侧向扫过相机本来也拍不清。
#
# 沿用现场三条硬规矩：刹轮 1.5 s -> 摆位 -> 静默 1.0 s -> 静止复检（memory
# 「摆位弹射与复位风暴」：根链接瞬移而子链接留原地，轮子还在受驱时必弹）；
# 杀 cmd_vel 发布者后必须补零速；只重起 shuttle，不碰 manager/recorder —— 它们的
# 内存环形记录是条目 5 的查询源，重起就把这轮已有的 P0014/P0015 从 /parking/query 里抹掉。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
PX=12.00
PY=-12.80
PYAW=1.5708
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 0) 前置：谁还在发 /car_3/cmd_vel + car_3 是不是已经停住 ==="
echo "  shuttle 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')（要求 0）"
echo "  manager/recorder 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')/$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')（要求各 1，本脚本不动它们）"
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node')（car_3 是 drive:false，它不该碰 car_3）"
echo "  /car_3/cmd_vel 发布者（DDS 计数有 ~21 s 滞后，只作参考不作闸）：$(timeout -k 2 12 ros2 topic info /car_3/cmd_vel 2>&1 | tr -d '\r' | grep -a 'Publisher count')"
A=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
echo "  car_3 第 1 拍：$A"
sleep 4
B=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
echo "  car_3 第 2 拍：$B"
python3 "$SIM/tools/pose_delta144.py" "$A" "$B"
MOV=$?
if [ "$MOV" != "0" ]; then
  echo '  还在滑 -> 先灌 4 s 零速再复检（静止复检不算证据，所以要连发而不是发一条）'
  timeout -k 3 20 python3 "$SIM/tools/hold_zero141.py" car_3 4 2>&1 | tail -2 | sed 's/^/    /'
  C=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
  sleep 4
  D=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
  echo "  零速后：$C"
  echo "  4 s 后：$D"
  python3 "$SIM/tools/pose_delta144.py" "$C" "$D"
  if [ "$?" != "0" ]; then
    echo '!! 灌了零速还在动 —— 有别的驱动者在发指令，摆位必弹，停在这里'
    exit 1
  fi
fi

echo
echo "=== 1) 刹轮 1.5 s -> 摆回等待位 (12.00,-12.80, yaw 1.5708) -> 静默 1.0 s ==="
timeout -k 3 20 python3 "$SIM/tools/hold_zero141.py" car_3 1.5 2>&1 | tail -1 | sed 's/^/  /'
gz model -m car_3 -x $PX -y $PY -z 6.70 -R 0 -P 0 -Y $PYAW 2>&1 | tr -d '\r' | sed 's/^/  /'
timeout -k 3 20 python3 "$SIM/tools/hold_zero141.py" car_3 1.0 2>&1 | tail -1 | sed 's/^/  /'
S1=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
echo "  摆后：$S1"
sleep 5
S2=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
echo "  5 s 后：$S2"
python3 "$SIM/tools/pose_delta144.py" "$S1" "$S2"
if [ "$?" != "0" ]; then
  echo 'FAIL 摆完还在自走/被弹 -> 别再摆第二次（会攒轮位误差），先查谁在发指令'
  exit 1
fi

echo
echo "=== 2) 净空复核：设施件（含 c130 已删的这份 world）+ 邻车活位姿 ==="
python3 "$SIM/tools/pk177_worldprechk308.py" --spin=$PX,$PY,0.0,$PYAW 2>&1 | tail -12 | sed 's/^/  /'
timeout -k 5 30 python3 "$SIM/tools/nearby_live.py" car_3 3.0 2>&1 | tr -d '\r' | sed 's/^/  /'

echo "=== 3) 重贴默认车牌（入口拍车头、出口拍车尾，少一块就只能走临时车分支）==="
bash "$SIM/tools/apply_default_plates.sh" 2>&1 | tail -6 | sed 's/^/  /'
echo "  退出码 ${PIPESTATUS[0]}（0=全部到位）"

echo
echo "=== 4) 只重起摆渡车（保住 manager/recorder 的内存环形记录）==="
NP0=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA0=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  起前账本：通行 $NP0 行 / 闸机动作 $NA0 行"
SHUTTLE_MODELS=car_3 bash "$SIM/tools/start_parking_nodes.sh" parking_shuttle_node.py 2>&1 | tail -20 | sed 's/^/  /'
echo "  shuttle 就绪行：$(grep -a '就绪' /tmp/shuttle_car_3.log | tail -1 | cut -c1-150)"

echo "=== 5) 盯 36 轮 ≈360 s：位姿 + 两闸角度 + 账本行数 + 节点日志尾 ==="
bash "$SIM/tools/pk_watch308.sh" 36 car_3 2>&1 | tail -80 | sed 's/^/  /'

echo
echo "=== 6) 收尾对账 ==="
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  通行账本：$NP0 -> $NP1 行；闸机动作：$NA0 -> $NA1 行"
echo '  ## parking_pass.jsonl 尾 4 行'
tail -4 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-220 | sed 's/^/     /'
echo '  ## 本轮新增的入口抓拍样张'
ls -t /tmp/v2x_parking_in/ 2>/dev/null | head -4 | sed 's/^/     /'
echo "  ## 关键词计数：停驶挂起 $(grep -ac '停驶挂起' /tmp/shuttle_car_3.log) / 没转过 $(grep -ac '没转过' /tmp/shuttle_car_3.log) / 顶住到上限 $(grep -ac '顶住次数到上限' /tmp/shuttle_car_3.log)"
echo "  ## manager 里那条 牌=无 挂起之后有没有新的到场：$(grep -a '到场' /tmp/parking_manager_node.log | tail -2 | tr '\n' ' ' | cut -c1-200)"
echo '  ## manager 尾 6 行'
tail -6 /tmp/parking_manager_node.log | cut -c1-170 | sed 's/^/     /'
echo "  ## car_3 终态：$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo "  ## car_2 终态：$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo 'PK182DONE'
exit 0
