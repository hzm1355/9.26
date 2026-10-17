#!/bin/bash
# pk183：等待位南移（-12.80 -> -13.95）上线 —— 把 car_3 摆到新等待位、重贴牌、
# 只重起 shuttle，盯满一个含"两次进场抓拍"的观察窗（旧几何在第二轮零抓拍并挂起）。
#
# 背景与依据（都是本轮实测/离线复核，不是推测）：
#   * 旧几何第二轮零抓拍的机制已证死：reset#2 在原地转（转点 12.00,-12.40）把
#     底盘原点甩进 near 1.4 触发圈，误拍一帧（P0019 d=1.4000 置信 0.1671）置
#     armed=true；而进场车道内 d 最大 1.59 < far 2.5 永远清不掉 → 后续所有真实
#     接近全被静音 → 等杆超时 ×3 → 停驶挂起（/tmp/shuttle_car_3.log 三连
#     "等 gate_in 抬杆 30 s 没等到"）。
#   * 修法 = 等待位南移到 (12.00,-13.95)（只动 entry#0 与 reset#1/#2 三点，yaml 已改）：
#     静止 2.74 m > far 2.5 —— 每轮回到等待位自动清锁；原地转机动底盘原点最小
#     1.94 m > near 1.4 —— 全程在触发圈外。离线判据 tools/pk184_clearprechk308.py
#     退出码 0（校准复现旧几何 1.59/0.79 两个实测值）。
#   * 摆位流程照 pk182 现场三硬规矩：刹轮 1.5 s -> 摆位 -> 静默 1.0 s -> 静止复检；
#     杀 cmd_vel 发布者后补零速；只重起 shuttle，manager/recorder 不碰（它们的
#     内存环形记录是条目 5 的查询源，重起就把已有通行记录从 /parking/query 里抹掉）。
#
# 预期对账（RTF≈0.45，每轮 120 sim-s ≈ 265 s 墙钟）：
#   [1] 第 1 轮进场抓拍（重启后 ~10 s 内）
#   [2] 第 1 轮跑完（~85 sim-s ≈ 190 s 墙钟）—— 含新 reset#2 转（旧几何在此误拍）
#   [3] 第 2 轮进场抓拍（~125 sim-s ≈ 280-300 s 墙钟）← 旧几何零抓拍的判死点
#   [4] 停驶挂起 = 0；本轮作废 = 0
#   [5] 第 2 轮跑完（~205 sim-s ≈ 455-485 s 墙钟，48 轮观察窗内）
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
DIAG=/tmp/v2x_parking_in/_diag183
PX=12.00
PY=-13.95
PYAW=1.5708
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 0) 前置：谁还在发 /car_3/cmd_vel + car_3 是不是已经停住 ==="
echo "  shuttle 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')（要求 1，第 4 步换新）"
echo "  manager/recorder 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')/$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')（要求各 1，本脚本不动它们）"
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node')（car_3 是 drive:false，它不该碰 car_3）"
echo "  /car_3/cmd_vel 发布者（DDS 计数有 ~21 s 滞后，只作参考不作闸）：$(timeout -k 2 12 ros2 topic info /car_3/cmd_vel 2>&1 | tr -d '\r' | grep -a 'Publisher count')"
BEFORE=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag183$' | sort)
echo "  既有入口抓拍文件数：$(printf '%s\n' "$BEFORE" | grep -c .)"
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
echo "=== 1) 刹轮 1.5 s -> 摆到新等待位 (12.00,-13.95, yaw 1.5708) -> 静默 1.0 s ==="
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
echo "$S2" | awk '{
  dx=$1-12.00; if(dx<0)dx=-dx;
  dy=$2+13.95; if(dy<0)dy=-dy;
  dz=$3-6.70;  if(dz<0)dz=-dz;
  dw=$6-1.5708; if(dw<0)dw=-dw;
  ok = (dx<=0.05 && dy<=0.05 && dz<=0.20 && dw<=0.05);
  printf "  落位复检 |dx|=%.3f |dy|=%.3f |dz|=%.3f |dyaw|=%.3f -> %s\n", dx, dy, dz, dw, ok?"PASS":"FAIL";
  exit(ok?0:1);
}'
if [ "$?" != "0" ]; then
  echo 'FAIL 摆位没落到目标容差内，停在这里（先查是不是有发布者在拽车）'
  exit 1
fi

echo
echo "=== 2) 净空复核：pk184 相位包络（新等待位）+ 邻车活位姿 ==="
python3 "$SIM/tools/pk184_clearprechk308.py" 2>&1 | sed 's/^/  /'
PKV=${PIPESTATUS[0]}
echo "  pk184 退出码 $PKV（0=全部通过）"
if [ "$PKV" != "0" ]; then
  echo 'FAIL pk184 现场判不通过 —— 停在这里，先看上面的 FAIL 行（世界/航线可能被动过）'
  exit 1
fi
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

echo "=== 5) 盯 48 轮 ≈500-650 s：位姿 + 两闸角度 + 账本行数 + 节点日志尾 ==="
bash "$SIM/tools/pk_watch308.sh" 48 car_3

echo
echo "=== 6) 收尾对账 ==="
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  通行账本：$NP0 -> $NP1 行；闸机动作：$NA0 -> $NA1 行"
echo '  ## parking_pass.jsonl 尾 4 行'
tail -4 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-220 | sed 's/^/     /'
echo '  ## 本轮新增的入口抓拍（BEFORE 差集）并拷进 _diag183'
mkdir -p "$DIAG"
AFTER=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag183$' | sort)
NEW=$(comm -13 <(printf '%s\n' "$BEFORE") <(printf '%s\n' "$AFTER"))
if [ -z "$NEW" ]; then
  echo '     !! 没有新抓拍 —— 直接判 FAIL：查 manager 到场与插件 armed 状态'
else
  printf '%s\n' "$NEW" | sed 's/^/     /'
  while IFS= read -r f; do
    [ -n "$f" ] && cp "/tmp/v2x_parking_in/$f" "$DIAG/" 2>/dev/null
  done <<< "$NEW"
  echo "     -> $(printf '%s\n' "$NEW" | grep -c .) 个文件已拷入 $DIAG"
fi
echo "  ## 关键词计数（本轮日志=重启后新文件）：轮跑完 $(grep -ac '轮跑完' /tmp/shuttle_car_3.log) / 本轮作废 $(grep -ac '本轮作废' /tmp/shuttle_car_3.log) / 停驶挂起 $(grep -ac '停驶挂起' /tmp/shuttle_car_3.log) / 没转过 $(grep -ac '没转过' /tmp/shuttle_car_3.log) / 顶住次数到上限 $(grep -ac '顶住次数到上限' /tmp/shuttle_car_3.log)"
echo "  ## 留痕：reset#2 误拍窗口应无新抓拍（旧几何在这里多出过 P0019）；新抓拍都应出现在核验位"
echo "  ## manager 到场/识别尾 3 行：$(grep -a '到场\|识别\|抬杆' /tmp/parking_manager_node.log | tail -3 | tr '\n' ' ' | cut -c1-260)"
echo '  ## manager 尾 6 行'
tail -6 /tmp/parking_manager_node.log | cut -c1-170 | sed 's/^/     /'
echo "  ## car_3 终态：$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo "  ## car_2 终态：$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo 'PK183DONE'
exit 0
