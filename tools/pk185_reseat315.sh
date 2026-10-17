#!/bin/bash
# pk185：出口侧转点北移（reset#0 -12.80 -> -12.55）上线 —— 摆回等待位、重贴牌、
# 只重起 shuttle，盯满 **三轮** 完整周期（旧转点在第三轮蹭 c131 猎振挂起）。
#
# 背景与依据（都是本轮实测/离线复核，不是推测）：
#   * pk183 等待位南移上线后第 1/2 轮完整（P0022 证明旧几何零抓拍判死点已治愈），
#     第 3 轮却在 reset#0 挂起：停位 (6.85,-12.83) 容差内偏南 0.03 -> 轮轴
#     (6.80,-13.23) -> 相位① 36.4° 过冲样本车尾角 y=-14.49，吃进 c131（北缘
#     -14.25）0.24 m -> 绕楔点猎振把 6.3 rad 原地转预算烧尽（watch 位姿
#     +2.46 -> -1.80 -> -2.56 rad、漂 1.6 m）-> 停驶挂起在 (7.72,-13.03)。
#   * 根因不是南移引入：旧转点 pk184 相位余量只有 +0.016 m，**小于 arrive_radius
#     0.12 的正常到位残差**，前两轮能过纯属残差恰好偏北。教训：净空余量必须盖住
#     到位残差。
#   * 修法 = exit 末点与 reset#0 北移 0.25 到 y=-12.55（yaml 已改）。pk184 场景叠
#     ±0.08 停位偏差重跑退出码 0：reset#0 相位包络最小 +0.186 m（c131）；出口链路
#     不受影响（车心 -12.55 距出口 loop 1.8 > 1.6 不占用、距 boom_col 南缘 0.65、
#     核验位/抓拍几何不动）。
#   * 摆位流程照现场三硬规矩：刹轮 1.5 s -> 摆位 -> 静默 1.0 s -> 静止复检；
#     只重起 shuttle，manager/recorder 不碰（内存环形记录是条目 5 的查询源）。
#     车当前在停驶挂起态连发零速，摆位期间它只会灌零速，不会被拽走。
#
# 预期对账（RTF≈0.45，每轮 120 sim-s ≈ 265 s 墙钟）：
#   [1] 第 1 轮进场抓拍（重启后 ~10 s 内）
#   [2] 第 1 轮跑完（~85 sim-s ≈ 190 s 墙钟）—— 含新 reset#0 北移转点
#   [3] 第 2 轮进场抓拍（~125 sim-s ≈ 280-300 s 墙钟）
#   [4] 第 3 轮跑完（~290 sim-s ≈ 650 s 墙钟）← 旧转点在这一轮猎振挂起，判死点
#   [5] 停驶挂起 = 0；本轮作废 = 0；"没转过" = 0（reset#0 顺利的旁证）
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
DIAG=/tmp/v2x_parking_in/_diag185
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
BEFORE=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag185$' | sort)
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
echo "=== 1) 刹轮 1.5 s -> 摆回等待位 (12.00,-13.95, yaw 1.5708) -> 静默 1.0 s ==="
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
echo "=== 2) 净空复核：pk184 相位包络（reset#0 北移 + 停位偏差 ±0.08）+ 邻车活位姿 ==="
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

echo "=== 5) 盯 80 拍 ≈800-950 s：位姿 + 两闸角度 + 账本行数 + 节点日志尾 ==="
bash "$SIM/tools/pk_watch308.sh" 80 car_3

echo
echo "=== 6) 收尾对账 ==="
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  通行账本：$NP0 -> $NP1 行；闸机动作：$NA0 -> $NA1 行"
echo '  ## parking_pass.jsonl 尾 6 行'
tail -6 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-220 | sed 's/^/     /'
echo '  ## 本轮新增的入口抓拍（BEFORE 差集）并拷进 _diag185'
mkdir -p "$DIAG"
AFTER=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag185$' | sort)
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
echo "  ## reset#0 旁证：日志里出现'原地画圈/超预算'即旧转点复发，计数 $(grep -ac '原地画圈\|预算' /tmp/shuttle_car_3.log)"
echo "  ## manager 尾 6 行"
tail -6 /tmp/parking_manager_node.log | cut -c1-170 | sed 's/^/     /'
echo "  ## car_3 终态：$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo "  ## car_2 终态：$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo 'PK185DONE'
exit 0
