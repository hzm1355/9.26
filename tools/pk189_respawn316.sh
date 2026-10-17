#!/bin/bash
# pk189：把掉出沙盘的 car_3 重 spawn 回**新等待位**，然后连盯 4 轮进出场。
#
# 前因（pk187 整栈重启实测，不是推测）：
#   重启本身是成功的 —— 新 world 载进去了、五副牌贴了、闸机插件接上了（/parking/*
#   四个话题 Publisher count 都 =1）。但 car_3 落在 (15.71,-44.06) z≈-0.001，也就是
#   **掉出台面、摔在 z=0 的地面层**，离等待位 30 m。成因是我这次改出生位改出来的：
#   world 里 car_2 的出生格还停在 2026-10-14 给它当"停车场排队位"的 (12.00,-14.60)，
#   而 car_3 的等待位同批写进 (12.00,-13.95) —— 两格只差 0.65 m、同向朝北、车长 1.60 m，
#   车体在 x 上完全重叠、y 上互穿 0.95 m，加载瞬间物理引擎把两台一起弹开（car_2 也被
#   甩到 (20.81,-16.64)）。手工摆位永远测不到这一发，因为摆位时另一台早就开走了。
#   这一条现在由 tools/pk188_worldprechk316.py 第 5 段拦住（已拿本次事故校准：改前
#   判 FAIL、car_2 搬回 Ring B 的 (1.8,-5.0) 后判 OK）。
#   反过来说，事故本身就是"新 world 真被加载"的证据：旧文件里 car_3 出生在 -12.80，
#   那一格和 car_2 的 -14.60 不互穿，就不会有这一弹 —— 所以线圈 -10.90/1.10 也一定是
#   活的（同一个文件）。**不需要再重启一次。**
#
# 处置口径（memory「摆位弹射与复位风暴」「离位 >20 m 直接重 spawn 别分小跳」）：
#   30 m 的离位不能靠分小跳往回搬 —— 一路瞬移会把轮位误差越攒越多，落地就是一发弹射。
#   走 respawn_car3_263.sh 那套已验证过的流程：抽 SDF → sensor 安全闸 → /delete_entity
#   → spawn 回出生位 → 实测速度闸（给 0.10 m/s 一小脚看峰值，静止复检不算证据）→
#   刹轮精摆 → 静止复检 → 净空复核 → 重贴车牌（重 spawn 会掉回占位牌，卡口就拍空号了）。
#   之后**只重起 shuttle**，manager/recorder 不碰 —— 它们刚随整栈重启起过，内存环形
#   记录是干净的，再重起会把这条时间线抹掉。
#
# 判据同 pk187 第 6 段：轮跑完 ≥3、停驶挂起/作废/"没转过" = 0、入口闸每拍
# "开到位已" ≥6 s（3.30 s 那种提前落杆就是方案甲要治的病灶）、入口抓拍差集非空。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
PK=/tmp/v2x_parking
DIAG=/tmp/v2x_parking_in/_diag189
ROUNDS=${1:-110}
PX=12.00
PY=-13.95
PYAW=1.5708
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 0) 前置闸：world 断言（含出生格互不互穿）+ 现状基线 ==='
python3 "$SIM/tools/pk188_worldprechk316.py" 2>&1 | tail -4 | sed 's/^/  /'
[ "${PIPESTATUS[0]}" = "0" ] || { echo 'FAIL world 断言没过，先修文件（别拿旧 world 再重启一次）'; exit 1; }
echo "  gzserver pid=$(pgrep -x gzserver | head -1)（pk187 起的那个，本脚本不重启它）"
echo "  car_3 现位姿：$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-60)"
echo "  其它车：$(for M in car_1 car_2 car_4 car_5 car_6; do printf '%s ' "$(timeout 6 gz model -m $M -p 2>&1 | head -1 | tr -d '\r' | cut -c1-16)"; done)"
echo "  manager/recorder 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')/$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')（要求各 1，本脚本不动它们）"
# 目标格必须真空着 —— 这正是 pk187 翻掉的那一件事，不能在 spawn 前再判一次就太晚了。
timeout -k 5 30 python3 "$SIM/tools/nearby_live.py" car_3 3.0 2>&1 | tr -d '\r' | grep -a "car_" | head -8 | sed 's/^/  邻车 /'

echo
echo '=== 1) 停摆渡车（rclpy 吞 TERM，一律 kill -9；它还在发零速没关系，车已经挂起）==='
for PID in $(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node'); do kill -9 "$PID" 2>/dev/null; done
sleep 2
echo "  shuttle 实例 $(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')（要求 0，删车瞬间不能有别人在发 cmd_vel）"
[ "$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')" = "0" ] || { echo 'FAIL 还有 shuttle 实例没死，别删车'; exit 1; }
timeout -k 5 25 python3 "$SIM/tools/hold_zero141.py" car_3 1.5 2>&1 | tr -d '\r' | tail -1 | sed 's/^/  /'

echo
echo '=== 2) 抽 car_3 模型块成 SDF，并过"不许带 sensor"这道安全闸 ==='
awk '/<model name="car_3">/{f=1} f{print} f&&/<\/model>/{exit}' "$W" > /tmp/car3_body_189.xml
WC=$(wc -l < /tmp/car3_body_189.xml)
echo "  抽到 $WC 行（<100 行说明 world 里 car_3 定义变了，停）"
[ "$WC" -gt 100 ] || exit 1
NS=$(grep -cE '<sensor|camera|ray' /tmp/car3_body_189.xml)
echo "  sensor/camera/ray $NS 处（要求 0：挂相机的模型绝不调 /delete_entity，会把 gzserver 打掉）"
[ "$NS" = "0" ] || exit 1
{ echo '<sdf version="1.6">'; cat /tmp/car3_body_189.xml; echo '</sdf>'; } > /tmp/car3_189.sdf
grep -a "<pose>" /tmp/car3_189.sdf | head -1 | sed 's/^/  SDF 里的出生位：/;s/[[:space:]]*$//'

echo
echo '=== 3) /delete_entity car_3 → spawn 回 (12.00,-13.95) z=7.0 自由落体 ==='
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'car_3'}" 2>&1 | tail -1 | sed 's/^/  /'
sleep 2
if gz model -m car_3 -p 2>&1 | head -1 | grep -aq 'Unable'; then
  echo '  确认：world 里已无 car_3'
else
  echo '!! 删除没生效，spawn 会撞名，停在这里'; exit 1
fi
timeout -k 5 40 ros2 run gazebo_ros spawn_entity.py -entity car_3 -file /tmp/car3_189.sdf \
  -x $PX -y $PY -z 7.0 -Y $PYAW 2>&1 | tail -2 | sed 's/^/  /'
sleep 5
timeout -k 5 25 python3 "$SIM/tools/pose155.py" car_3 2>&1 | tr -d '\r' | tail -2 | sed 's/^/  落位 /'

echo
echo '=== 4) 实测速度闸：刹轮 8 s → v=0.10 给 4 s，峰值须 <= 0.40 m/s ==='
timeout -k 8 90 python3 -u "$SIM/tools/launch_guard155.py" car_3 8 0.10 4 0.40 2>&1 | tr -d '\r' | tail -3 | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "  退出码 $RC（0 通过 / 2 残余轮位误差还在放 / 3 给油不动）"
[ "$RC" = "0" ] || { echo '!! 重 spawn 后仍弹射或不动，别起节点接手一辆不动的车'; exit 1; }

echo
echo '=== 5) 刹轮 → 精摆回等待位 → 静默 → 静止复检（落位容差 0.05/0.05/0.20/0.05）==='
timeout -k 5 25 python3 "$SIM/tools/hold_zero141.py" car_3 1.5 2>&1 | tr -d '\r' | tail -1 | sed 's/^/  /'
gz model -m car_3 -x $PX -y $PY -z 6.70 -R 0 -P 0 -Y $PYAW 2>&1 | tr -d '\r' | sed 's/^/  /'
timeout -k 5 25 python3 "$SIM/tools/hold_zero141.py" car_3 1.0 2>&1 | tr -d '\r' | tail -1 | sed 's/^/  /'
D=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r')
echo "  摆后：$D"
sleep 5
E=$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r')
echo "  5 s 后：$E"
python3 "$SIM/tools/pose_delta144.py" "$D" "$E"
[ "$?" = "0" ] || { echo 'FAIL 摆完还在自走 —— 有别的东西在发指令，停'; exit 1; }
echo "$E" | awk -v wx=$PX -v wy=$PY '{
  dx=$1-wx; if(dx<0)dx=-dx; dy=$2-wy; if(dy<0)dy=-dy; dz=$3-6.70; if(dz<0)dz=-dz; dw=$6-1.5708; if(dw<0)dw=-dw;
  ok=(dx<=0.05 && dy<=0.05 && dz<=0.20 && dw<=0.05);
  printf "  落位复检 |dx|=%.3f |dy|=%.3f |dz|=%.3f |dyaw|=%.3f -> %s\n", dx, dy, dz, dw, ok?"PASS":"FAIL";
  exit(ok?0:1);}'
[ "$?" = "0" ] || { echo 'FAIL 没落到等待位容差内，停（别带着偏移起节点）'; exit 1; }

echo
echo '=== 6) 净空复核 + 重贴默认车牌（重 spawn 掉回占位牌，不贴就只能走临时车分支）==='
timeout -k 5 30 python3 "$SIM/tools/nearby_live.py" car_3 3.0 2>&1 | tr -d '\r' | sed 's/^/  /'
bash "$SIM/tools/apply_default_plates.sh" 2>&1 | tail -4 | sed 's/^/  /'
echo "  贴牌退出码 ${PIPESTATUS[0]}（0=全部到位）"

echo
echo '=== 7) 只重起摆渡车（manager/recorder 的内存记录要保持连续）==='
# [D] 的"本轮新增抓拍"按**墙钟起点 + mtime** 判，不按文件名差集：抓拍名是
# `<gate>-<序号>`、每 gzserver 进程从 1 重数，而 /tmp/v2x_parking_in 不在
# restart_clean_stack.sh 的轮转清单里 —— 于是重启后新拍的 gate_in-1.png 与上一轮的
# 同名，差集恒为空，把"拍到了"判成"零抓拍"（pk189 实跑踩过这一发）。
T0=$(date +%s)
SHUTTLE_MODELS=car_3 bash "$SIM/tools/start_parking_nodes.sh" parking_shuttle_node.py 2>&1 | tail -12 | sed 's/^/  /'
echo "  就绪行：$(grep -a '就绪' /tmp/shuttle_car_3.log | tail -1 | cut -c1-150)"

echo
echo "=== 8) 盯 $ROUNDS 拍 ≈$((ROUNDS*10)) s ==="
bash "$SIM/tools/pk_watch308.sh" "$ROUNDS" car_3 2>&1 | tee /tmp/pk189_watch.log | tail -20

echo
echo '=== 9) 收尾对账 ==='
echo '  ## [B] 关键词计数（本轮日志=重起后新文件）'
# 用 `; true` 而不是 `|| echo 0`：grep -ac 计数为 0 时本来就会打印 "0" 再以非零退出，
# 后面那支 echo 0 会把第二个 0 拼进变量，printf 一展开就断成两行，事后判读容易看错。
for K in '轮跑完' '本轮作废' '停驶挂起' '没转过' '顶住次数到上限' '原地画圈' '偏离本段车道'; do
  printf '    %-16s %s 行\n' "$K" "$(grep -ac "$K" /tmp/shuttle_car_3.log 2>/dev/null; true)"
done
echo '  ## [C] 入口闸落杆时长（gate_in 单独取；gate_out 正常就只有 5.0~5.5 s，混取会把正常读成异常）'
# 注意 awk 的字符串拼接：`"...：$2 s"` 里的 $2 在引号内是**字面量**，会原样打印 "$2 s"。
# 要把字段拼进去就得闭合引号写 "…：" $2 " s"。
grep -a 'gate_in 下发目标角 0.00' /tmp/parking_manager_node.log 2>/dev/null \
  | grep -ao '开到位已 [0-9.]*' | awk '{print "     入口闸落杆：" $2 " s"}'
BAD=$(grep -a 'gate_in 下发目标角 0.00' /tmp/parking_manager_node.log 2>/dev/null \
  | grep -ao '开到位已 [0-9.]*' | awk '{if ($2+0 < 6) n++} END{print n+0}')
echo "     其中 <6 s 的：$BAD 处（要求 0；3.30 s 那种提前落杆就是方案甲的病灶）"
echo "     等杆超时/退回：$(grep -ac '抬杆 30 s 没等到\|已退回 entry#0' /tmp/shuttle_car_3.log 2>/dev/null; true) 行（要求 0）"
echo '  ## [D] 本轮新增入口抓拍（按 mtime > T0，T0='"$T0"'）'
mkdir -p "$DIAG"
NEW=$(find /tmp/v2x_parking_in -maxdepth 1 -name '*.png' -newermt "@$T0" -printf '%f\n' 2>/dev/null | sort)
if [ -z "$NEW" ]; then
  echo '     !! 没有新抓拍 —— 判 FAIL'
else
  echo "$NEW" | sed 's/^/     /'
  echo "$NEW" | while IFS= read -r f; do [ -n "$f" ] && cp "/tmp/v2x_parking_in/$f" "$DIAG/" 2>/dev/null; done
  echo "     -> $(echo "$NEW" | grep -c .) 个已拷入 $DIAG"
fi
echo "  ## 新账：通行 $(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0) 行 / 动作 $(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0) 行"
tail -4 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-220 | sed 's/^/     /'
echo '  ## shuttle 尾 8 行'
tail -8 /tmp/shuttle_car_3.log 2>/dev/null | cut -c1-175 | sed 's/^/     /'
echo "  ## car_3 终态：$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\r' | cut -c1-62)"
echo 'PK189DONE'
exit 0
