#!/bin/bash
# pk187：入口地感几何修复（方案甲）上线 —— 整栈重启 + 出生位反证 + 连盯 4 轮进出场。
#
# 这一发要治的是什么（机制链，不是"看起来不稳"）：
#   旧入口线圈 center (12.00,-10.75) / size [1.30,0.80] 在插件里换算成占用框
#   y -12.35..-9.15（半边长 = (车长1.60+车宽0.80)/2 + 半线圈 0.40 = 1.20）。核验位
#   -12.21 只在框内 0.14 m，而 shuttle 的到位残差本身就有 ±0.12 m（arrive_radius 0.12），
#   于是停在核验位上的车会把占用读数抖成"占用→释放"，manager 的 clearance_delay_s 0.5 s
#   一到就落杆 —— 2026-10-16 pk185 实测：第 1 轮 P0024 "开到位已 3.30 s" 就结单落杆
#   （正常轮这一数是 8.1 / 11.6 s），杆落在还没动车的车头前，shuttle 等 gate_in 抬杆
#   30 s 超时 → 倒车退回 → 再进场带 10° 偏西 → 前左角楔进 pivot_post_col（pk186 实测
#   间隙 0.000）→ 4 次 0.70 m/s 走不动 → 停驶挂起，轮跑完 0。
#   修法 = 线圈改 center (12.00,-10.90) / size [1.30,1.10]，占用框 y -12.65..-9.15：
#   核验位进框 0.44 m（> 0.12 残差，抖动不会再被读成释放），北缘 -9.15 一字不动
#   （释放线/通道带 -9.55 在框外 0.40、出场带 -8.90 的语义都不变）。出口那道刻意不扩：
#   扩了会把 reset#0 那一格 (6.80,-12.55) 吃进框内 0.10 m，多演一次防砸重抬。
#
# 为什么必须整栈重启而不是只重起 shuttle：线圈几何是 world 里 <plugin> 的参数，
# gzserver 只在启动时读 SDF。而重启会连带把车牌（gzserver 进程内 SetMaterial）和
# manager/recorder 的内存账一起清掉，所以走 restart_clean_stack.sh：它先轮转备份
# /tmp/v2x_parking（旧账留档，避免 pass_id 跨重启撞号被读成"账丢了"），再重贴五副牌。
#
# 判据（全部来自重启后的新日志，不看旧文件）：
#   [A] 出生位反证：car_3 重启后落在 (12.00,-13.95)、z≈6.70 —— 这一格和线圈在同一个
#       文件里，读到它就证明运行期载的是改后那份 world（不是 install 侧符号链接）。
#   [B] 轮跑完 ≥ 3、停驶挂起 = 0、本轮作废 = 0、"没转过" = 0。
#   [C] 每一轮 gate_in 的 "开到位已 X s" 全部 ≥ 6 s —— 这一条是本次修复的直接效果。
#       口径要点：这道数两道闸共用同一句日志，而 gate_out 正常就只有 5.0~5.5 s，必须
#       按 gate_id 分开取，否则把正常出口轮读成异常。6 s 这个界取在病灶（3.30）与
#       正常入口轮（8.10 / 11.60）之间。
#   [D] 入口抓拍差集非空（每轮至少一张），且没有出现新的楔住位姿。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
DIAG=/tmp/v2x_parking_in/_diag187
ROUNDS=${1:-110}          # 每行 10 s；110 行 ≈ 1100 s ≈ 4 轮（一轮 120 sim-s，RTF 0.45~0.63）
WX=12.00
WY=-13.95
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 1) 前置硬闸：world / yaml / 净空三关全过才允许重启 ==='
# 整栈重启是破坏性动作，摊上"world 载入即崩"就什么都没了（memory「判据脚本先用已通过
# 数据校准」「触发阈值别卡在等式上」）。三关各自的职责不重叠：
python3 "$SIM/tools/pk188_worldprechk316.py" 2>&1 | sed 's/^/  /'
[ "${PIPESTATUS[0]}" = "0" ] || { echo 'FAIL world 断言没过（线圈/出生位/c130 任一对不上），不重启'; exit 1; }
python3 "$SIM/tools/check_parking_layout.py" "$SIM/src/v2x_demo/config/parking_zones.yaml" 2>&1 | tail -6 | sed 's/^/  /'
[ "${PIPESTATUS[0]}" = "0" ] || { echo 'FAIL 布局体检没过，不重启'; exit 1; }
python3 "$SIM/tools/pk184_clearprechk308.py" 2>&1 | tail -8 | sed 's/^/  /'
[ "${PIPESTATUS[0]}" = "0" ] || { echo 'FAIL 净空预检没过，不重启'; exit 1; }

echo
echo '=== 2) 改前基线（楔住现场 + 旧 gzserver pid + 账本行数）==='
OLD_PID=$(pgrep -x gzserver | head -1)
echo "  gzserver pid=$OLD_PID 载的 world：$(tr '\0' ' ' < /proc/$OLD_PID/cmdline 2>/dev/null | cut -c1-70) cwd=$(readlink -f /proc/$OLD_PID/cwd 2>/dev/null)"
echo "  car_3 现位姿：$(timeout 8 gz model -m car_3 -p 2>&1 | tr -d '\n' | cut -c1-60)"
echo "  节点实例：manager $(pgrep -cf 'lib/v2x[_]demo/parking_manager_node') recorder $(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node') shuttle $(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node') signal_drive $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node')"
NP0=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA0=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  账本（重启会被轮转备份，这里只作对账基线）：通行 $NP0 行 / 动作 $NA0 行"
BEFORE=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag' | sort)
echo "  既有入口抓拍文件数：$(printf '%s\n' "$BEFORE" | grep -c .)"
echo "  旧 shuttle 日志关键词：轮跑完 $(grep -ac '轮跑完' /tmp/shuttle_car_3.log) / 停驶挂起 $(grep -ac '停驶挂起' /tmp/shuttle_car_3.log) / 作废 $(grep -ac '本轮作废' /tmp/shuttle_car_3.log)"

echo
echo '=== 3) 整栈重启（约 5 min，含五车重 spawn + 重贴牌 + 停车场三节点，SHUTTLE_MODELS 默认 car_3）==='
bash "$SIM/tools/restart_clean_stack.sh" yes 2>&1 | tee /tmp/pk187_restart.log | tail -22
NEW_PID=$(pgrep -x gzserver | head -1)
if [ -z "$NEW_PID" ]; then echo '!! 重启后 gzserver 不在，别往下盯，查 /tmp/pk187_restart.log 与 gzserver stderr'; exit 1; fi
[ "$NEW_PID" != "$OLD_PID" ] || echo "  !! 警告：gzserver pid 没变（$NEW_PID），说明没真重启"

echo
echo '=== 4) 生效闸：car_3 必须落在新出生位 (12.00,-13.95) z≈6.70 ==='
# 世界加载要 30~90 s，读不到位姿就等，最多 6 次；拿到后按容差判 —— 这一格是 world 里
# 和线圈同一次改的字节，落到它就等于"载的是改后那份文件"。
for i in 1 2 3 4 5 6; do
  P=$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n')
  echo "$P" | grep -aq 'Unable' || break
  echo "  第 $i 次读不到位姿（世界还在加载），等 20 s"; sleep 20
done
echo "  car_3：$P"
echo "$P" | awk -v wx=$WX -v wy=$WY '{
  dx=$1-wx; if(dx<0)dx=-dx; dy=$2-wy; if(dy<0)dy=-dy; dz=$3-6.70; if(dz<0)dz=-dz;
  ok=(dx<=0.15 && dy<=0.15 && dz<=0.25);
  printf "  出生位复核 |dx|=%.3f |dy|=%.3f dz=%.3f -> %s\n", dx, dy, dz, ok?"PASS（载的是改后 world）":"FAIL";
  exit(ok?0:1);
}'
[ "$?" = "0" ] || { echo 'FAIL 出生位不对 —— 载的可能不是这份 world，停在这里别盯'; exit 1; }
echo "  节点实例：manager $(pgrep -cf 'lib/v2x[_]demo/parking_manager_node') recorder $(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node') shuttle $(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node') signal_drive $(pgrep -cf 'lib/v2x[_]demo/signal_drive_node')（要求各 1/1/1/1）"
echo "  闸机回包：$(timeout 12 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 | tr -d '\r' | grep -aE '^gate_id|^angle_rad|^state|^loop_occupied' | tr '\n' ' ' | cut -c1-150)"
echo "  轮转后的新账：$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0) 行 / $(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0) 行"
echo "  备份目录：$(ls -d /tmp/v2x_parking_* 2>/dev/null | tr '\n' ' ')"

echo
echo "=== 5) 盯 $ROUNDS 拍 ≈$((ROUNDS*10)) s：位姿 + 两闸角度 + 占用 + 账本行数 ==="
bash "$SIM/tools/pk_watch308.sh" "$ROUNDS" car_3 2>&1 | tee /tmp/pk187_watch.log | tail -24

echo
echo '=== 6) 收尾对账 ==='
echo '  ## [B] 关键词计数（本轮日志=重启后新文件）'
for K in '轮跑完' '本轮作废' '停驶挂起' '没转过' '顶住次数到上限' '原地画圈' '预算'; do
  printf '    %-16s %s 行\n' "$K" "$(grep -ac "$K" /tmp/shuttle_car_3.log 2>/dev/null || echo 0)"
done
echo '  ## [C] 入口闸落杆时长：gate_in 每一拍的"开到位已 X s"都要 ≥ 6 s'
# 这一行是 manager 落杆时打的，两道闸共用同一措辞，必须按 gate_id 分开取：等值线
# 实测 gate_in 正常轮是 8.10 / 11.60 s，gate_out 正常本来就只有 5.00~5.50 s（它的
# 核验位距释放线更近），混在一起取会把出口闸读成"提前结单"。本次的病灶是 gate_in
# 出现 3.30 s —— 车头还停在核验位、占用被抖成释放，0.5 s 确认期一到就把杆落在还没
# 动车的车头上。所以 6 s 这个界是"病灶 3.30 与正常 8.1 之间"，不是拍的。
grep -a 'gate_in 下发目标角 0.00' /tmp/parking_manager_node.log 2>/dev/null \
  | grep -ao '开到位已 [0-9.]*' | awk '{print "     入口闸落杆：$2 s"}'
BAD=$(grep -a 'gate_in 下发目标角 0.00' /tmp/parking_manager_node.log 2>/dev/null \
  | grep -ao '开到位已 [0-9.]*' | awk '{if ($2+0 < 6) n++} END{print n+0}')
echo "     其中 <6 s 的：$BAD 处（要求 0；3.30 s 那种提前落杆就是本次病灶）"
echo "     等杆超时/倒车退回：$(grep -ac '抬杆 30 s 没等到\|倒车退回\|已退回 entry#0' /tmp/shuttle_car_3.log 2>/dev/null || echo 0) 行（要求 0）"
echo '  ## [D] 本轮新增入口抓拍'
mkdir -p "$DIAG"
AFTER=$(ls /tmp/v2x_parking_in/ 2>/dev/null | grep -av '^_diag' | sort)
NEW=$(comm -13 <(printf '%s\n' "$BEFORE") <(printf '%s\n' "$AFTER"))
if [ -z "$NEW" ]; then
  echo '     !! 没有新抓拍 —— 判 FAIL：查 manager 到场与插件 armed'
else
  echo "$NEW" | sed 's/^/     /'
  echo "$NEW" | while IFS= read -r f; do [ -n "$f" ] && cp "/tmp/v2x_parking_in/$f" "$DIAG/" 2>/dev/null; done
  echo "     -> $(echo "$NEW" | grep -c .) 个已拷入 $DIAG"
fi
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  ## 新账增量：通行 $NP1 行 / 闸机动作 $NA1 行（旧账在上面的备份目录里）"
tail -4 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-220 | sed 's/^/     /'
echo '  ## manager 尾 6 行'
tail -6 /tmp/parking_manager_node.log 2>/dev/null | cut -c1-170 | sed 's/^/     /'
echo '  ## shuttle 尾 6 行'
tail -6 /tmp/shuttle_car_3.log 2>/dev/null | cut -c1-170 | sed 's/^/     /'
echo "  ## car_3 终态：$(timeout 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo 'PK187DONE'
exit 0
