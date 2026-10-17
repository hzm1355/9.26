#!/bin/bash
# 任务五 条目5 视频取证：记录管理 —— 进出场时间/车牌/闸机动作日志三类记录，可查询、可导出。
#   要求原文：记录车辆进出场时间、车牌、闸机动作日志，支持查询与导出。
#
#   片长 1300 s 而不是 720 s：逐段最坏耗时要点名相加 ——
#     5h 的 pk_record_audit 外面套了 timeout 420，它内部 --wait-new 300 会真等一条通行
#     （一轮进出循环实测墙钟 204 s，220 s 的旧值是卡在超时线上的）；
#     5i 的 pk_wait_pass 300 再等一条；
#     5c 的过滤矩阵 12 次 CLI 调用，每次都要走一遍 DDS 发现，实测约 12 s/次 ≈ 170 s；
#     其余段落（0a/0b/0c/5a/5b/5d/5e/5f/5g）加起来约 250 s。合计约 1190 s。
#   片长必须盖过内容，否则Recorder 到点收工，后面的段落只留在字幕里、不在片里。
#   两段等待都以「账本真长出一条新通行」为条件，所以新增前置闸 0c：先看摆渡车在不在跑。
#   它若处于停驶挂起（原地转烧满预算后 ERROR 停驶），JSONL 就不涨，那两段会各白等到超时
#   —— 那不是记录管理失效，是没有任何一次通行可记。挂起要在录制之前处置，别靠这段视频演。
#
# 这一段的判据全部走"节点报的数 vs 我从 JSONL 重算的数"，因为记录器有三份账，
# 而且它们**本来就允许不等**（memory「统计数字要点名口径并对账闭合」）：
#   · JSONL  = 追加式真相，跨进程重启不丢，是 CSV 的上游；
#   · 内存环形表 = ParkingQuery/ParkingExport 的唯一数据源，上限 max_records_in_memory 5000，
#     记录器一重启就清零（所以录这段之前已把整栈会话的账本轮转干净，见 0b）；
#   · CSV    = 导出时刻按当前口径投影出来的快照，only_new 时是增量。
# 只看"服务回 success=True"是最弱的证据：回包里没有计数就没有对账余地，所以每一次导出
# 都把 pass_rows/action_rows、CSV 落盘行数、文件 mtime 三样一起读出来。
# 过滤矩阵必须逐项跑真服务：ParkingQuery 的 DIR_*/RESULT_* 常量比 ParkingPass 大 1
# （ParkingPass 侧是 DIR_IN=0 / RESULT_ALLOWED=0，srv 侧是 1/2/3），映射写错不会报错，
# 只会把"只查放行"筛成一堆拒绝、回包照样 success=True（audit C 项专咬这个）。
# 5c 的每个过滤值都先由 pk_pass_census.py 从 JSONL 独立重算命中数，两个口径同框对照。
# 过滤用的车辆按当前在环真值取：car_3=京B88990（白名单月租、摆渡车本体）、
# car_2=粤Z1A2B3（名单外，unknown_as_temp=true 时入口发临时卡）。car_6 已退役 ——
# 上一版矩阵里的 car_6 口径全是陈旧值，跑出来一片 0 还看不出为什么。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 中途退出会把字幕停在半路，整段作废
SIM=$SIM_ROOT     # vid_common.sh 只定义 SIM_ROOT，$SIM 不补会展开成 /src/...
OUT="$VID_ROOT/任务五_停车场闸机/条目5_记录管理_进出场车牌动作日志查询导出.mp4"
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml
REC=/tmp/v2x_parking
# 摆渡车日志按模型名分文件（start_parking_nodes.sh 用 __node:=shuttle_$M 重映射节点名），
# 上一版这里 grep 的 /tmp/parking_shuttle_node.log 从来不存在，那条 capq 只会静默出空行。
SHUTLOG=/tmp/shuttle_car_3.log
CENSUS="python3 $TOOLS/pk_pass_census.py"

vid_start "$OUT" "${VID_DUR:-1300}" "$VIEW_PK,$VIEW_PIN" parking \
  "任务五 条目5  记录管理：进出场时间/车牌/闸机动作日志，查询与导出对账"
cap "上格：intersection_cam 裁出的停车场实况；下格：入口卡口相机最新一张实拍抓拍"
sleep 6

cap "== 0a 记录器必须在环且只有 1 个实例（双实例会让同一条 ParkingPass 落两行，'命中 N 条'直接翻倍）=="
capq "for N in parking_manager_node parking_recorder_node parking_shuttle_node; do printf '%-24s 实例 %s\n' \"\$N\" \"\$(pgrep -cf lib/v2x[_]demo/\$N)\"; done"
capq "timeout -k 3 15 ros2 topic info /parking/pass | sed -n '1,4p'"
cap "== 0b 本轮会话的账本起点（三段口径点名：JSONL 行数 / 记录器内存 / 会话启动时刻）=="
capq "echo 'JSONL：'; wc -l $PKJ $PKA"
capq "ls -l --time-style=+%H:%M:%S $REC/*.jsonl | awk '{print \$6, \$7}'"
cap "   记录器没有「就绪」这两个字：它的开机行是「[park-rec] 记录目录 …｜内存上限 …｜服务 …」，"
cap "   上一版这里 grep '记录器就绪' 是猜出来的过滤式，只会静默出空行。按真文案 grep："
capq "grep -a '记录目录\|已轮转为' /tmp/parking_recorder_node.log | head -3 | cut -c1-175"
cap "== 0b2 离线重算器视角的同一份账（含编号两套口径：seq 是记录器进程内流水，"
cap "     重启归 1；pass_id 是 manager 生成的全局回指键，跨重启从遗留 JSONL 尾部续号。"
cap "     两者不等长就当场证明了「当前账本只是历史的一段」，别把 seq=1 读成史上第一条）=="
capq "$CENSUS 2>&1"
sleep 4

cap "== 0c 录制前置闸：摆渡车必须在跑（5h 与 5i 各要等一条真通行落账，它挂起就等不到）=="
capq "tail -3 $SHUTLOG | cut -c1-185"
capq "printf '  本段日志里 停驶挂起 %s 次 / 跑完 %s 轮\\n' \"\$(grep -ac '停驶挂起' $SHUTLOG)\" \"\$(grep -ac '轮跑完' $SHUTLOG)\""
cap "   最后一条若是 ERROR 停驶挂起（原地转累计下发烧满预算却没转到位），先跑 tools/vid_parking_preflight.sh"
cap "   定性并处置完再录 —— 它第 8 项「150 s 内 JSONL 有没有新通行」就是这段的硬闸；"
cap "   这段视频的职责是演记录管理，不是演「等不到通行」。"
sleep 4

cap "== 5a 三份落盘物各自的字段本体（进出场时间/车牌/闸机动作三类要素都在字段名里）=="
capq "echo '  parking_pass.jsonl 首行原文：'; head -1 $PKJ | cut -c1-300"
sleep 3
capq "echo '  barrier_actions.jsonl 首行原文：'; head -1 $PKA | cut -c1-300"
sleep 3
capq "echo '  通行记录消息定义（导出 CSV 的列就按它排）：'; cat $SIM/src/v2x_msgs/msg/ParkingPass.msg"
sleep 4
capq "echo '  闸机动作消息定义：'; cat $SIM/src/v2x_msgs/msg/ParkingAction.msg"
cap "  ↑ 通行账里：sim_ts=进出场时刻（仿真时钟秒，与 /clock 同基准）、plate_number=识别出的车牌、"
cap "    gate_id=哪道闸、result=放行/拒绝/挂起、arrive_s/raise_s/clear_s/lower_s=四次时刻边沿"
cap "    动作账里：action_id=这一次动作的编号、gate_id=哪道闸、source=自动还是人工、"
cap "    reached=杆到位没有（超时未到位=false，配合 fault 说明为什么）"
cap "    注意 seq 这一列不在消息里，它是记录器给每行追加的进程内流水，重启归 1 ——"
cap "    回指某一次动作要用 action_id，不能用 seq。"
sleep 4

cap "== 5b 查询接口本体：ParkingQuery.srv 的入参就是全部可过滤维度 =="
capq "sed -n '1,25p' $SIM/src/v2x_msgs/srv/ParkingQuery.srv"
sleep 5

cap "== 5c 过滤矩阵：每个维度各筛一次，命中的数当场与 JSONL 重算值对（不写进字幕的数不算证据）=="
cap "   离线侧先把 12 个口径的重算值摆出来，下面 CLI 逐项的 matched 必须与它逐个相等。"
cap "   两个口径允许有差（内存表在记录器重启时清零、超上限会挤掉最老的），但差多少必须说得出原因。"
capq "$CENSUS 2>&1 | grep -aA14 '与 5c 过滤矩阵同序'"
sleep 3
for SPEC in \
  "全量：query --limit 500" \
  "只查进场：query --dir in --limit 500" \
  "只查出场：query --dir out --limit 500" \
  "只查放行：query --result allowed --limit 500" \
  "只查拒绝：query --result denied --limit 500" \
  "只查挂起：query --result pending --limit 500" \
  "车牌子串：query --plate 京B --limit 500" \
  "入口闸：query --gate gate_in --limit 500" \
  "出口闸：query --gate gate_out --limit 500" \
  "摆渡车本体：query --vehicle car_3 --limit 500" \
  "名单外临时车：query --vehicle car_2 --limit 500" \
  "组合条件：query --dir in --result allowed --plate 京B --limit 500"; do
  cap "-- ${SPEC} --"
  capq "$PKCLI ${SPEC#*：} 2>&1 | head -6"
  sleep 2
done
cap "   命中 0 的两种读法要分清：车牌子串/闸机/组合这类过滤本来就允许某维度为空（本轮账布里"
cap "   名单外的临时车没来过、拒绝与挂起归条目4 演），0 恰恰证明该维度真的参与了筛选 ——"
cap "   若所有维度都回同一个数，才说明过滤没生效。"
sleep 3
cap "== 5d 时段过滤 + 排序/limit 语义：t0/t1 用仿真秒，newest_first 决定单调方向 =="
capq "echo '  当前仿真时刻：'; timeout -k 3 12 ros2 topic echo /clock --qos-profile sensor_data --no-daemon --once 2>&1 | grep -a '^sec'"
capq "$PKCLI query --t0 0 --t1 99999 --limit 500 2>&1 | head -3"
capq "$PKCLI query --limit 3 2>&1 | head -7"
capq "$PKCLI query --limit 3 --oldest 2>&1 | head -7"
cap "  ↑ 默认倒序（最近 3 条）/ --oldest 正序（最早 3 条）；limit 只截返回，matched 仍是全量命中数"
sleep 4

cap "== 5e 全量导出：回包三条计数 + CSV 真的落盘（先记导出前的 mtime，导出后必须变）=="
capq "ls -l --time-style=+%H:%M:%S $REC/*.csv 2>/dev/null | awk '{print \$6, \$7}'; echo '（上一条为空=本轮还没导过）'"
capq "$PKCLI export 2>&1"
capq "ls -l --time-style=+%H:%M:%S $REC/parking_pass.csv $REC/barrier_actions.csv | awk '{print \$6, \$7}'"
capq "echo 'CSV 数据行数（总行数减表头）：'; for f in $REC/parking_pass.csv $REC/barrier_actions.csv; do printf '%-34s %s 行\n' \$(basename \$f) \$(( \$(wc -l < \$f) - 1 )); done"
capq "echo '  表头（带 BOM，Excel 直接双击可开）：'; head -1 $REC/parking_pass.csv | cut -c1-220"
cap "  ↑ pass_rows/action_rows 是记录器内存口径，CSV 行数必须与它相等；JSONL 是上游全量"
sleep 5

cap "== 5f 增量导出：游标已推进，only_new 应回 0 行，且不得在文件中间插第二行表头 =="
capq "$PKCLI export --only-new 2>&1 | head -4"
capq "echo \"第二行表头出现次数（必须 0，否则读回会多一条全空假记录）：\"; grep -ac '^seq,' $REC/parking_pass.csv"
capq "echo \"导出后 CSV 行数仍为：\"; echo \$(( \$(wc -l < $REC/parking_pass.csv) - 1 ))"
sleep 4

cap "== 5g 护栏：prefix 带目录分隔符必须被拒（这个 srv 的调用方是看板，接受路径=任意目录写文件）=="
capq "$PKCLI export --prefix ../../tmp/evil 2>&1 | head -3"
capq "ls /tmp/evil_parking_pass.csv 2>&1 | head -1; echo '上一条为 No such file = 确实没写出文件'"
sleep 4

cap "== 5h 在环全量对账器：A~H 八组判据一次跑完（它自己从 JSONL 重算，不信节点说的数）=="
cap "   --wait-new 300：先等账本真长出一条新通行再做增量导出，\"只追加、不重写整表\"是被演出来的。"
cap "   300 s 是按一轮进出循环的实测墙钟 204 s 往上留的余量；上一版给 220 s，"
cap "   碰上出口闸落杆慢一拍就会卡在超时线上，把\"记录在长\"演成\"对账器挂了\"。"
cap "   它若真报等不到新通行，读法是 0c 的前置闸（摆渡车没在跑）而不是记录链路坏 —— 别改判据。"
capq "timeout -k 10 420 python3 $TOOLS/pk_record_audit.py --config $YAML --prefix audit --wait-new 300 > /tmp/pk5_audit.txt 2>&1; echo 退出码=\$?; grep -ac '^  ✓' /tmp/pk5_audit.txt | sed 's/^/  通过判据 /'"
capq "grep -a '✓' /tmp/pk5_audit.txt | head -13"
capq "grep -a '✓' /tmp/pk5_audit.txt | tail -13"
capq "grep -aE '✗' /tmp/pk5_audit.txt | head -6; echo '  （上面为空 = 零条 ✗）'"
capq "grep -aA4 '=== 汇总 ===' /tmp/pk5_audit.txt | head -6"
cap "   动作账为什么要把\"到位\"分成三条来源对账（分开才数得平）："
cap "     通行闭合：一次到位配一次下发抬杆/下发落杆；"
cap "     手动到位：手动占权期间到位，配的是\"手动生效\"，不是自动下发；"
cap "     收尾补记：故障清除/交还自动之后补的那一拍，天生没有下发指令 —— 不补这一行，"
cap "     动作账就成\"抬了没落\"。三类各自去对各自的凭据（下发/手动生效/故障清除）。"
sleep 4

cap "== 5i 记录持续在长：等下一条真实通行落账（证明这不是查一份死档）=="
pk_wait_pass 300
capq "$PKCLI gate list 2>&1 | head -6"
cap "   导出这件事只有记录器的日志里有（manager 不管落盘），文案是\"全量导出 通行 N 行、动作 M 行 ->\""
capq "grep -a '全量导出\|增量导出' /tmp/parking_recorder_node.log | tail -3 | cut -c1-175"
vid_wait
echo DONE
