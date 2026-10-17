#!/bin/bash
# 任务五 条目4 视频取证：闸机管控逻辑仿真。
#   要求原文：白名单车辆自动放行、临时车辆登记后放行、异常车辆拒绝入内并告警；
#             支持手动抬落杆与故障注入。
#
# 这一段的两条硬规矩（都是前面几轮踩出来的）：
#  1) 三态（放行/临时/拒绝）全部由**真实通行**产生，判据只看记录器落的那两份 JSONL 在不在长，
#     不看话题 —— DDS 发现竞态会把 --once 读成空包，读成"没有通行"是假证据。
#  2) 白名单热改走 srv（不重启，演的是"运行期可改"）；黑名单与 unknown_as_temp 是 yaml 基线，
#     只能改文件 + 单独重启 manager 才生效。重启 manager 不碰 recorder，内存里的环形记录
#     一清零，条目 5 的"内存留存 N 条"就断档了，所以这一段只在最后重启一次 shuttle。
#  3) 手动/故障演示期间把摆渡车摘掉：它是自动流程的唯一驱动，留在环里会和手动指令抢杆，
#     读回的闸态就成了"不知道谁下的令"。它是 cmd_vel 发布者，杀掉必须补发零速。
#
# ── 2026-10-14 11:31 的事故与由此定下的三条规矩（这一版全部改到位）────────────
# 上一版 3a/3d 是 `cp -f $YAML /tmp/…vid4bak` … `cp -f /tmp/…vid4bak $YAML && rm -f 备份`。
# 那次条目 4 在 2b 挂死中止、根本没跑到 3a，所以 /tmp 里没有备份；重录跑到 3d 时
# `cp -f` 从一个不存在的源把**唯一真值文件截成 12 字节**，紧接着的 `rm -f` 把唯一能追溯
# 的备份也删了，manager 当场加载 yaml 抛 TypeError 退出。三条规矩：
#   甲、开录前先体检配置（0 段）：解析不过、或字节数不像话，就不开录。
#   乙、备份要"写完立刻回读校验"，且用带时刻的文件名、留在仓库外的 /tmp 里也留一份，
#       回灌前必须确认源存在且字节数与备份一致 —— 不满足就停，绝不"顺手 rm"。
#   丙、每一次"改身份"的动作都锚在**出场落账之后**（pk_wait_pass169.sh 按方向等，不按行数等）。
#       上一版 2b 是"等任意一条新通行"，那条多半是进场，紧接着摘掉白名单 → 车在场内遇到的
#       下一个闸是出口，而出口没有"未登记按临时车"分支（parking_manager.on_result 里
#       unknown_as_temp 只在 direction=='in' 生效）→ 出口判拒 → 车等 30 s 退回 → 连废三轮
#       触发 #83 停驶挂起，第 4/5 段的手动与故障演出就此没有驱动。
#
# ── 2026-10-16 三修：为什么三态改成"同一辆车改身份"，而不是换一辆车 ──────────────
# 上一版 2b/2c/3a 打的牌是 沪A12345，那是 car_6 的牌。car_6 早在 2026-10-14 就退役了
# （模型仍在 world 里、出生位挪到 (23.50,-23.50)，但**不给它起节点**），摘它的白名单
# 不会让任何一辆在环的车改变结论 —— 那一支演出来是"面板上名单少了一条、场上毫无反应"，
# 属于典型的"改了个没人读的值"。现在按用户指令"只用 car_3 在停车场里、car_2 回归正常
# 行驶赛道"，在环的摆渡车只有 car_3（京B88990，月租白名单）。所以：
#   · 白名单放行 = car_3 命中京B88990（自动流程本来就在演这一支，2a 只是把它读出来）；
#   · 临时车登记放行 = **srv 热摘**京B88990（不重启，顺带演"运行期可改"）→ 未登记且
#     unknown_as_temp=true → 入口发临时卡放行，出场凭在途卡放行；演完 srv 加回去；
#   · 异常拒绝入内 = yaml 基线改 blacklist:[京B88990] + unknown_as_temp=false，单独重启
#     manager 生效（黑名单优先级高于白名单，两条都留着才是真的"拒"）。
# 另有一条备选路没走：parking_zones.yaml 的 fleets.car_2 那支航线（粤Z1A2B3 本来就不在
# 白名单，临时车分支天生成立）仍在文件里，要用它就得停 signal_drive_node、把 car_2 置回
# drive:false、SHUTTLE_MODELS=car_2 重起 shuttle，演完再归队 —— 那会打断任务二/三/四的
# 车队取证窗口，属于"可演但不该在这一版演"。体检器现在会盯住 car_2 那支航线没被删。
# 还有一处必须改的字面：摆渡车日志名。start_parking_nodes.sh 按模型名分文件写
# /tmp/shuttle_car_3.log（两个实例的 ROS 节点名靠 __node:=shuttle_<模型> 分开），
# 旧版 grep 的 /tmp/parking_shuttle_node.log 根本不存在，那句"3c 摆渡车会退回"就永远
# 是一片空白 —— 空白读起来像"没发生"，其实是"没查到"。
#
# 依赖：pk_wait_pass169.sh / pk_yaml_check165.py / pk_action_stat.py / pk_safelower186.sh
# 这四个 2026-10-16 随任务五代码从 存档_弃用任务五六_1016 一起恢复回 tools/
# （restore_parking_308.py 当时只列了 19 个脚本，没带上它们，所以前一版条目 4 是跑不起来的）。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
SIM=$SIM_ROOT     # vid_common.sh 只定义了 SIM_ROOT，$SIM 不补会展开成 /src/...
OUT="$VID_ROOT/任务五_停车场闸机/条目4_管控逻辑_三类放行拒绝与手动故障.mp4"
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml
MGRLOG=/tmp/parking_manager_node.log
# 按模型名分文件（start_parking_nodes.sh 里 > "/tmp/shuttle_$M.log"），不是节点名。
SHUTLOG=/tmp/shuttle_car_3.log
PLATE=${PLATE:-京B88990}     # 在环那辆（car_3）的车牌；改身份三段都打在它身上
# 片长 1700 s（上一版 1400 不够）：这一版有**三**段要等一条真通行落账（2a 等出场、
# 2b 等进场、3b 等进场的 DENIED），而一轮周期实测 204 s 墙钟（RTF≈0.76，进场→出场→
# 复位≈98 s 仿真 + loop_period 120 s 仿真）。三段各自最坏 320 s，加上自测段 150 s 和
# 手动/故障两段 ≈190 s，内容最坏会顶到 ~2100 s。1700 是"典型路径 + 一段超时"的口径；
# 真遇到两段超时就用 VID_DUR=2200 重录 —— 片长必须盖过内容，否则 Recorder 到点收工，
# 后面的段落只留在字幕里、不在片里。
DUR=${VID_DUR:-1700}
# 备份名带时刻：同一分钟里重跑两次也不会互相覆盖，更不会让 3d 去回灌一个"上一轮已删"的路径
BAK=/tmp/parking_zones.yaml.vid4.$(date +%H%M%S).bak

vid_start "$OUT" "$DUR" "$VIEW_PK,$VIEW_PIN" parking \
  "任务五 条目4  管控逻辑：白名单放行 / 临时车登记放行 / 异常拒绝+告警 / 手动抬落杆 / 故障注入"

cap "== 0 开录前体检：配置解析得开、关键字段是运行期实测那一套；三节点各 1 实例、四个服务都在 =="
# 字节数只报数、不当判据（判据在 pk_yaml_check165.py 里，它按在环实测值逐项比）。
# 这份真值现在 550 行 / 40184 字节；10-14 那次截断事故的形态是 12 字节，所以"报出来
# 像不像话"这一眼必须有，但它不能替代体检器。
capq "wc -c $YAML | awk '{print \"  yaml 字节 \" \$1 \"（当前真值 550 行 / 40.2 KB）\"}'"
capq "python3 $TOOLS/pk_yaml_check165.py $YAML 2>&1 | tail -4"
capq "for N in parking_manager_node parking_recorder_node parking_shuttle_node; do printf '%-24s 实例 %s\n' \"\$N\" \"\$(pgrep -cf lib/v2x[_]demo/\$N)\"; done"
capq "ros2 service list 2>/dev/null | grep -aE '^/parking/(gate|whitelist|query|export)$'"
cap "== 0b 管控基线（名单/临时车开关/识别超时/故障类型都读自同一份 yaml）=="
capq "grep -aE '管控就绪' $MGRLOG | tail -1 | cut -c1-190"
capq "grep -nE 'blacklist:|unknown_as_temp:' $YAML"
cap "== 0c 在环的到底是哪一辆摆渡车（三态都打在它身上，认错一辆整段就是空转）=="
# 只看进程不够：两辆摆渡车是同一份脚本的两个实例，必须读命令行里的 -p model:=。
capq "for P in \$(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node'); do tr '\\0' ' ' < /proc/\$P/cmdline | grep -oE 'model:=[a-z0-9_]+' | sed 's/^/  pid '\$P' 驱动 /'; done"
capq "ls -l --time-style=+%H:%M:%S $SHUTLOG | awk '{print \"  摆渡车日志（字幕要 grep 的就是这一个）\", \$6, \$7}'"
capq "grep -aE '核验通过' $MGRLOG | tail -2 | cut -c1-150"
capq "timeout -k 3 12 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null | grep -m1 '^data:' | cut -c1-170"
sleep 4

cap "== 1 合成数据自测先把 srv 全 op 与三个拒绝分支一次跑完（真实动力学时序回放）=="
capq "timeout -k 10 150 python3 $SIM/src/v2x_demo/v2x_demo/parking_manager_selftest.py $YAML 2>&1 | tail -12"
sleep 4
cap "  ↑ D 黑名单优先于白名单、F 未登记且 unknown_as_temp=false 直接拒入、C 识别超时按 PENDING 收口不猜牌"

cap "== 2a 白名单车自动放行：等这一轮的【出场】落账（同一轮的进场已经在账上，两条一起读）=="
cap "  ↑ 为什么锚在出场：上场车一轮是 进场→出场→复位，两个闸各判一次。把后面\"改身份\"的"
cap "    动作对齐到出场之后，下一个闸事件才是进场；对齐到进场之后就会先撞上出口闸，而出口"
cap "    没有临时车分支（manager.on_result 的 unknown_as_temp 只在 direction=='in' 生效，"
cap "    出场走的是'名单外且无在途卡 → 拒绝出场'那一支）（见 丙 段注释）"
capq "$PKCLI wl list"
cap "  ↑ 起点状态：在环那辆 $PLATE（car_3，月租）此刻必须在名单里，2a 演的才是'命中白名单'"
capq "bash $TOOLS/pk_wait_pass169.sh out 320"
capq "$PKCLI query --dir in --result allowed --limit 1"
capq "$PKCLI query --dir out --result allowed --limit 1"
capq "grep -a '核验通过' $MGRLOG | tail -2 | cut -c1-150"
capq "python3 $TOOLS/pk_action_stat.py --gate gate_in --tail 6"
capq "python3 $TOOLS/pk_action_stat.py --gate gate_out --tail 6"
sleep 6
cap "  ↑ 两道闸都是识别命中白名单后自动抬的杆，全程没人点：进场=入口闸、出场=出口闸"
cap "    落杆时长按 gate_id 分开读（上面两行就是分开的）：入口闸正常 7.7~8.6 s、出口闸"
cap "    4.9~5.0 s 本来就不同 —— 混取会把出口闸的正常拍读成'提前落杆'异常。"
cap "    方案甲（线圈 -10.90 / y 向 1.10）治的就是入口闸 3.30 s 提前落杆那一发"

cap "== 2b 临时车登记后放行：就在这一刻热摘在环那辆的月租身份（srv 改名单，不重启），等下一轮进场 =="
capq "$PKCLI wl remove --plate $PLATE"
capq "$PKCLI wl list"
cap "  ↑ 摘完必须当场读回：wl list 里没有 $PLATE 才算 srv 真的动了内存里的名单。"
cap "    注意别看 '管控就绪' 那行 —— 它是进程启动时打的一次性快照，热改之后不会重播，"
cap "    拿它当回读会把'已摘掉'读成'还在名单里'（生效要用效果复核，不是看 echo）"
capq "bash $TOOLS/pk_wait_pass169.sh in 320"
capq "$PKCLI query --dir in --limit 2"
capq "grep -aE '发临时卡|未登记车牌' $MGRLOG | tail -3 | cut -c1-190"
sleep 6
cap "  ↑ 名单外 + unknown_as_temp=true → 入口自动发临时卡放行，记录里带卡号（不是直接放行）"
cap "    这一支与 2a 那一支的区别就在 kind/ticket_id 两列：月租是 kind=月租、无卡号；"
cap "    临时是 kind=临时、ticket_id=T-<毫秒>，而且 whitelist_hit=未命中"
cap "== 2c 出场凭在途临时卡放行、落杆即核销；然后把名单恢复基线 =="
capq "bash $TOOLS/pk_wait_pass169.sh out 320"
capq "$PKCLI query --dir out --limit 2"
capq "grep -aE '发临时卡|核销|在途' $MGRLOG | tail -4 | cut -c1-190"
capq "$PKCLI wl add --plate $PLATE --kind monthly --label 月租车"
capq "$PKCLI wl list"
sleep 4
cap "  ↑ 恢复也必须读回：wl list 里 $PLATE 重新出现，3a 的黑名单才是'白名单+黑名单同时命中'"

cap "== 3a 异常车辆拒绝入内并告警：黑名单是 yaml 基线，改文件后单独重启 manager =="
capq "cp -f $YAML $BAK && wc -c $BAK | awk '{print \"  备份 -> \" \$2 \" 字节 \" \$1}'"
# 备份写完立刻回读校验：上一版就是没做这一步，才让 3d 去回灌一个不存在的路径。
# 下限 35000（真值 40184 字节）：那次事故的形态是"截成 12 字节"，任何低于下限的读回
# 都说明备份本身不可信，宁可不演拒绝，也不能把唯一真值置于不可恢复的境地。
BAK_BYTES=$(wc -c < $BAK 2>/dev/null || echo 0)
if [ "$BAK_BYTES" -lt 35000 ]; then
  cap "  !! 备份只有 $BAK_BYTES 字节，改文件这一步跳过（宁可不演拒绝，也不能把真值文件置于不可恢复的境地）"
  SKIP_BL=1
else
  SKIP_BL=0
  # 打在 $PLATE 上：黑名单优先级高于白名单（manager.on_result 先查 blacklist），
  # 所以 2c 刚把它加回白名单反而是这一支的前提 —— "既在名单内又在名单外"才是真实的
  # 异常车场景，也顺带证明了 D 分支"黑名单优先于白名单"不只是自测里的一条合成用例。
  sed -i "s/^    blacklist: \[\].*\$/    blacklist: [$PLATE]   # vid4 临时：演拒绝入内+告警/" "$YAML"
  sed -i 's/^    unknown_as_temp: true/    unknown_as_temp: false/' "$YAML"
  capq "grep -nE 'blacklist:|unknown_as_temp:' $YAML"
  cap "  ↑ 重启时机动过：这一刻是 2c 的出场刚落账，车在复位段（场外），不依赖任何一道闸。"
  cap "    manager 重启会丢掉它内存里那条在途 cycle —— 挑复位段重启就丢不到东西；"
  cap "    recorder 不碰，条目 5 的内存环形记录才不断档。"
  capq "bash $SIM/tools/start_parking_nodes.sh parking_manager_node.py 2>&1 | grep -aE '实例|OK：|!!|管控就绪' | tail -6"
  sleep 6
  capq "grep -a '管控就绪' $MGRLOG | tail -1 | cut -c1-190"
  cap "  ↑ '管控就绪' 这行是重启后才打的，所以它是**新基线已加载**的凭据：黑名单 1 条、"
  cap "    未登记按临时车=False，与上面 grep 到的 yaml 两行一一对上才算生效"
  cap "== 3b 等这一轮的【进场】拒绝结论落账（DENIED 当场写，不等落杆）=="
  capq "bash $TOOLS/pk_wait_pass169.sh in 320"
  capq "$PKCLI query --result denied --limit 3"
  capq "grep -a '拒绝放行' $MGRLOG | tail -3 | cut -c1-190"
  cap "  ↑ warn 级日志就是运维侧的告警出口；同时 /parking/pass 上这条 result=DENIED"
  cap "    reject() 会当场 emit_pass 并把四个时刻写成 -1（不抬杆就没有抬到位），"
  cap "    动作日志那边配一条 action_name=拒绝动作 的 ACT_REFUSE 行"
  capq "python3 $TOOLS/pk_action_stat.py --gate gate_in --tail 6"
  sleep 6
  cap "== 3c 摆渡车在核验位等不到抬杆会退回，本轮作废 —— 拒绝本来就该过不去 =="
  capq "grep -aE '等杆|退回|本轮作废|停驶挂起' $SHUTLOG | tail -4 | cut -c1-190"
  capq "timeout -k 3 12 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null | grep -m1 '^data:' | cut -c1-170"
  cap "  ↑ 连废三轮会触发 #83 的停驶挂起（顶住次数封顶），所以 3d 必须连同 shuttle 一起重启，"
  cap "    否则第 4/5 段的手动与故障演出没有驱动。"
fi
cap "== 3d 恢复基线（回灌前先校验备份存在且字节数对得上）并重启 manager+shuttle =="
if [ "$SKIP_BL" = "1" ]; then
  cap "  上一段没改文件，这里只把 manager 与 shuttle 交还自动流程"
else
  RB=$(wc -c < $BAK 2>/dev/null || echo 0)
  if [ "$RB" -lt 35000 ]; then
    cap "  !! 备份不可信（$RB 字节），停止回灌，人工介入"
  else
    cp -f "$BAK" "$YAML"
    capq "wc -c $YAML | awk '{print \"  回灌后 yaml 字节 \" \$1}'"
    capq "python3 $TOOLS/pk_yaml_check165.py $YAML 2>&1 | tail -2"
    capq "grep -nE 'blacklist:|unknown_as_temp:' $YAML"
    # 校验通过才清备份；不清的话留着也无害（带时刻的名字不会撞下一轮）
    rm -f "$BAK"
  fi
  # 拒绝这一支必然让摆渡车连废三轮 → #83 会挂停，所以这里连同 shuttle 一起重启，
  # 否则第 4/5 段的手动与故障演出没有驱动，收尾也交不回自动状态机。
  capq "SHUTTLE_MODELS=car_3 bash $SIM/tools/start_parking_nodes.sh parking_manager_node.py parking_shuttle_node.py 2>&1 | grep -aE '实例|OK：|!!|管控就绪' | tail -6"
  sleep 4
  capq "$PKCLI wl list"
fi

cap "== 4a 手动抬落杆（服务通道）：先摘摆渡车，免得自动流程与手动指令抢杆 =="
# 摆渡车是这一段唯一的自动驱动，也是 /car_3/cmd_vel 的发布者：rclpy 接管 SIGTERM，
# 主线程不在 spin 里的常驻节点 TERM 杀不掉，一律 kill -9；杀完必须补发零速，否则它
# 最后一拍的速度会一直作用到差速轮默认的 1 s 超时之后（memory「杀 cmd_vel 发布者
# 后必须补发零速」）。只碰 car_3 —— car_2 现在归 signal_drive_node 的车流在管，
# 往它的 cmd_vel 上插一发零速会把车流里那辆车莫名刹住。
for PID in $(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node' 2>/dev/null); do
  kill -9 "$PID" 2>/dev/null || true
done
sleep 2
capq "timeout -k 5 25 python3 $TOOLS/hold_zero141.py car_3 1.5 2>&1 | tr -d '\\r' | tail -1"
capq "timeout -k 8 gz model -m car_3 -p 2>&1 | head -1 | tr -d '\\r' | sed 's/^/  car_3 停在 /'"
capq "printf '摆渡车实例 %s（应为 0），上场车 car_3 已补发零速并停稳\\n' \"\$(pgrep -cf lib/v2x[_]demo/parking_shuttle_node)\""
cap "== 4b 半抬 0.7854 rad：目标角≠全开角，画面里杆应停在 45° =="
capq "$PKCLI gate raise --gate gate_in --angle 0.7854"
sleep 5
capq "$PKCLI gate list"
capq "timeout -k 3 12 ros2 topic echo /parking/gate_state --no-daemon --once 2>/dev/null | grep -aE 'gate_id|state:|angle_rad|target_rad|source' | head -8"
sleep 6
capq "bash $TOOLS/pk_safelower186.sh gate_in"
cap "== 4c 手动通道（话题）：gate_id:rad，无回包，靠闸态读回自证 =="
capq "$PKCLI manual gate_in:1.5708"
sleep 7
capq "$PKCLI gate list"
capq "grep -aE '\"action_name\": *\"手动生效\"' $PKA | tail -2 | cut -c1-200"
capq "echo -n '  动作日志里 手动生效 累计 '; grep -acE '\"action_name\": *\"手动生效\"' $PKA"
capq "$PKCLI manual gate_in:AUTO"
sleep 5
capq "bash $TOOLS/pk_safelower186.sh gate_in"

cap "== 5 故障注入四种（对出口闸演，入口闸留给自动流程）=="
for K in STUCK RAISE_FAIL NO_CLOSE SENSOR_FAIL; do
  cap "-- 注入 $K --"
  capq "$PKCLI gate fault --gate gate_out --kind $K"
  sleep 5
  capq "$PKCLI gate list"
  capq "grep -a \"$K\" $MGRLOG | tail -1 | cut -c1-190"
  sleep 4
  capq "$PKCLI gate clear --gate gate_out"
  sleep 3
done
cap "  ↑ 动作日志的字段本体是中文 action_name + 整数 action（6=故障生效、7=故障清除），"
cap "    按 'ACT_FAULT' 字样去 grep 会静默得 0，那是过滤式猜错，不是没落账"
capq "echo -n '  故障生效 累计 '; grep -acE '\"action_name\": *\"故障生效\"' $PKA"
capq "echo -n '  故障清除 累计 '; grep -acE '\"action_name\": *\"故障清除\"' $PKA"
capq "grep -aE '\"action_name\": *\"(故障生效|故障清除)\"' $PKA | tail -4 | cut -c1-190"

cap "== 6 收尾：摆渡车回环 + 两道闸交还自动 =="
# 新实例第一次拿到位姿时走 seat_at()：按**最近航点**重选起点，并清空 hold_gate /
# pending_boom / back_gate / boom_stall / ref / stuck_tries / cycle_start 这些遗留判定
# —— 所以它不一定要从 entry#0 重跑，而是就地接上航线。这一条正是"摘掉再拉起"安全的依据。
capq "SHUTTLE_MODELS=car_3 bash $SIM/tools/start_parking_nodes.sh parking_shuttle_node.py 2>&1 | grep -aE '实例|OK：|!!|位姿' | tail -6"
sleep 6
capq "grep -aE '就绪|重新选路|起步' $SHUTLOG | tail -3 | cut -c1-190"
capq "timeout -k 3 12 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null | grep -m1 '^data:' | cut -c1-170"
capq "$PKCLI gate auto"
for G in gate_in gate_out; do
  capq "$PKCLI gate clear --gate $G"
  sleep 3
done
cap "== 6a 收尾效果闸：两道闸必须 CLOSED + fault=NONE + mode=auto（不满足就当场报，别把脏状态留给条目 5）=="
for K in 1 2 3 4 5 6; do
  capq "$PKCLI gate list 2>&1 | head -4"
  BAD=$(timeout -k 3 20 python3 $TOOLS/parking_cli.py gate list 2>&1 | tr -d '\r' |
        grep -cE 'fault=(STUCK|RAISE_FAIL|NO_CLOSE|SENSOR_FAIL)|mode=manual' )
  printf '  第 %s 次复检：仍非干净态的行数 %s\n' "$K" "$BAD"
  [ "${BAD:-9}" = "0" ] && break
  sleep 8
done
capq "$PKCLI gate list"
cap "== 6b 这一段产生的账（三条通行里必须含 1 条 DENIED）=="
capq "wc -l $PKJ $PKA"
capq "$PKCLI query --result denied --limit 2"
capq "$PKCLI query --dir in --limit 3"
vid_wait
# 收尾标记必须打：链式启动器按这行判"这一段录完了"。少了它（上一版就是少的），
# 启动器会一路空转到自己的超时上限，然后照样往下起条目 5 —— 那时如果已经用别的
# 方式起了 5，就会出现两个 vid_rec 同时写盘、互相把对方的字幕流清空。
echo DONE
