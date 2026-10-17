#!/bin/bash
# 任务五 条目3 视频取证：闸机动力学仿真 —— 真实物理碰撞 + 升降动力学，
# 以及"核验通过→抬杆→通行→地感离场→落杆"的全自动时序。
#
# 这一段最容易出的假证据是"节点自己把 state 写成抬起、面板跟着跳数字"。所以给三条
# 互相独立的读数，任一条都能单独把那种写法排除掉：
#   · 逐拍表（pk_gate_trace.py）：目标角一跳到 90°，实测角一拍一拍往上爬，同一拍上
#     角速度与力矩都非零 —— 只有真按 revolute joint 积分才画得出这种斜坡；
#   · 动作日志时序（pk_action_stat.py）：hold 1.5 s / 单程理论 open_angle/velocity
#     ≈0.79 s / 落杆超时 5 s 到底有没有被执行，只从 JSONL 相邻两行的时间差里量；
#   · 独立 gzserver 实测（test_barrier_dyn.py）：把 5 s 超时判据、四种故障、落杆砸车
#     放到只有一道闸的世界里量，主栈的车队与四路 1080p 相机会把 RTF 抖动的污染带给
#     超时判据，独立实例没有这个问题（实测整轮 79 s，17 条断言）。
# 防砸不另外凑一次通行：后台挂"入口地感一占用就下令落杆"的陷阱，它必然落在杆正抬起、
# 车正在杆下那一瞬 —— 同一次通行里就把 refuse_lower_while_occupied=true 演出来。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 中途退出会把字幕停在半路，整段作废；宁可少一步操作
SIM=$SIM_ROOT     # vid_common.sh 只定义 SIM_ROOT，$SIM 不补会展开成 /src/...
OUT="$VID_ROOT/任务五_停车场闸机/条目3_闸机动力学_升降时序防砸与独立实测.mp4"
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml
WORLD=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world

vid_start "$OUT" 620 "$VIEW_PK,$VIEW_PIN" parking \
  "任务五 条目3  闸机动力学仿真（真实物理碰撞 + 升降动力学 + 核验→抬杆→通行→落杆 全自动）"
cap "上格：停车场高位裁切画面，盯红白闸杆——它必须是一寸一寸抬起来的，不是瞬间立正"
cap "下格：入口卡口相机实拍，此刻车在核验位"
sleep 6

cap "== 0 运行期前置：三节点在环、BarrierState 有发布者、上场摆渡车真在 world 里 =="
capq "for n in parking_manager_node parking_recorder_node parking_shuttle_node; do printf '%-24s python 实例 %s\n' \$n \$(pgrep -cf lib/v2x_demo/\$n); done"
capq "timeout -k 3 15 ros2 topic info /parking/gate_state | sed -n '1,4p'"
capq "for M in car_3 car_2; do printf '%-7s ' \$M; gz model -m \$M -p 2>&1 | head -1; done"
cap "  两份日志当前的行数（动作日志=闸机侧的账本，通行日志=记录侧的账本）："
capq "wc -l $PKJ $PKA"
sleep 6

cap "== 3a 动力学参数只有一处真值：parking_zones.yaml 的 barrier 段 =="
capq "sed -n '93,107p' $YAML"
sleep 6
capq "sed -n '108,121p' $YAML"
cap "  effort/velocity 是给 ODE 关节的上限；section/mass_kg 决定杆的惯量与碰撞体；"
cap "  jam_effort_ratio=0.90 是判堵转的门限（顶到 270 N·m 且角速度归零才算卡滞）；"
cap "  state_rate=20 Hz 就是 3d 那张逐拍表的读数来源。杆的'尺寸可配置'落在这里。"
sleep 6

cap "== 3b 同一份参数被 gen_parking_world.py 落成 SDF：关节限值/阻尼/杆体惯量/碰撞盒 =="
capq "grep -an '<model name=\"parking_gate_in\">' $WORLD"
# 两道闸的关节与碰撞件**同名**（都叫 boom_joint / boom_col），直接对整份 world 取区间
# 会把 gate_out 的那一份也带进来，字幕上就成了"一根杆两个轴"。先按模型块限域。
GB="awk '/<model name=\"parking_gate_in\">/,/<\/model>/' $WORLD"
capq "$GB | awk '/<joint name=\"boom_joint\"/,/<\/joint>/'"
cap "  ↑ lower 0 / upper 1.5708 / effort 300 / velocity 2.0 / damping 0.1 / friction 0.05"
capq "$GB | awk '/<link name=\"boom\">/,/<\/inertial>/'"
capq "$GB | awk '/<collision name=\"boom_col\"/,/<\/collision>/'"
cap "  ↑ 杆身碰撞件是 1.18x0.10x0.06 的盒体、整杆 3 kg —— 落杆砸到车是真接触，"
cap "    不是纯 visual 的贴图杆；机箱反过来刻意不挂碰撞（它压在分隔岛里，车进不去）"
capq "$GB | awk '/<plugin name=\"v2x_barrier\"/,/<\/plugin>/' | sed -n '8,17p'"
sleep 8

cap "== 3c 全自动一轮：核验通过→抬杆→通行→地感占用→离场→落杆，全程不下任何手动指令 =="
cap "  后台同时挂两个只读探针：采样器等'目标角被下发新值'那一刻才开播（提速后上场"
cap "  车一个循环约 80 s，早开播窗口会整段落在空转上）；陷阱等地感占用那一拍下令落杆。"
timeout -k 5 300 python3 $TOOLS/pk_gate_trace.py --gate gate_in --on-change \
       --dur 22 --stride 0.1 --arm-timeout 240 > /tmp/pk3_trace.log 2>&1 &
GT=$!
# 命中后隔 2 s 再确认一次仍占用才下令：占用判据用的是外接方框（半边 1.2 m，刻意放大），
# 只读到一次电平就下令，可能正好卡在释放沿上，那条落杆就成了"正常落杆"而不是防砸。
timeout -k 5 280 bash -c "python3 $TOOLS/pk_wait_gate.py --gate gate_in --occupied --timeout 250 \
  && sleep 2 && python3 $TOOLS/pk_wait_gate.py --gate gate_in --occupied --timeout 2 \
  && python3 $TOOLS/parking_cli.py gate lower --gate gate_in" > /tmp/pk3_trap.log 2>&1 &
TR=$!
pk_wait_pass 250
capq "wait $GT; echo '采样器退出码='\$?"
capq "wait $TR; echo '陷阱退出码='\$?"

cap "== 3d 逐拍表：目标角一跳到 90°，实测角一拍一拍爬（含防砸重抬那一段）=="
capq "head -14 /tmp/pk3_trace.log"
capq "tail -14 /tmp/pk3_trace.log"
cap "  ↑ 看三件事：实测角单调爬升不是阶跃；同一拍上角速度≈2 rad/s 且力矩非零；"
cap "    '本次耗时'按仿真秒累加、到位那一拍归到 open_angle/velocity≈0.79 s 附近。"
cap "  陷阱那条指令是手动令（source=手动），但它只在'杆下正占用'时下出——"
cap "  manager 侧走 refuse_lower_while_occupied，插件侧走防砸重抬，日志见 3e。"
capq "cat /tmp/pk3_trap.log"
sleep 6

cap "== 3e 动作日志的时序对账：保持全开 1.5 s / 单程 0.79 s / 防砸有没有真发生过 =="
capq "python3 $TOOLS/pk_action_stat.py --gate gate_in"
sleep 6
capq "python3 $TOOLS/pk_action_stat.py --gate gate_out --tail 8"
sleep 6
cap "  ↑ 差值是从记录器落盘的动作日志里量的，不是节点自己打的耗时；两道闸各自对账一遍"

cap "== 3f 独立 gzserver 实测：一道闸 + 地坪 + 一个车盒，量超时判据/四种故障/落杆砸车 =="
cap "  这一步会在 GAZEBO_MASTER_URI=11399、ROS_DOMAIN_ID=42 上另起一个 gzserver，"
cap "  只放入口这道闸；它不听主栈指挥、也不动主栈的世界，跑完自己收（实测整轮 79 s）。"
capq "fuser 11399/tcp 2>&1; echo '上一条为空=起跑前 11399 没人占'"
capq "timeout -k 10 170 python3 $TOOLS/test_barrier_dyn.py --gate gate_in > /tmp/pk3_dyn.txt 2>&1; echo 退出码=\$?; wc -l < /tmp/pk3_dyn.txt"
capq "sed -n '1,3p' /tmp/pk3_dyn.txt"
capq "grep -a '✓' /tmp/pk3_dyn.txt | head -12"
capq "grep -aE '✗|断言' /tmp/pk3_dyn.txt"
cap "  ↑ 断言行是这轮唯一的结论：'失败 0' 才说明 5 s 超时、0.08 rad 容差、270 N·m 堵转"
cap "    门限在这台机器上是真能落地的数，不是写在 yaml 里好看的"
capq "fuser -k 11399/tcp 2>/dev/null; sleep 2; fuser 11399/tcp 2>&1; echo '上一条为空=测试实例已收干净'"
capq "pgrep -cf 'gzserv[e]r'"
sleep 6

cap "== 3g 复位：把两道闸交还自动状态机，并读回此刻的关节实测角 =="
capq "$PKCLI gate auto --gate gate_in"
capq "timeout -k 3 20 ros2 topic echo /parking/gate_state v2x_msgs/msg/BarrierState --no-daemon --once 2>&1 | grep -aE 'gate_id|state:|target_rad|angle_rad|loop_occupied|raise_count|lower_count' | head -12"
vid_wait
echo DONE
