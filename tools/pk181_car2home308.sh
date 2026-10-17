#!/bin/bash
# 一次把"car_2 归队 → 解除 car_3 的顶住 → 按实测决定要不要重 spawn → 复跑停车场周期"串完。
#
# 为什么是这个顺序（本轮实测，不是猜的）：
#   1) tools/pk180_obb308.py：car_2 从车流退出后一直停在它的 world 出生位
#      (12.0047,-14.6125, yaw=1.5738)，正卡在复位格 (12.00,-12.80) 正南。car_3 由东转北
#      那个原地转，扫掠 29 档里 **28 档与 car_2 重叠、最深 0.257 m**，当前位形离 car_2
#      只有 0.000~0.005 m（SAT 带符号分离量）。所以 [WARN] 给 ω 却 5 s 没转过的第一因
#      是**被顶住**，不是漏登记的隐形盒（pk177 的口径只加载设施件、显式跳过车体，
#      它报"最近件 1.098 m"从来就没把 car_2 算进判据）。
#   2) 但同一时刻的三态探针（tools/pk_probe308.py，shuttle 已杀到 0 实例、探针是唯一
#      发布者）在前进方向完全无障碍的情况下也读到全零：v=0.35 发 6 s -> 位移 0.001 m、
#      轮关节点峰值 0.000 rad/s。这正是 memory「gz-diffdrive-instance-wedge」的第二形态
#      "给什么都不转"。所以还留着"插件缓存也卡了"这一支没排除。
#   3) 用户指令"只用 car3 在停车厂里，car2 回归正常行驶赛道"正好把 1) 的阻塞件移走，
#      于是重测一次探针就能把两支分开：**移开阻塞件后还零出力 -> 才动 /delete_entity**；
#      能出力 -> 病灶只有顶住，不碰车。重 spawn 是高危动作，只在实测支持下做。
#
# 三条硬规矩沿用现场脚本：先删后拷+去 CR；杀同名实例要杀光；杀完 cmd_vel 发布者必须灌零速。
# 只杀 signal_drive_node —— 绝不叫 stop_signal.sh / restart_clean_stack.sh，那两片按
# lib/ 目录整片 pkill，会把停车场的 manager/recorder/shuttle 一起带走（宽 pkill 连带杀伤）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PK=/tmp/v2x_parking
CFG=$SIM/src/v2x_demo/config/signal_drive_params.yaml
CSV=/tmp/signal_drive_path.csv
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "############ A0) 拆之前的硬前置（没过就不动任何进程）############"
GZ=$(pgrep -c gzserver || true)
echo "  gzserver 实例：${GZ:-0}（必须 >=1）"
[ "${GZ:-0}" -ge 1 ] || { echo '!! gzserver 不在，place_cars 与探针全是空转'; exit 1; }
P3=$(gz model -m car_3 -p 2>&1 | head -1)
echo "  car_3 位姿（拆前）：$(echo "$P3" | tr -d '\n' | cut -c1-70)"
echo "$P3" | grep -aq 'Unable' && { echo '!! 运行期 world 里没有 car_3'; exit 1; }
P2=$(gz model -m car_2 -p 2>&1 | head -1)
echo "  car_2 位姿（拆前）：$(echo "$P2" | tr -d '\n' | cut -c1-70)"
NP0=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA0=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  账本基线：通行 $NP0 行 / 闸机动作 $NA0 行"

echo
echo "############ A1) 同步 v2x_demo 源码 + 去 CR + 核对 YAML 内容 ############"
rm -rf ~/v2x_ws/src/v2x_demo
cp -r "$SIM/src/v2x_demo" ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo \( -name '*.py' -o -name '*.yaml' -o -name 'CMakeLists.txt' \) \
  -exec sed -i 's/\r//g' {} +
# 运行期加载的是 ~/v2x_ws 那份副本，所以必须在这一份上核对 drive 开关，而不是看 D 盘。
python3 -c "import yaml;d=yaml.safe_load(open('$HOME/v2x_ws/src/v2x_demo/config/signal_drive_params.yaml'))['signal_drive']['cars'];print('  装载副本里的 drive 开关：',{k:v.get('drive',True) for k,v in d.items()})"
python3 -c "import yaml,sys;d=yaml.safe_load(open('$HOME/v2x_ws/src/v2x_demo/config/signal_drive_params.yaml'))['signal_drive']['cars'];sys.exit(0 if (d['car_2'].get('drive',True) and d['car_3'].get('drive') is False) else 1)"
[ "$?" = "0" ] || { echo '!! drive 开关不对：要求 car_2 归队(缺省或 true)、car_3 留在停车场(false)，停在这里不动进程'; exit 1; }

echo "  归档当前驱动轨迹（重启驱动会把 CSV 以 w 截断）"
if [ -f "$CSV" ] && [ ! -f "${CSV%.csv}.pre_car2home.csv" ]; then
  cp "$CSV" "${CSV%.csv}.pre_car2home.csv"
  echo "    已存 ${CSV%.csv}.pre_car2home.csv：$(wc -l < "${CSV%.csv}.pre_car2home.csv") 行"
else
  echo "    不动（已存在或无 CSV）"
fi

echo "############ A2) colcon build（不编译则 drive 改动不生效）############"
cd ~/v2x_ws
colcon build --symlink-install --packages-select v2x_demo > /tmp/pk181_build.log 2>&1
if [ "$?" = "0" ]; then echo '  BUILD_OK'; else echo '!! 编译失败，日志尾部：'; tail -20 /tmp/pk181_build.log; exit 1; fi
source ~/v2x_ws/install/setup.bash

echo
echo "############ A3) 只停 signal_drive_node 的全部实例 ############"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true); do
  echo "  杀掉 pid $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 1
ND=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
echo "  signal_drive 残留实例：${ND:-0}（必须 0，place_cars 不能和驱动抢方向盘）"
[ "${ND:-0}" = "0" ] || { echo '!! 没杀干净，停在这里'; exit 1; }
echo "  停车场三节点没被碰：manager=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node') recorder=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node') shuttle=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')"

echo "############ A4) 给四辆车流车灌零速，覆盖插件里缓存的巡航指令（car_3 不碰）############"
for c in car_1 car_2 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  echo "  $c 零速已灌"
done

echo "############ A5) place_cars：按 start_idx 把车流车摆回路线起点（跳过 drive:false 的 car_3）############"
timeout -k 2 60 python3 "$SIM/tools/place_cars.py" "$CFG" 2>&1 | tail -8 | sed 's/^/  /'
echo "  摆完 car_2：$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"

echo "############ A6) 重起 signal_drive_node 并做效果复核 ############"
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV > /tmp/signal_drive_fix.log 2>&1 < /dev/null &
sleep 12
echo "  驱动实例：$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node')（必须 1）"
Q1=$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n')
echo "  car_2 第 1 拍：$Q1"
sleep 10
Q2=$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n')
echo "  car_2 第 2 拍：$Q2"
python3 -c "
import sys
a='$Q1'.split(); b='$Q2'.split()
try:
    ax,ay,az,rr,pp,yw=[float(v) for v in a[:6]]; bx,by=map(float,b[:2])
    import math
    d=math.hypot(bx-ax,by-ay)
    print('  car_2 位移 %.3f m、z=%.3f、roll=%.2f pitch=%.2f' % (d,az,rr,pp))
    ok = d>0.8 and abs(rr)<2 and abs(pp)<2 and 6.6<az<6.75 and by>-14.2
    print('  A6 判定：%s（要求 位移>0.8m、姿态没翻、z 在台面、y>-14.2 即已离开停车场南口）' % ('PASS' if ok else 'FAIL'))
    sys.exit(0 if ok else 1)
except Exception as exc:
    print('  A6 判定：读不到位姿 %s' % exc); sys.exit(1)"
A6=$?
if [ "$A6" != "0" ]; then
  echo '!! car_2 没归队（没动 / 姿态不对 / 还在停车场南口）。这时候再探 car_3，'
  echo '   读到的还是"被 car_2 顶住"，两支分不开 —— 停在这里，先看 /tmp/signal_drive_fix.log'
  exit 4
fi

echo
echo "############ B) 阻塞件已让开，重跑 car_3 三态探针（shuttle 保持 0 实例）############"
bash "$SIM/tools/pk_run308.sh" 2>&1 | tail -22 | sed 's/^/  /'

echo "############ C) 按探针实测决定要不要重 spawn car_3 ############"
# 不接管道：`python ... | sed` 之后 $? 是 sed 的退出码，判据会被抹成 0（假绿）。
python3 "$SIM/tools/pk181_decide308.py" /tmp/pk_probe308.log
RC=$?
echo "  判定退出码 $RC（0=出力了/不必重 spawn，2=零出力/要重 spawn，3=探针本身没跑完）"
RESPAWN=0
if [ "$RC" = "2" ]; then
  RESPAWN=1
  echo '  -> 走重 spawn：delete_entity car_3 + spawn 回等待位 + 速度闸 + 重贴车牌'
  bash "$SIM/tools/respawn_car3_263.sh" 2>&1 | tail -30 | sed 's/^/  /'
elif [ "$RC" = "0" ]; then
  echo '  -> 探针显示 car_3 会给油就动，插件没卡死：**不动 /delete_entity**，只把摆渡节点拉起来接手'
else
  echo '  !! 探针没跑完，判据无效，不做重 spawn（也不做后续复跑），把日志留着人工看'
  exit 3
fi

echo
echo "############ D) 拉起停车场三节点（SHUTTLE_MODELS=car_3）############"
SHUTTLE_MODELS=car_3 bash "$SIM/tools/start_parking_nodes.sh" 2>&1 | tail -34 | sed 's/^/  /'

echo "############ E) 盯 30 轮 ≈300 s：位姿 + 闸机 + 账本 + 节点日志尾 ############"
bash "$SIM/tools/pk_watch308.sh" 30 car_3 2>&1 | tail -70 | sed 's/^/  /'

echo
echo "############ F) 收尾对账 ############"
NP1=$(wc -l < "$PK/parking_pass.jsonl" 2>/dev/null || echo 0)
NA1=$(wc -l < "$PK/barrier_actions.jsonl" 2>/dev/null || echo 0)
echo "  通行账本：$NP0 -> $NP1 行；闸机动作：$NA0 -> $NA1 行"
echo '  ## parking_pass.jsonl 尾 3 行'
tail -3 "$PK/parking_pass.jsonl" 2>/dev/null | cut -c1-200 | sed 's/^/     /'
echo '  ## barrier_actions.jsonl 尾 3 行'
tail -3 "$PK/barrier_actions.jsonl" 2>/dev/null | cut -c1-160 | sed 's/^/     /'
echo "  ## 停驶挂起 次数：$(grep -ac '停驶挂起' /tmp/shuttle_car_3.log)"
echo "  ## 没转过 次数：$(grep -ac '没转过' /tmp/shuttle_car_3.log)"
echo "  ## 顶住次数到上限 次数：$(grep -ac '顶住次数到上限' /tmp/shuttle_car_3.log)"
echo "  ## 重 spawn 是否走过：$([ "$RESPAWN" = 1 ] && echo 是 || echo 否（实测有出力）)"
echo "  ## A6 车队效果复核：$([ "$A6" = 0 ] && echo PASS || echo FAIL)"
echo "  ## car_3 终态：$(gz model -m car_3 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo "  ## car_2 终态：$(gz model -m car_2 -p 2>&1 | head -1 | tr -d '\n' | cut -c1-70)"
echo 'PK181DONE'
exit 0
