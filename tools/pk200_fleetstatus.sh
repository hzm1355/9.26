#!/bin/bash
# 车队状态定性：六台车 12 s 两次位姿差分 + 谁在发 cmd_vel + drive 节点加载的是哪份配置。
#
# 为什么第一步必须是"动没动"而不是"位置对不对"：memory「生效要用效果复核」——
# 静止复检不算证据。四台车现在挤在南环一带不动，至少三种成因长得一模一样：
#   A) 正常等红灯（J2_ew/J2_ns 的停车线就在这片，等杆是应该的）；
#   B) drive 节点活着但没出力（实例卡死 / 参数文件是旧的 / 车被判定"挂起"）；
#   C) 有别的东西在发 cmd_vel 把它们顶住（shuttle 或残留实例）。
# 只量位姿分不出这三样，所以要同时读 cmd_vel 的发布者计数与该节点的实测速度。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 1) 12 s 两次位姿，逐台给位移（>0.15 m 才算真在走）==='
for M in car_1 car_2 car_4 car_5 car_3 car_6; do
  eval "P1_$M=\$(timeout 8 gz model -m $M -p 2>&1 | head -1 | tr -d '\r')"
done
sleep 12
for M in car_1 car_2 car_4 car_5 car_3 car_6; do
  P2=$(timeout 8 gz model -m $M -p 2>&1 | head -1 | tr -d '\r')
  eval "P1=\$P1_$M"
  printf '  %s：' "$M"
  # 用已验证过的 pose_delta144.py（它的 0.5 m/自转判据是给"摆位后静止复检"用的，
  # 这里只取它打印的位移与 z 两个数来定性"在走 / 静止 / 掉台"，不采纳它的退出码）。
  python3 "$SIM/tools/pose_delta144.py" "$P1" "$P2" 2>&1 | head -1 | sed 's/^ *//'
  echo "     读回：$(echo "$P2" | cut -c1-58)"
done

echo '=== 2) 谁在给每辆车发 cmd_vel（Publisher count 应为 1=drive，出现 2 就是有人在抢）==='
for M in car_1 car_2 car_4 car_5 car_3; do
  printf '  %-7s ' "/$M/cmd_vel"
  timeout -k 2 12 ros2 topic info "/$M/cmd_vel" 2>&1 | tr -d '\r' | grep -a 'Publisher count' | head -1
done
echo '  drive 节点实测速度（/car_1/odom 的线速度，2 s 采样）'
timeout -k 3 12 ros2 topic echo /car_1/odom --no-daemon --once 2>&1 | tr -d '\r' \
  | grep -aA2 'twist:' | grep -a 'x:' | head -2 | sed 's/^/    /'

echo '=== 3) drive 节点加载的是哪份参数文件、进程起了多久、日志尾==='
PID=$(pgrep -f 'lib/v2x[_]demo/signal[_]drive' | head -1)
echo "  pid=$PID  运行 $(ps -o etime= -p $PID 2>/dev/null | tr -d ' ')"
tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-200 | sed 's/^/    /'
PF=$(tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | grep -ao '/tmp/[^ ]*yaml' | head -1)
echo "  参数文件：$PF"
[ -n "$PF" ] && grep -an 'cruise_speed\|drive:' "$PF" 2>/dev/null | head -8 | sed 's/^/    /'
for LOG in /tmp/signal_drive_node.log /tmp/signal_drive.log /tmp/drive.log; do
  [ -f "$LOG" ] && echo "  ## $LOG 尾 6 行" && tail -6 "$LOG" | cut -c1-165 | sed 's/^/    /'
done

echo '=== 4) 停车场侧现状（car_3 归队期间 shuttle 必须停着，否则两边抢同一辆车）==='
echo "  shuttle 实例 $(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')（要求 0）"
grep -a 'gate_in' /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null | tail -2 | cut -c1-190 | sed 's/^/    /'
echo FLEETSTATUSDONE
