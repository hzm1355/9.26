#!/bin/bash
# 只读体检：分清"car_6 卡死"与"整个仿真没在跑"。
# 上一轮 300 s 探针里 gate_in/gate_out 的角度和 car_6 位姿三次采样一模一样，
# 那个现象既符合"车顶住碰撞体不动"，也符合"gzserver 暂停/物理不推进"——
# 两者的处置完全不同（前者要复位车，后者要醒世界），所以先用 /clock 是否推进来劈开。
# 每条都是报告，读不到也要报完，故 set +e。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 1) /clock 在不在推进（间隔 8 s 两点）==="
# /clock 是 transient local + best effort 的常见坑：echo 要 --once 重试，
# 拿不到点别当成"没时钟"，原文都落盘了。
for i in 1 2; do
  timeout -k 3 20 ros2 topic echo /clock --no-daemon --once > /tmp/p86_clock$i.txt 2>&1
  C=$(grep -a 'sec:' /tmp/p86_clock$i.txt | head -1 | tr -dc '0-9')
  echo "  第 $i 点 $(date +%T)：sec=$C  （原文 /tmp/p86_clock$i.txt，$(wc -l < /tmp/p86_clock$i.txt) 行）"
  [ $i = 1 ] && sleep 8
done

echo "=== 2) car_6 全位姿两点（z 与俯仰滚转看是不是摔出界/翻了）==="
for i in 1 2; do
  timeout -k 3 20 gz model -m car_6 -p > /tmp/p86_pose$i.txt 2>&1
  echo "  第 $i 点 $(date +%T): $(head -1 /tmp/p86_pose$i.txt)"
  [ $i = 1 ] && sleep 8
done
echo "  （停车场台面 z≈6.70，界外地面层 z≈0.45；位姿第 3 个数就是 z）"

echo "=== 3) /car_6/cmd_vel 有没有人发、发的什么 ==="
timeout -k 3 20 ros2 topic info /car_6/cmd_vel -v > /tmp/p86_cmd.txt 2>&1
grep -aiE 'Publisher count|Subscription count|Node name|QoS profile' /tmp/p86_cmd.txt | sed 's/^/  /'
timeout -k 3 20 ros2 topic echo /car_6/cmd_vel --no-daemon --once > /tmp/p86_cmd1.txt 2>&1
echo "  最新一帧指令原文行=$(wc -l < /tmp/p86_cmd1.txt)：$(tr -d '\n' < /tmp/p86_cmd1.txt | cut -c1-160)"

echo "=== 4) 两道闸实测 ==="
for G in gate_in gate_out; do
  timeout -k 3 20 ros2 topic echo /parking/gate/$G/state --no-daemon --once \
      > /tmp/p86_$G.txt 2>&1
  echo "  ## $G（$(wc -l < /tmp/p86_$G.txt) 行）"
  grep -aE 'state:|angle_rad|at_target|loop_occupied|vehicle_id' /tmp/p86_$G.txt | sed 's/^/     /'
  grep -aqE 'loop_occupied' /tmp/p86_$G.txt || echo "     !! 读不到回包（不代表闸机无状态，见原文）"
done

echo "=== 5) 停车场三节点 + 车牌链实例数 ==="
for N in parking_manager_node parking_recorder_node parking_shuttle_node \
         plate_gen_node plate_recognizer_node plate_recorder_node; do
  echo "  $N: $(pgrep -cf "lib/v2x[_]demo/${N}")"
done

echo "=== 6) 账本 ==="
ls -l /tmp/v2x_parking/ 2>/dev/null | sed 's/^/  /'
exit 0
