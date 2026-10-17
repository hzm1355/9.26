#!/bin/bash
# 丁 闸拦下后必须定性"为什么这 200 s 一次都没过杆"。两种读法完全不同：
#  甲、周期本来就比 200 s 长（一轮 = 进+出+复位 ≈ 98 s 仿真 + loop_period 120 s 仿真，
#      除以 RTF 0.76 可能顶到 ~290 s 墙钟）—— 那是判据窗口给短了，不是故障；
#  乙、摆渡车又挂了（#104 的过冲—回修自激或看门狗封顶）—— 那条目 5 根本没法录。
# 用日志的"轮跑完/停驶挂起"计数与最后时刻来分这两支，不靠猜。
set +e
LOG=/tmp/shuttle_car_3.log
echo "--- 现在 ---"; date '+%T'
echo "--- 进程 ---"
printf '  shuttle %s   manager %s   recorder %s   vid_rec %s\n' \
  "$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')" \
  "$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')" \
  "$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')" \
  "$(pgrep -cf 'vid_rec[.]py')"
echo "--- 计数：轮跑完 / 停驶挂起 / WARN / 顶住 ---"
printf '  轮跑完 %s / 停驶挂起 %s / WARN %s / 顶住 %s\n' \
  "$(grep -ac '轮跑完' $LOG)" "$(grep -ac '停驶挂起' $LOG)" \
  "$(grep -ac 'WARN' $LOG)" "$(grep -ac '顶住\|没转过\|挪不到' $LOG)"
echo "--- 日志尾 12 行 ---"
tail -12 $LOG | cut -c1-200
echo "--- 最后一条 shuttle_status 面板读数（车现在在干什么）---"
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source ~/v2x_ws/install/setup.bash >/dev/null 2>&1
timeout -k 3 15 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null \
  | grep -am1 '^data:' | cut -c1-200
echo "--- 两道闸当前态 ---"
timeout -k 3 25 python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/parking_cli.py gate list 2>&1 | head -6 | cut -c1-170
echo "--- 账与位姿 ---"
wc -l /tmp/v2x_parking/parking_pass.jsonl
gz model -m car_3 -p 2>&1 | head -1
