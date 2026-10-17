#!/bin/bash
# recover205 段 4 的现场复核：账本长没长、车在哪、shuttle 在哪个 phase、卡口插件有没有见过上升沿。
# 只读，不动任何进程。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
PKJ=/tmp/v2x_parking/parking_pass.jsonl

echo '=== 1) pk99 是否还在跑（420 s 到点会自己收）==='
ps -o pid,etime,args -C bash 2>/dev/null | grep -a 'pk99_recover205' | grep -av grep
echo '--- 日志尾 12 行 ---'
tail -12 /mnt/d/desktop1/9.26.1/_diag155/recover205.log 2>/dev/null | tr -d '\r'

echo '=== 2) 账本 ==='
echo "  通行 $(wc -l < $PKJ 2>/dev/null) 行｜动作 $(wc -l < /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null) 行"
echo '  --- 最新 3 条 ---'
tail -3 $PKJ 2>/dev/null | cut -c1-200

echo '=== 3) car_6 位姿 + 距抓拍点 (12.0,-11.4) 的直线距离 ==='
timeout -k 5 25 python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/pose155.py car_6 2>&1 | tail -2
echo '  触发带 near 1.4 ~ far 2.5：由远及近进带才产生上升沿'

echo '=== 4) shuttle 当前 phase ==='
timeout -k 3 15 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null | grep -m1 '^data:' | cut -c1-200

echo '=== 5) 两道闸 ==='
timeout -k 5 25 ros2 service call /parking/gate v2x_demo/srv/SetParkingGate "{}" 2>&1 | tr -d '\r' | head -6

echo '=== 6) manager 最近动作（有没有在发 route，还是挂起）==='
tail -25 /tmp/parking_manager_node.log 2>/dev/null | tr -d '\r' | cut -c1-170
echo '=== 7) 卡口插件日志（触发/抓拍有没有被打印）==='
ls -la /tmp/pk98_*.log /tmp/parking_*.log 2>/dev/null | tail -8
echo CHECK207_DONE
