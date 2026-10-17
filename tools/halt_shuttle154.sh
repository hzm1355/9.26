#!/bin/bash
# 停住"脱困怠速顶亭子"这一件事：先连发零速刹轮，再杀 shuttle，且不重启。
# 为什么现在要停：car_6 此刻楔在 parking_booth 角上（OBB 间隙 -0.000 m），0.08 m/s
# 的倒退指令只挪得动 0.00004 m/s —— 轮子在打滑，每多拖一秒都在往轮位里攒误差，
# 而攒出来的误差会在下一次摆位那一下释放（#77 的弹射就是这个机制）。
# 本脚本**不摆位**：摆位/重 spawn 要用户明文批准后才做。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 0) 停之前现场 ==="
gz model -m car_6 -p 2>&1 | head -1 | sed 's/^/  gz model: /'

echo "=== 1) 连发 3 s 零速（RELIABLE 默认 QoS，插件才收）==="
timeout -k 3 12 ros2 topic pub -r 20 --duration 3 /car_6/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1

echo "=== 2) 杀 shuttle（TERM→KILL 两趟，rclpy 吞 SIGTERM）==="
for PID in $(pgrep -f "lib/v2x[_]demo/parking_shuttle_node"); do
  echo "  kill $PID"; kill "$PID" 2>/dev/null; sleep 1; kill -9 "$PID" 2>/dev/null
done
sleep 2
echo "  剩余 shuttle 实例：$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')（应为 0）"
timeout -k 2 12 ros2 topic info /car_6/cmd_vel 2>&1 | grep -a 'Publisher count' | sed 's/^/  /'

echo "=== 3) 杀完再补一次零速，然后 15 s 静置观察（应当不动）==="
timeout -k 2 10 ros2 topic pub --once /car_6/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
A=$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)
sleep 15
B=$(gz model -m car_6 -p 2>&1 | tr -d '\r' | head -1)
echo "  静置前 $A"
echo "  静置后 $B"

echo "=== 4) 其它停车场节点没被带走（应为 1/1/1）==="
for n in parking_manager_node parking_recorder_node plate_recognizer_node; do
  echo "  $n: $(pgrep -cf "lib/v2x[_]demo/$n")"
done
echo "  shuttle 日志尾 3 行："
tail -3 /tmp/parking_shuttle_node.log 2>/dev/null | cut -c1-200 | sed 's/^/    /'
echo DONE
