#!/bin/bash
# #87 现场谁活着：摆位瞬间车被"挪到 1.26 m 外且朝向翻转 180°"，那不是物理，
# 是有人调 /set_entity_state 或发 cmd_vel。把 ROS 节点、进程、服务调用者一次列全。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "=== 1) 相关节点进程（含命令行里的 --car）==="
ps -ef | grep -a '[p]ython3' | grep -aE 'v2x_demo|smart_car_sim|nodes/' | sed 's/^/  /'

echo "=== 2) ros2 node list ==="
timeout -k 2 25 ros2 node list 2>&1 | sed 's/^/  /'

echo "=== 3) 谁在发 cmd_vel（六车逐个）==="
for c in car_1 car_2 car_3 car_4 car_5 car_6; do
  echo -n "  /$c/cmd_vel: "
  timeout -k 2 12 ros2 topic info /$c/cmd_vel 2>&1 | grep -aE 'Publisher count|Subscription count' | tr '\n' ' '
  echo
done

echo "=== 4) 六车位姿（离落点多远）==="
for c in car_1 car_2 car_3 car_4 car_5 car_6; do
  echo -n "  $c: "
  gz model -m $c -p 2>&1 | head -1
done

echo "=== 5) /model_states 里 20 m 内所有模型名（找非 car 的实体）==="
timeout -k 2 20 ros2 topic echo --once /gazebo/model_states --field name 2>&1 | head -40 | sed 's/^/  /'
