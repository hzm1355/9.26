#!/bin/bash
# 复位 car_6 之前，把 /spawn_entity 这类服务的**类型名**读回来（不猜：猜错会静默失败，
# 而"删掉了但没生回来"是不可逆的半程状态）。纯只读。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
for S in spawn_entity delete_entity get_entity_state set_entity_state reset_world reset_simulation get_model_list; do
  printf '  %-20s ' "/$S"
  timeout -k 3 20 ros2 service type "/$S" 2>&1 | tr -d '\r' | tail -1
done
echo '== SpawnModel 的字段（以实际装包为准）=='
for M in gazebo_msgs/srv/SpawnEntity gazebo_msgs/srv/SpawnModel gazebo_msgs/srv/DeleteEntity gazebo_msgs/srv/DeleteModel gazebo_msgs/srv/ModelState; do
  printf '  %-32s ' "$M"
  timeout -k 3 20 ros2 interface show "$M" >/dev/null 2>&1 && echo 存在 || echo 无
done
echo DONE
