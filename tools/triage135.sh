#!/bin/bash
# 分诊：聋的是"整条 ROS→gzserver 输入路"，还是只有 car_6 那个 diff_drive 实例？
# 前者意味着在环什么都验不了（闸机指令也进不去）；后者只说明 car_6 的插件实例死了。
# 全部是只读操作：一次服务调用 + 两段位姿采样，不发任何行驶指令、不动闸机。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID

echo '== 1) gzserver 的 ROS 服务还接不接（gazebo_ros_state 是 world 里挂的）=='
timeout -k 3 25 ros2 service call /gazebo/get_model_state gazebo_msgs/srv/GetModelState \
  "{model_name: car_6}" 2>&1 | tr -d '\r' | head -6

echo '== 2) 五车这 12 s 有没有在动（它们由 signal_drive_node 指挥）=='
timeout -k 3 25 ros2 topic echo /model_states --no-daemon --once --field name \
  >/tmp/_n135a.txt 2>&1
timeout -k 3 25 ros2 topic echo /model_states --no-daemon --once --field pose \
  >/tmp/_p135a.txt 2>&1
sleep 12
timeout -k 3 25 ros2 topic echo /model_states --no-daemon --once --field pose \
  >/tmp/_p135b.txt 2>&1
# 解析动作放独立文件（triage_pose135.py），不要内联 heredoc：一次性命令里挂过。
python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/triage_pose135.py
echo '== 3) 闸机这 12 s 的实测角有没有动（barrier 插件是另一条 ROS 输入路）=='
for G in gate_in gate_out; do
    printf '  %s ' "$G"
    timeout -k 3 15 ros2 topic echo /parking/gate/$G/state --no-daemon --once \
      --field angle_rad 2>&1 | tr -d '\r' | tail -1
done
echo DONE
