#!/usr/bin/env bash
# 车姿态异常诊断：gz pose / odom 连续采样 / 物理日志 / 转向节角度
export GAZEBO_MASTER_URI=http://localhost:11345
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '== A) gz 里车的完整 pose（model 级） =='
gz model -m v2x_prius -i 2>/dev/null | sed -n '/^ - /p;/name: v2x_prius$/p' | head -20

echo '== B) /odom 连续采样 6 次（间隔 1s，看是否仍在移动） =='
for i in 1 2 3 4 5 6; do
  P=$(timeout 3 ros2 topic echo /odom --once --field pose.pose 2>/dev/null \
      | grep -E 'x:|y:|z:|w:' | tr -d ' ' | tr '\n' ' ')
  echo "  t$i  $P"
  sleep 1
done

echo '== C) 转向节角度（复用 steer_probe 采样） =='
A=$(gz model -m v2x_prius -i 2>/dev/null \
    | grep -E 'name: "v2x_prius::front_(left|right)_steer_joint"' -A2 \
    | grep 'angle:' | tr -d ' ' | tr '\n' ' ')
echo "  $A"

echo '== D) gzserver.log 全文（物理警告检查） =='
cat /tmp/gzserver.log

echo '== E) /cmd_vel 是否有残余发布者 =='
timeout 3 ros2 topic info /cmd_vel 2>/dev/null | grep -E 'Publisher|Subscription' | head -6

echo '[diag] 完成'
