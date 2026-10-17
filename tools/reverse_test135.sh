#!/bin/bash
# 决定性实验：car_6 现在带着一份"没人发却一直在执行"的指令（odom 自报 v=+0.098、w=-2.717）
# 原地打转。发一条**反向**的角速度，看 odom 的 w 会不会翻转：
#   翻转   → cmd_vel 通路是活的，那"零速流没让它停"就得另找解释（比如它只在有非零残值时生效）；
#   不翻转 → 插件根本不再收 cmd_vel，这个转与 ROS 无关，只能靠重启 gzserver 复位。
# 反向流结束后立刻停发，再采一次，看它是不是回到原来的 -2.717（回到原值=latch 在插件里）。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
unset ROS_DOMAIN_ID

snap() {
    # 先落盘再取值：直接在管道尾猜过滤式，一旦字段格式变了就静默产出空证据，
    # 读起来像"odom 没数据"，其实是我的 grep 没匹配上（本项目的老坑）。
    timeout -k 2 12 ros2 topic echo /car_6/odom --no-daemon --once --field twist.twist \
      >/tmp/_od135.txt 2>&1
    printf '  %s %s\n' "$(date +%H:%M:%S)" "$(tr -d '\r' < /tmp/_od135.txt | tr -d '\n ' | cut -c1-140)"
}

echo '== 0) 起始状态 =='
snap
snap

echo '== 1) 反向角速度流（w=+3.0）持续 20 s，期间采 4 次 =='
timeout -k 2 20 ros2 topic pub -r 10 /car_6/cmd_vel \
  geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}' \
  >/tmp/_rev135.txt 2>&1 &
REV=$!
sleep 4
timeout -k 2 12 ros2 topic info /car_6/cmd_vel -v 2>&1 | grep -aE 'count|Node name' | head -6
snap
sleep 5
snap
sleep 5
snap
wait $REV 2>/dev/null

echo '== 2) 停发后 6 s（看是不是回到 latch 的原值）=='
sleep 6
snap
snap

echo '== 3) 再来一段零速流 20 s，期间采 3 次 =='
timeout -k 2 20 ros2 topic pub -r 10 /car_6/cmd_vel \
  geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  >/tmp/_zero135b.txt 2>&1 &
Z=$!
sleep 6
snap
sleep 7
snap
wait $Z 2>/dev/null
echo '== 4) 零速流刚结束后 =='
snap
snap
echo DONE
