#!/bin/bash
# car_6 的 diff_drive 插件实例到底还活着没有：拿它和一辆正常车比 odom 发布。
#
# 判据逻辑：cmd_vel 订阅者数已经确认 = 1（指令送得到），但 4 段指令合计该转 1.75 rad
# 实测 -0.024 rad、该走 0.6 m 实测 0.018 m —— 响应基本为零。剩下两种解释：
#   插件实例死了/卡了 → 它连 odom 也不会再发；
#   插件活着但轮子被物理顶住 → odom 是插件按轮速自己积分的，照发不误。
# 所以 /car_6/odom 有没有帧可以直接把两条分开，而且完全只读、不动仿真。
set +e
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "== 1) 两辆车的 odom 话题在不在 =="
ros2 topic list 2>/dev/null | grep -a 'odom' | tr -d '\r' | sed 's/^/  /'

echo "== 2) 各读一帧（重试 4 次；DDS 发现竞态的空回包不算'没有'）=="
for t in /car_2/odom /car_6/odom; do
  ok=0
  for i in 1 2 3 4; do
    R=$(timeout -k 3 10 ros2 topic echo --once "$t" 2>/dev/null | tr -d '\r' | head -8)
    if [ -n "$R" ]; then
      echo "  $t 有帧:"; echo "$R" | sed 's/^/    /'; ok=1; break
    fi
    sleep 2
  done
  [ "$ok" = "0" ] && echo "  $t  重试 4 次仍无帧"
done

echo "== 3) 发布者计数（DDS 侧，注意这个数有 ~20 s 滞后，只当旁证）=="
for t in /car_2/odom /car_6/odom; do
  echo "  $t  $(timeout -k 3 10 ros2 topic info "$t" 2>/dev/null | tr -d '\r' | grep -a 'Publisher count')"
done

echo "== 4) gzserver 日志里跟 car_6 有关的告警 =="
tail -400 /tmp/gzserver151.log 2>/dev/null | grep -ai 'car_6\|freeze\|large velocity\|nan\|error' |
  tail -12 | cut -c1-170 | tr -d '\r' | sed 's/^/  /'
echo "  （日志名如果不对，这一步就是空的，别当成'没有告警'）"
ls -1t /tmp/gz*.log /tmp/*gzserver* 2>/dev/null | head -5 | tr -d '\r' | sed 's/^/  候选日志: /'
echo DONE
