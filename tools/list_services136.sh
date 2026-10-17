#!/bin/bash
# 分诊已定论：barrier 插件听得到 ROS 指令，只有 car_6 的 diff_drive 实例聋了。
# 那"最小代价复位"就看 gzserver 现在到底注册了哪些服务 —— 有 delete/spawn 就能只重
# spawn car_6（不必整栈重启），一个都没有就只能整栈重启。纯只读列名，不调用。
set +e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '== 全部服务名（第一次跑要等 DDS 发现，重试到非空）=='
for i in 1 2 3; do
  timeout -k 3 25 ros2 service list > /tmp/_sv136.txt 2>&1
  [ -s /tmp/_sv136.txt ] && grep -c . /tmp/_sv136.txt && break
  echo "  第 $i 次空回包，重试"; sleep 8
done
cat /tmp/_sv136.txt | tr -d '\r' | sed 's/^/  /'

echo '== 其中与"删模型/生模型/关节/力"有关的 =='
grep -aiE 'spawn|delete|joint|wrench|force|pose|state|reset' /tmp/_sv136.txt | sed 's/^/  /'

echo '== car_6 的关节实测（看轮子现在被设成什么速度）=='
timeout -k 3 20 gz model -m car_6 -j 2>&1 | head -12
echo DONE
