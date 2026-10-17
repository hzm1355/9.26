#!/bin/bash
# 下发光照档并等回读：$1 = 预设名（day/backlit/lowlight/night 或 custom:a,b,c,d,e,f）
# 连发 5 次是因为 /gate/lighting 是易失话题，ros2 topic pub --once 有概率在插件
# 订阅者建好连接之前就发完（gate_lighting_test 里也是同一手法连发）。
# 一轮发完读不回就整轮重来：实测插件在密集抓拍/贴牌期间会吃掉整轮下发（条目3
# 那次 restore 到 day 连发 5×8 都没动档位），单轮重试不足以把档位拧回来。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
P=$1
C=''
for round in 1 2 3; do
  for i in 1 2 3 4 5; do
    timeout -k 1 3 ros2 topic pub --once /gate/lighting std_msgs/msg/String "{data: '$P'}" >/dev/null 2>&1
    sleep 0.4
  done
  for i in 1 2 3 4 5 6 7 8; do
    C=$(timeout -k 2 5 ros2 topic echo /gate/lighting/current --once 2>/dev/null \
        | grep '^data:' | sed "s/^data: //" | tr -d "'")
    echo "r${round}try$i -> $C"
    [ "$C" = "$P" ] && break
    sleep 1.5
  done
  [ "$C" = "$P" ] && break
done
if [ "$C" = "$P" ]; then echo "[set_lighting] OK $P"; else echo "[set_lighting] FAIL 仍在 $C"; exit 1; fi
