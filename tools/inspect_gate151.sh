#!/bin/bash
# 第 151 轮：动手前先看清"车—杆—闸"三者的当前关系。
#
# 为什么必须先看清再挪：shuttle_status 报 car_6 停在 (12.00,-12.21)，到 gate_in 轮轴
# (12.00,-11.21) 只有 1.0 m，比 near_radius=1.4 还近 —— 车头朝北，半截车身很可能已经
# 压在放平的杆上。这时无脑倒车就是在跟一根刚性碰撞体较劲，轻则楔住攒轮位误差（上一轮
# 就是这么把车顶出沙盘的），重则把杆别弯。先拿 BarrierState 的 angle_rad 和 gz 的实测
# 位姿把"压没压"这件事判死。
set +e
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "== 1) car_6 实测位姿（gz-transport，不经 DDS）=="
gz model -m car_6 -p 2>&1 | head -2 | tr -d '\r' | sed 's/^/  /'

echo "== 2) 两道闸的当前状态 =="
timeout -k 3 20 ros2 topic echo /parking/gate_state --no-daemon 2>&1 | tr -d '\r' |
  awk '/gate_id/{g=$2} /state:/{s=$2} /angle_rad/{a=$2} /target_rad/{t=$2}
       /loop_occupied/{l=$2} /^---/{if(g!=""){printf("  %-10s state=%-8s angle=%-8s target=%-8s loop=%s\n",g,s,a,t,l); g=""}}' |
  head -8

echo "== 3) manager 日志尾（它为什么没抬杆）=="
tail -8 /tmp/parking_manager_node.log 2>/dev/null | cut -c1-170 | sed 's/^/  /'

echo "== 4) shuttle 日志尾（作废 3 轮是怎么作废的）=="
tail -10 /tmp/parking_shuttle_node.log 2>/dev/null | cut -c1-170 | sed 's/^/  /'

echo "== 5) 抓拍与识别 =="
echo "  gzserver.log 里 '抓拍' 总行数: $(grep -ac '抓拍' /tmp/gzserver.log 2>/dev/null)"
grep -a '抓拍' /tmp/gzserver.log 2>/dev/null | tail -4 | cut -c1-170 | sed 's/^/  /'
echo "  recognizer 日志尾:"
tail -6 /tmp/plate_recognizer_node.log 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo "== 6) 运行期 shuttle 加载的是哪份 route（改动生效了没有）=="
grep -a 'entry#\|航线\|route' /tmp/parking_shuttle_node.log 2>/dev/null | head -3 |
  cut -c1-170 | sed 's/^/  /'
grep -ac '\-13.95' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/parking_zones.yaml |
  sed 's/^/  yaml 里 -13.95 计数: /'
echo DONE
