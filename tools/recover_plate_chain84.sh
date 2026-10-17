#!/bin/bash
# 一次性恢复：重启的第二轮把车牌链和车队驱动一起带走了（宽 pkill 的连带杀伤），
# 而闸机要拍到牌才谈得上"核验通过"。这里只补车牌链 + 默认五副牌，并确认停车场三节点
# 没被这一通操作再次带走。刻意不跑 start_signal.sh —— 它的 stop_signal.sh 会按包目录
# 整片 pkill，会把刚验好的停车场三节点再杀掉一次。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '=== 1) 车牌链（生成器 + 参考识别）==='
timeout -k 5 240 bash $SIM/tools/start_plate_nodes.sh 2>&1 | tail -8
echo "  plate_gen=$(pgrep -cf 'lib/v2x[_]demo/plate_gen_node') plate_recognizer=$(pgrep -cf 'lib/v2x[_]demo/plate_recognizer_node')"

echo '=== 2) 下发默认车牌（含 car_6 前后两块）==='
timeout -k 5 240 bash $SIM/tools/apply_default_plates.sh 2>&1 | tail -10

echo '=== 3) 停车场三节点还在不在（被带走就当场补拉）==='
for N in parking_manager_node parking_recorder_node parking_shuttle_node; do
  C=$(pgrep -cf "lib/v2x[_]demo/$N")
  echo "  $N 实例 $C"
  [ "$C" = "1" ] || timeout -k 5 120 bash $SIM/tools/start_parking_nodes.sh $N.py 2>&1 | tail -3
done
echo '=== 4) 收尾快照 ==='
echo "  gzserver=$(pgrep -x gzserver | tr '\n' ' ')"
for N in parking_manager_node parking_recorder_node parking_shuttle_node plate_gen_node plate_recognizer_node speed_detect_node; do
  printf '  %-24s %s\n' "$N" "$(pgrep -cf "lib/v2x[_]demo/$N")"
done
exit 0
