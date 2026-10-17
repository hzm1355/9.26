#!/bin/bash
# 只读旁观：看 car_3 在无干预下能不能连跑多轮、"停驶挂起"会不会复现。
# 为什么要有这个：A 方案（重起 shuttle）只证明了一次"重启后能跑"，不能证明
# 上一轮那个"同一航点误差翻转 5 次"的挂起是偶发；判偶发要靠连跑，不能靠一次。
# 全程只读日志与行数，不发 cmd_vel、不动位姿。
set +e
MIN=${1:-30}
LOG=/tmp/shuttle_car_3.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
END=$((SECONDS + MIN * 60))
while [ $SECONDS -lt $END ]; do
  T=$(timeout -k 3 15 ros2 topic echo /clock --qos-profile sensor_data --no-daemon --once 2>/dev/null \
        | tr -d '\r' | grep -am1 'sec:' | tr -dc '0-9')
  DONE=$(grep -ac '轮跑完' $LOG)
  HANG=$(grep -ac '停驶挂起' $LOG)
  WARN=$(grep -acE 'WARN' $LOG)
  P=$(gz model -m car_3 -p 2>&1 | head -1 | awk '{printf "%s %s yaw=%s", $1, $2, $6}')
  N=$(wc -l < /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null)
  echo "sim=${T:-?} 跑完 ${DONE} 轮 / 停驶挂起 ${HANG} / WARN ${WARN} | pass ${N:-?} 行 | 位 $P"
  sleep 100
done
echo "--- 采样结束，日志尾 ---"
tail -30 $LOG | cut -c1-200
