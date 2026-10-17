#!/bin/bash
# 长跑/录制前的车队体检：五车是不是真的在动。
#
# 为什么需要：驱动节点活着 ≠ 车在跑。实测出现过五车同时 0 速度停摆 7 分钟
# （脱困逻辑反复"复位到别的起点"把车队甩出路线后互锁），这段时间卡口一张图都不出，
# 任何"等抓拍/等告警"的取证窗口都会空转到超时。
# 判据用 fleet_peek 的 10 s 位移：全部 < 0.5 m 就认定卡死，直接 reset_fleet 复位。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

OUT=$(timeout -k 3 45 python3 -u $SIM/tools/fleet_peek.py 10 2>&1 | tr -d '\r')
echo "$OUT" | grep -aE '^car_' | tail -5
# 位移列是"  xx.xx m"这一对，取每行 "数字 + 空格 + m" 里的那个数字
DISTS=$(echo "$OUT" | grep -aE '^car_' | sed 's/.*[ ]\([0-9.]*\)[ ]m[ ].*/\1/')
MOV=$(echo "$DISTS" | awk '{if ($1+0 >= 0.5) n++} END{print n+0}')
echo "10 s 内有位移的车 = ${MOV:-0} / 5（最大位移 $(echo "$DISTS" | sort -n | tail -1) m）"
# 为什么看"几辆在动"而不是"最大位移"：卡死时驱动节点会反复把某辆车 teleport 回复位点，
# 那两辆的 10 s 位移能到 30 m，按最大值判就永远判成"在动"——实测 12:0x 就是这样空转掉一整轮。
if [ "${MOV:-0}" -ge 3 ]; then
  echo '[fleet_guard] 至少三辆在跑，无需复位'
  exit 0
fi
echo '[fleet_guard] 多数车辆停摆 -> 执行 reset_fleet.sh'
bash $SIM/tools/reset_fleet.sh 2>&1 | tr -d '\r' | tail -3
sleep 25
timeout -k 3 45 python3 -u $SIM/tools/fleet_peek.py 10 2>&1 | tr -d '\r' | grep -aE '^car_' | tail -5
echo '[fleet_guard] 复位后复检完成'
