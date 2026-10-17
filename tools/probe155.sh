#!/bin/bash
# 只读+激励判别：car_6 在 (6.467,-12.644) 给油 0.10 m/s x 7 s 只挪 0.003 m，
# 而几分钟前它才跑完一整周期（入口 P0002、出口 P0003 都自动放行）。
# 两种成因要分开：
#   A 被实体顶住 —— 轮子在转、底盘不动，odom 会跟着积分往前走；
#   B diff_drive 实例卡死 —— 轮子也不转，odom 也不动（memory「gz-diffdrive-instance-wedge」）。
# 所以三路一起看：OBB 最紧的几行（上一版 tail 取了输出末端=最松的件，是错的）、
# 四段激励的实测位姿进展与 odom 进展、以及轮子 link 到底动没动。
set +e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=$WS/tools
W=$WS/src/smart_car_sim/worlds/v2x_city_scene.world
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '[1] 当前位姿'
gz model -m car_6 -p 2>/dev/null | tr -d '\r'

echo '[2] OBB 全量输出（最紧的排在前面，这次取头几行）'
timeout -k 10 90 python3 "$TOOLS/car_vs_colliders.py" "$W" \
  6.467 -12.644 -0.911 0.80 0.40 > /tmp/obb155.txt 2>&1
grep -a '' /tmp/obb155.txt | head -12
echo '  ...（完整文件在 /tmp/obb155.txt）'

echo '[3] 四段激励判别（纯ω 小/大、纯v 前/后，每段 5 s，收尾无条件零速）'
timeout -k 15 120 python3 "$TOOLS/drive_probe153.py" car_6 2>&1 | tail -22

echo '[4] 轮子到底转没转：/joint_states 轮关节角 + /link_states 轮 link 姿态'
timeout -k 10 60 python3 "$TOOLS/wheel_spin155.py" car_6 0.10 6 2>&1 | tail -18
echo "[probe155] DONE"
