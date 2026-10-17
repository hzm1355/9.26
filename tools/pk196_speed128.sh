#!/bin/bash
# 条目 128 复核：提速后的六处 cruise_speed + 摆渡 v/ω 比值，过离线体检与自测。
# 口径要点名清楚（memory「统计数字要点名口径并对账闭合」）：
#   改的是 D 盘源码树；~/v2x_ws/src 那份要等 pk125 同步、install 侧再等 colcon build，
#   进程真正 exec 的是 install 侧。所以这里同时报 D 盘与 home 两侧的数，别让"改了"
#   和"生效了"混成一句话。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
cd "$SIM" || exit 1

echo '=== 1) D 盘六处巡航值（5 条路线 + 1 摆渡）==='
grep -an "cruise_speed" src/v2x_demo/config/signal_drive_params.yaml \
  src/v2x_demo/config/parking_zones.yaml | grep -av '^.*:#' | sed 's/^/  /'
echo "  signal_drive 里 0.85 的行数：$(grep -ac 'cruise_speed: 0.85' src/v2x_demo/config/signal_drive_params.yaml)（要求 5）"
echo "  parking 里 0.85/1.19：$(grep -ac 'cruise_speed: 0.85' src/v2x_demo/config/parking_zones.yaml)/$(grep -ac 'turn_speed: 1.19' src/v2x_demo/config/parking_zones.yaml)（要求各 1）"

echo '=== 2) yaml 逐字段体检（含 v/ω 比值必须还是 0.714；这个脚本要传 yaml 路径）==='
python3 tools/pk_yaml_check165.py src/v2x_demo/config/parking_zones.yaml 2>&1 | tail -18 | sed 's/^/  /'
echo "  退出码 ${PIPESTATUS[0]}"

echo '=== 3) 摆渡车离线自测（走已验证过的包装脚本，它会 source ROS 再跑）==='
bash tools/parking_shuttle_selftest.sh 2>&1 | tail -18 | sed 's/^/  /'
echo "  退出码 ${PIPESTATUS[0]}"

echo '=== 4) home 侧还没同步，先报现状（要生效得跑 pk125 同步 + colcon）==='
for F in config/signal_drive_params.yaml config/parking_zones.yaml; do
  printf '  %-38s home: %s\n' "$F" "$(grep -ahc 'cruise_speed: 0.85' $HOME/v2x_ws/src/v2x_demo/$F 2>/dev/null; true) 行 0.85"
done
echo PK128CHECKDONE
