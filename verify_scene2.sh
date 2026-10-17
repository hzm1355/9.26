#!/usr/bin/env bash
# 新场景元素逐一抽查（gz model -m X -i 验证存在与坐标）+ 信号灯话题
export GAZEBO_MASTER_URI=http://localhost:11345
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '== 元素抽查（存在性 + 坐标） =='
for m in road_NS-main road_EW-east VIADUCT-01_deck VIADUCT-01_pillar_0 \
         ETC-01_beam ETC-02_beam PKG-NW_barrier PKG-NW_sign_P PKG-NW_access \
         TL-J1-NE TL-J2-E RSU-J1-NE_pole SCHOOL-01 OFFICE-NW TOWER-NE \
         TREES-NS-N_t0 building_e2 obstacle_1 zebra_J2_NS-school_0 stopline_J1_EW-west_p; do
  P=$(gz model -m "$m" -i 2>/dev/null | grep -A3 'position {' | grep -E 'x:|y:|z:' | head -3 | tr -d ' ' | tr '\n' ' ')
  if [ -n "$P" ]; then
    printf '  OK  %-24s %s\n' "$m" "$P"
  else
    printf '  ??  %-24s (无输出)\n' "$m"
  fi
done

echo '== 高架立柱根数 =='
N=0
for i in 0 1 2 3 4; do
  gz model -m "VIADUCT-01_pillar_$i" -i >/dev/null 2>&1 && N=$((N+1))
done
echo "  立柱 0-$((N-1)) 共 $N 根"

echo '== 信号灯话题 =='
timeout 4 ros2 topic list 2>/dev/null | grep -i traffic
timeout 4 ros2 topic echo /traffic_light/state --once 2>/dev/null | head -3
echo '[verify2] 完成'
