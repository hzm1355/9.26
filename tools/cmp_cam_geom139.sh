#!/bin/bash
# 对比"已验证能读到 0.99 的卡口龙门相机"和"停车场闸机相机"的安装几何。
# 为什么先看这个：识别器 find_plate 的形状门槛（长宽比 2.2~5.5）是在卡口那套几何下
# 标出来的，而竖直车牌投到画面里的长宽比 = 3.5/sin(俯视角)。门槛 5.5 等价于
# "相机看牌的俯视角不得低于 39.5°"。停车场闸机若低于这个角，牌就会被当成"太扁的条"
# 拒掉 —— 这不是抓拍没拍到，是两个子系统的标定不兼容。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
echo '=== 卡口龙门相机（任务二/三在用，识别 0.99）==='
grep -rn -B4 -A14 'gate_j2_south\|gate_j3_north' "$SIM/src/v2x_gazebo/worlds" 2>/dev/null \
  | grep -aE 'name=|pose=|camera|horizontal_fov|gate_j' | head -40
echo
echo '=== 停车场闸机相机（本次 0.30 / 未定位）==='
grep -n -A6 'camera:' "$SIM/src/v2x_demo/config/parking_zones.yaml" \
  | grep -aE 'gate_id|pose:|focus|capture_point|hfov|near_radius'
echo
echo '=== 生成器里这两类相机的来源 ==='
grep -rn 'pose' "$SIM/tools/gen_parking_world.py" 2>/dev/null | grep -aiE 'camera|gate' | head -12
exit 0
