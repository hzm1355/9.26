#!/bin/bash
# 只读：为复位段原地转点选点，把"扫掠圆 vs 每件碰撞体"按姿态逐个算，不靠中心距。
#
# 现场：155 轮整栈重启后入口闸 P0002、出口闸 P0003 全自动放行，但复位段
# reset#1=[6.80,-12.80,0.0] 这一转 4 次 5 s 零进展 → 车楔在 tl_j1_ns_ped
# (5.800,-12.600, 圆柱碰撞 r=0.10) 上，实测 OBB 重叠 -0.041 m。
# 该转点离杆中心 1.02 m，而原地转的扫掠半径 = hypot(1.20,0.40)+0.10 = 1.365 m，
# 所以这个点从几何上就不可能转得开，必须换点或改航线。
#
# 这里做两件事：
#   A) 现转点按 yaw 从 -90° 转到 0° 分 6 档算净空（证明它不是"刚好差一点"）
#   B) 候选转点 (7.60,-12.80) 同样分档算，并把它和别的候选一起排个净空下限
# 判据取 OBB 分离轴间隙（tools/car_vs_colliders.py 同口径），>0.15 m 才算能转。
set +e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$WS/src/smart_car_sim/worlds/v2x_city_scene.world

minline() {
  python3 "$WS/tools/car_vs_colliders.py" "$W" "$1" "$2" "$3" 0.80 0.40 2>&1 \
    | grep -a '==>'
}

turn() {
  local X=$1 Y=$2 TAG=$3
  echo "--- $TAG：($X,$Y) 从 -1.5708 转到 0 的六档姿态净空 ---"
  for AW in -1.5708 -1.2566 -0.9425 -0.6283 -0.3142 0.0; do
    printf '  yaw=%+0.4f  ' "$AW"
    minline "$X" "$Y" "$AW"
  done
  echo
}

turn 6.80 -12.80 '现转点（复位#1 原样）'
turn 7.60 -12.80 '候选A 东移 0.80'
turn 7.20 -12.80 '候选B 东移 0.40'
turn 8.00 -12.80 '候选C 东移 1.20'

echo "--- 候选A 的驶入段（出场驶出 6.80,-9.95 → 7.60,-12.80 这条斜线）沿线净空 ---"
echo "    这段要穿 gate_out 的杆轴 y=-11.05，且要压地感带，所以沿线采 8 点算 OBB"
for T in 0.0 0.15 0.3 0.45 0.6 0.75 0.9 1.0; do
  X=$(python3 -c "print('%.3f' % (6.80 + 0.80 * $T))")
  Y=$(python3 -c "print('%.3f' % (-9.95 - 2.85 * $T))")
  printf '  t=%.2f (%s,%s)  ' "$T" "$X" "$Y"
  minline "$X" "$Y" -1.4300
done
echo "[turn_clearance155] DONE"
