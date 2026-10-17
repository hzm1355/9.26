#!/usr/bin/env bash
# 只读核对：回退到 10-14 那一版 world 之前，先确认 10-15 对 world 动的是什么。
# 不写任何文件、不碰进程。
set +e
W=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world
CUR=$W
BK1=${W}.pre_deploy_1015_120330.bak
BK2=${W}.pre_etc_20261015_120340.bak
BKF=${W}.pre_parking_20261014_162819.bak
BFG=${W}.pre_parking_20261014_163544.bak

echo '=== 0) 三份候选基线的大小与行数 ==='
for F in "$CUR" "$BK1" "$BK2" "$BKF" "$BFG"; do
  [ -f "$F" ] && echo "  $(printf '%-56s' "$(basename "$F")") $(stat -c %s "$F") 字节  $(wc -l < "$F") 行  $(stat -c %y "$F" | cut -c1-19)"
done

echo '=== 1) 两份 10-15 12:03 备份是不是同一内容（cmp 只报差异或沉默）==='
cmp "$BK1" "$BK2" && echo "  完全一致：任选一份作基线" || echo "  ↑ 有差异，看上面偏移"

echo '=== 2) 当前 world vs BK1（10-15 12:03）的模型名差集 ==='
names() { grep -aoE '<model name="[^"]+"' "$1" | sed 's/.*="//;s/"//' | sort; }
diff <(names "$BK1") <(names "$CUR") | sed 's/^/  /'
echo "  读法：< = 备份里有而当前没有（回退会加回来）；> = 当前独有（回退会删掉）"

echo '=== 3) 当前 world 相对 BK1 的行数与位姿差（只看 pose 行）==='
echo "  当前 $(wc -l < "$CUR") 行 / BK1 $(wc -l < "$BK1") 行 / 差 $(( $(wc -l < "$CUR") - $(wc -l < "$BK1") )) 行"
diff <(grep -aoE '<pose>-?[0-9][^<]*</pose>' "$BK1") <(grep -aoE '<pose>-?[0-9][^<]*</pose>' "$CUR") \
  | sed 's/^/  /' | head -40

echo '=== 4) BK1 相对 10-14 16:28/16:35 两份备份又动了什么（决定基线选哪份）==='
for B in "$BKF" "$BFG"; do
  echo "  ---- BK1 vs $(basename "$B")：模型名差集 ----"
  diff <(names "$B") <(names "$BK1") | sed 's/^/    /'
  echo "  ---- 行数差：$(( $(wc -l < "$BK1") - $(wc -l < "$B") )) ----"
done

echo '=== 5) 停车场配置与 world 的耦合点（回退后 world 里车位划线几何要还在）==='
grep -anE 'parking_(floor|stripes|booth|gantry|gate_in|gate_out)' "$BK1" | sed 's/^/  BK1 /' | head -8
echo '=== 6) 信号灯杆几何在两份里是否一致（10-13/14 挪过灯杆，别回退掉）==='
for T in tl_j1_ns tl_j1_ew tl_j2_ns tl_j2_ew tl_j3_ns tl_j3_ew; do
  A=$(grep -aA3 "<model name=\"$T\"" "$BK1"  | grep -aoE '<pose>[^<]*' | head -1)
  B=$(grep -aA3 "<model name=\"$T\"" "$BFG"  | grep -aoE '<pose>[^<]*' | head -1)
  C=$(grep -aA3 "<model name=\"$T\"" "$CUR"  | grep -aoE '<pose>[^<]*' | head -1)
  echo "  $T BK1=${A:-无} 16:35备份=${B:-无} 当前=${C:-无}"
done
echo 'DIFFDONE302'
