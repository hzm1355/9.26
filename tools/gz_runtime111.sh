#!/usr/bin/env bash
# #111 运行期加载的到底是哪份 world：文件 mtime 对比 gzserver 启动时间，并直读运行态本体。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
OUT=/tmp/gz111.txt
: > $OUT

echo "=== 0) gzserver 进程与启动时刻 ===" >> $OUT
for pid in $(pgrep -x gzserver); do
  echo "PID=$pid  start_ticks=$(awk '{print $22}' /proc/$pid/stat)" >> $OUT
  ps -o lstart=,etime= -p $pid >> $OUT 2>&1
  echo "  cwd=$(readlink /proc/$pid/cwd)" >> $OUT
  echo "  exe=$(readlink /proc/$pid/exe)" >> $OUT
done

echo "=== 1) world 文件 mtime（与上面启动时刻比） ===" >> $OUT
stat -c '%y  %s  %n' "$W" >> $OUT 2>&1
echo "  其它同名副本：" >> $OUT
find "$SIM" -name 'v2x_city_scene.world*' -printf '%T@ %TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null | sort -n >> $OUT

echo "=== 2) 运行态本体：car_1 与 car_6 的 gz model -i ===" >> $OUT
source /opt/ros/humble/setup.bash 2>/dev/null
export HOME=${HOME:-/root}
for m in car_1 car_6; do
  echo "--- $m ---" >> $OUT
  timeout 20 gz model -m $m -i >> $OUT 2>&1
done

echo "=== 3) 运行态位姿 ===" >> $OUT
for m in car_1 car_6; do
  printf '%-7s ' $m >> $OUT
  timeout 20 gz model -m $m -p >> $OUT 2>&1
done

echo "=== 4) world 里所有 diff_drive 插件的 wheel_separation 行（文件侧） ===" >> $OUT
grep -n 'wheel_separation\|wheel_diameter\|robot_base_frame\|odometry_frame' "$W" >> $OUT 2>&1

wc -l $OUT
cp -f $OUT "$SIM/_gz111.txt"
echo "copied rc=$?"
