#!/usr/bin/env bash
# 上线 signal_drive 的"原地画圈"修法，并只重启 signal_drive 一支节点，用效果复核判定。
#
# 改了什么（D 盘是真值，两处都在 src/v2x_demo/v2x_demo/signal_drive_node.py）：
#   F1 speed_limit() 的停车线循环：旧代码第一行就是 `if abs(wrap(ag.yaw-head))>1.0: continue`
#      （本意：横向腿的车别被这条线拦）。副作用是把"在停止线前被转到歪斜"的车直接放行 ——
#      car_7 实测在 (-3.6..-4.4, 1.5..2.3) 0.8 m 见方里连续绕圈，/signal_drive/status 里
#      同一台车一会儿 `限0.00 J3_ew(RED)`、一会儿 limited_by 空（=巡航），14 s 一圈不停。
#      新写法先算几何带（未过线 + 在本车道 + 25 m 内），带内 + 该停时，朝向歪也判 limit=0，
#      车就只给转向不给前进油，慢慢回正，画圈的能量来源被切断。
#   F2 航点超时分支：被信号灯合法拦停时不再打 "航点 N 超时…交给脱困/复位"。Ring A 的几何
#      决定了这条日志必然是假的：J3_ew 停止线 x=-2.0、standoff 2.4 → 合法停车位 x≈-4.4，
#      而航点 1 在 (-0.8,1.5)，差 3.6 m > arrive_radius 1.1，红灯期间永远进不了到达圈。
#
# 为什么不走 tools/start_signal.sh：它第 7 行调 stop_signal.sh（旧版宽 pkill 已改窄，但它
# 还会重跑 place_cars.py —— 对已经存在的 car_7 是一步瞬移，正是「摆位弹射与复位风暴」那条
# 坑）。这里只杀 signal_drive 一个具名节点、只起它一个，灯控/自适应/记录器/ETC 全部不动。
#
# 三条重启纪律（沿用 restart_drive_only.sh）：先杀光再起一个；杀完必须灌零速（diff_drive
# 插件会缓存最后一条指令继续推车）；重启会把轨迹 CSV 以 w 模式截断，所以先归档。
# 判据一律看效果不看回读：改前 30 s 和改后 40 s 各量一次 car_7 的航向累计扫过角和位置
# 包围盒，另外拿 car_1/car_4 的净位移当"没把车流搞停"的回归护栏。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
SRC=$HOME/v2x_ws/src/v2x_demo
NODE=$SIM/src/v2x_demo/v2x_demo/signal_drive_node.py
CSV=/tmp/signal_drive_path.csv
OUT=/tmp/fix_spin_299
LOG=$OUT/run.log
mkdir -p "$OUT"
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash

# yaw 累计扫过角（解卷绕）+ 位置包围盒。只吃 CSV 的切片，口径：20 Hz 一行一车。
trail() {
  local tag=$1 lo=$2
  awk -F, -v s="$lo" 'NR>s && $2=="car_7" {
      if (!got) { px=$3; py=$4; p=$5; got=1; x0=$3; y0=$4; minx=$3; maxx=$3; miny=$4; maxy=$4 }
      d=$5-p; while (d>3.14159) d-=6.28318; while (d<-3.14159) d+=6.28318
      sw+=d; if ($3<minx) minx=$3; if ($3>maxx) maxx=$3
      if ($4<miny) miny=$4; if ($4>maxy) maxy=$4; p=$5; n++ }
    END { printf "  %s car_7 样本 %d 行｜航向累计扫过 %.2f rad｜位置包围盒 %.2f x %.2f m\n",
          tag, n+0, sw+0, maxx-minx, maxy-miny }' "$CSV"
}

say '===== 1) 改前基线（不先量出"确实在画圈"，改后的"不画圈"就没有意义）====='
say "  $CSV 现有 $(wc -l < "$CSV" 2>/dev/null || echo 无) 行"
L0=$(wc -l < "$CSV" 2>/dev/null || echo 0)
sleep 30
trail '改前' "$L0"
say '  改前 /signal_drive/status 里 car_7 的原样三行（含限/归因）：'
timeout -k 3 25 ros2 topic echo /signal_drive/status --no-daemon 2>&1 \
  | grep -a 'car_7=' | head -3 | cut -c1-260 | sed 's/^/    /' | tee -a "$LOG"

say '===== 2) 同步 D 盘 → ~/v2x_ws/src（cp 覆盖，不 rm -rf，避免吃掉未回归的改动）====='
diff -rq "$SIM/src/v2x_demo" "$SRC" 2>/dev/null | head -12 | sed 's/^/  改前差异 /' | tee -a "$LOG"
cp -a "$SIM/src/v2x_demo/." "$SRC/" 2>> "$LOG"
find "$SRC" \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name 'CMakeLists.txt' \) \
  -exec sed -i 's/\r//g' {} +
diff -q "$NODE" "$SRC/v2x_demo/signal_drive_node.py" >/dev/null 2>&1 \
  && say '  同步后 signal_drive_node.py 与 D 盘一致 OK' || say '!! 同步后仍不一致，停'
grep -ac '车头歪' "$SRC/v2x_demo/signal_drive_node.py" | sed 's/^/  新代码锚点1（车头歪）命中 /' | tee -a "$LOG"
grep -ac 'if not ag.stopped:' "$SRC/v2x_demo/signal_drive_node.py" | sed 's/^/  新代码锚点2（超时抑制）命中 /' | tee -a "$LOG"
python3 -m py_compile "$SRC/v2x_demo/signal_drive_node.py" && say '  PYCOMPILE_OK' || say '!! 语法错，停'

say '===== 3) 归档轨迹 + 只杀 signal_drive 的全部实例 ====='
[ -f "$CSV" ] && [ ! -f /tmp/signal_drive_path.pre_fix299.csv ] \
  && cp "$CSV" /tmp/signal_drive_path.pre_fix299.csv
say "  归档：$(wc -l < /tmp/signal_drive_path.pre_fix299.csv 2>/dev/null || echo 无) 行"
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x[_]demo signal[_]drive_node' || true); do
  kill "$PID" 2>/dev/null || true
done
sleep 3
for PID in $(pgrep -f 'lib/v2x[_]demo/signal_drive_node' || true); do kill -9 "$PID" 2>/dev/null || true; done
sleep 1
N=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
say "  signal_drive 残留实例 ${N:-0}（必须 0，留一个就是双发布者抢 cmd_vel）"
N=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
say "  复认 ${N:-0}"

say '===== 4) 杀完立刻灌零速（car_6 归 parking_shuttle 管，不掺手）====='
for c in car_1 car_2 car_3 car_4 car_5 car_7; do
  R=$(timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' 2>&1 | tail -1)
  say "  ${c}: ${R:-已发}"
done

say '===== 5) 只起 signal_drive 一支（install 是符号链接，不重编）====='
setsid nohup ros2 run v2x_demo signal_drive_node.py \
  --ros-args -p use_sim_time:=true -p path_csv:=$CSV \
  > /tmp/signal_drive_fix299.log 2>&1 < /dev/null &
sleep 14
M=$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)
say "  实例数 ${M:-0}（必须正好 1）"
say '  节点日志尾部（原样）：'
tail -8 /tmp/signal_drive_fix299.log | cut -c1-200 | sed 's/^/    /' | tee -a "$LOG"
say '  其它三支没被动过（各 1）：'
for N in traffic[_]light[_]manager adaptive[_]timing_node traffic[_]light[_]recorder etc[_]manager[_]node; do
  say "    ${N} 实例 $(pgrep -cf "$N" || true)"
done

say '===== 6) 改后效果复核（40 s 窗口，同一把尺子）====='
: > "$OUT/status_after.txt"
timeout -k 3 40 ros2 topic echo /signal_drive/status --no-daemon >> "$OUT/status_after.txt" 2>&1
say '  car_7 的归因行（取 4 行原样）：'
grep -a 'car_7=' "$OUT/status_after.txt" | head -4 | cut -c1-260 | sed 's/^/    /' | tee -a "$LOG"
L1=$(wc -l < "$CSV")
sleep 40
trail '改后' "$L1"
say '  回归护栏：车流没被我搞停（同窗 car_1/car_4 的净位移，从位姿流实测）'
for c in car_1 car_4; do
  timeout -k 3 20 python3 $SIM/tools/pose155.py $c >> "$OUT/pose_after.txt" 2>&1
  sleep 2
  timeout -k 3 20 python3 $SIM/tools/pose155.py $c >> "$OUT/pose_after.txt" 2>&1
done
grep -a '当前位姿' "$OUT/pose_after.txt" | awk '{
    for (i=1; i<=NF; i++) { if ($i ~ /^x=/) {sub(/^x=/,"",$i); x=$i+0}
                             if ($i ~ /^y=/) {sub(/^y=/,"",$i); y=$i+0} }
    name=$1
    if (!(name in fx)) { fx[name]=x; fy[name]=y }
    lx[name]=x; ly[name]=y }
  END { for (k in fx) printf "    %s 两点净位移 %.2f m\n", k, sqrt((lx[k]-fx[k])^2+(ly[k]-fy[k])^2) }' \
  | tee -a "$LOG"

say '===== 7) ETC 侧现场没被牵连 + 道闸当前角 ====='
timeout -k 3 25 ros2 service call /etc/query v2x_msgs/srv/EtcQuery \
  "{plate_number: '', vehicle_id: 'car_7', obu_id: '', gate_id: '', pass_id: '', result: 0, \
    t0: 0.0, t1: 0.0, limit: 3, newest_first: true, include_identify: true}" \
  > "$OUT/query.txt" 2>&1
say "  $(grep -aoE 'success=[A-Za-z]+|matched=[0-9-]+' "$OUT/query.txt" | tr '\n' ' ')"
# 道闸角用已验证过的那一发读法（/etc/gate/<闸名>/state），不改口径。
timeout -k 3 20 ros2 topic echo /etc/gate/etc_gate_1/state --no-daemon --once > "$OUT/boom.txt" 2>&1
AN=$(awk '/^ *angle_rad:/ {print $2; exit}' "$OUT/boom.txt")
say "  道闸实测角 angle_rad=${AN:-读不到}（>=1.49 抬到位＝西腿通；<=0.1 放平＝横拦西腿）"
say "  gzserver pid $(pgrep -x gzserver | head -1 || true)"
say 'DONE299'
exit 0
