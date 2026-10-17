#!/usr/bin/env bash
# car_7 在 J3 停止位画圈的归因取证（只读，不改现场）。
#
# 为什么读 /signal_drive/status：signal_drive_node.py 每拍把每台车的分支归因拼成一行发出来
# （第 737-739 行：横偏 lat、限 limit、limited_by=who、phase），这正是"它此刻走的是哪条
# 分支"的自述。我只有 cmd_vel 的读数时只能反推（-0.12 钉停 / 0.0+0.45 大角度转），
# 反推容易把"被灯限停"和"被障碍限停"混成一个。
#
# 口径纪律：所有 ros2 读数先原样 tee 落盘，再从文件里取数（不在管道上现挂过滤式）；
# 话题名先 ros2 topic list 原样落盘再挑，不猜。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
OUT=/tmp/car7_branch298
mkdir -p "$OUT"
LOG=$OUT/run.log
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash

say '===== 0) 现场谁活着 ====='
for N in signal[_]drive_node traffic[_]light[_]manager etc[_]manager[_]node etc[_]gate[_]node etc[_]recorder[_]node; do
  say "  ${N} 实例 $(pgrep -cf "$N" || true)"
done
say "  gzserver pid $(pgrep -x gzserver | head -1 || true)"

say '===== 1) 话题名单原样落盘（不猜名字）====='
timeout -k 3 30 ros2 topic list --no-daemon > "$OUT/topics.txt" 2>&1
say "  写了 $(wc -l < "$OUT/topics.txt") 行，其中含 light 的行："
grep -ai 'light\|signal' "$OUT/topics.txt" | sed 's/^/    /' | tee -a "$LOG"

say '===== 2) /signal_drive/status 连续 30 拍（1 Hz 级）====='
: > "$OUT/status.txt"
timeout -k 3 45 ros2 topic echo /signal_drive/status --no-daemon > "$OUT/status.txt" 2>&1
NS=$(grep -ac 'car_7=' "$OUT/status.txt")
say "  抓到含 car_7 的行 $NS 行（原样前 6 行）："
grep -a 'car_7=' "$OUT/status.txt" | head -6 | cut -c1-300 | sed 's/^/    /' | tee -a "$LOG"

say '  car_7 的归因字段分布（limited_by 是最后一个词）：'
grep -a 'car_7=' "$OUT/status.txt" \
  | sed 's/.*限[0-9.]* //' | sort | uniq -c | sort -rn | head -12 \
  | sed 's/^/    /' | tee -a "$LOG"
say '  同一窗里其它车作为参照（证明不是全局故障）：'
grep -a 'car_1=' "$OUT/status.txt" | tail -3 | cut -c1-300 | sed 's/^/    /' | tee -a "$LOG"

say '===== 3) 航迹 CSV 里 car_7 最近 300 行的 limited_by 全程分布 ====='
CSV=/tmp/signal_drive_path.csv
if [ -f "$CSV" ]; then
  say "  $CSV 共 $(wc -l < "$CSV") 行"
  tail -400 "$CSV" > "$OUT/csv_tail.txt"
  awk -F, '$2=="car_7"{c[$7]++} END{for(k in c) printf "    %s -> %d\n", k, c[k]}' "$OUT/csv_tail.txt" \
    | tee -a "$LOG"
  say '  car_7 最近 14 行原样（t,car,x,y,yaw,speed,limited_by,phase）：'
  awk -F, '$2=="car_7"' "$OUT/csv_tail.txt" | tail -14 | sed 's/^/    /' | tee -a "$LOG"
else
  say "  读不到 $CSV（本轮链路可能没开 path_csv）"
fi

say '===== 4) J3 东西向灯态（用话题名单里真有的那个）====='
LT=$(grep -a '^/traffic_light/J3_ew/state$' "$OUT/topics.txt" | head -1)
say "  话题名单里 J3_ew 的状态话题：${LT:-没找到}"
if [ -n "$LT" ]; then
  timeout -k 3 25 ros2 topic echo "$LT" --no-daemon > "$OUT/light.txt" 2>&1
  sed 's/^/    /' "$OUT/light.txt" | head -14 | tee -a "$LOG"
fi

say '===== 5) car_7 位姿与朝向（航向对不上前视点才是画圈的直接原因）====='
for I in $(seq 1 6); do
  timeout -k 3 20 python3 $SIM/tools/pose155.py car_7 >> "$OUT/pose.txt" 2>&1
  sleep 3
done
grep -a '当前位姿' "$OUT/pose.txt" | sed 's/^/  /' | tee -a "$LOG"

say '===== 6) /tmp/signal.log 里 car_7 的最近动作行 ====='
grep -a 'car_7' /tmp/signal.log 2>/dev/null | tail -16 | cut -c1-200 | sed 's/^/  /' | tee -a "$LOG"
say "DONE298"
exit 0
