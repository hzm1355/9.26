#!/usr/bin/env bash
# car_7 修复后的真实轨迹复测（只读）。上一版把 $3/$4 当数字比大小取 min/max，awk 在
# 两边都是字符串时按字典序比（"-4.3" > "-3.5"），所以报出 0.08 x 3.06 m 这种自相矛盾的
# 包围盒。这一版一律 +0 强转数值，并把首末行原样贴出来对账。
#
# 顺带查两件本段冒出来的事：
#   1) etc_manager_node 实例数报 2 —— 双 manager 会一起抢 /etc/gate 指令、双写设备台账，
#      这是重启纪律里最容易漏的一条，必须点名到 PID 与完整命令行才判得清是真双实例
#      还是我的匹配式数到了别的东西（pgrep 的 -f 会匹整条命令行）。
#   2) 改后 car_7 的 status 一直是 限0.70（绿灯无拦停者）却只有 v=0.09 —— 要么"指令没送到"
#      "轮子没出力"，要么"被顶住"，三态分开排：cmd_vel 原样 + odom + 实测位移同窗对比。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CSV=/tmp/signal_drive_path.csv
OUT=/tmp/car7_where300
LOG=$OUT/run.log
mkdir -p "$OUT"; : > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash

say '===== 0) ETC 三件套的实例点名到 PID（不只看计数）====='
pgrep -af 'v2x[_]demo' > "$OUT/procs.txt" 2>&1
say "  命中 $(wc -l < "$OUT/procs.txt") 行，其中 etc 相关的原样："
grep -a 'etc' "$OUT/procs.txt" | cut -c1-190 | sed 's/^/    /' | tee -a "$LOG"

say '===== 1) 本轮 CSV 里 car_7 的数值口径轨迹（全程，行数应该不多）====='
say "  $CSV 现有 $(wc -l < "$CSV" 2>/dev/null || echo 无) 行，表头原样："
head -1 "$CSV" 2>/dev/null | sed 's/^/    /' | tee -a "$LOG"
awk -F, '$2=="car_7" { x=$3+0; y=$4+0; p=$5+0; v=$6+0; t=$1+0;
    if (!got) { got=1; minx=maxx=x; miny=maxy=y; p0=p; t0=t; x0=x; y0=y; tp=p }
    if (x<minx) minx=x; if (x>maxx) maxx=x; if (y<miny) miny=y; if (y>maxy) maxy=y;
    d=p-tp; while (d>3.14159) d-=6.28318; while (d<-3.14159) d+=6.28318; sw+=d; tp=p;
    tv=t; sp=v; n++ }
  END { printf "  样本 %d 行｜t %.1f→%.1f s｜x %.2f..%.2f（跨度 %.2f m）｜y %.2f..%.2f（跨度 %.2f m）｜\n",
        n+0, t0, tv, minx, maxx, maxx-minx, miny, maxy, maxy-miny;
        printf "  航向 %.2f→%.2f rad，累计扫过（解卷绕）%.2f rad｜末行实测速度 %.2f m/s｜首末直线净位移 %.2f m\n",
        p0, tp, sw, sp, sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0)) }' "$CSV" 2>/dev/null | tee -a "$LOG"
say '  car_7 最近 8 行原样（t,car,x,y,yaw,speed,limited_by,phase）：'
awk -F, '$2=="car_7"' "$CSV" 2>/dev/null | tail -8 | sed 's/^/    /' | tee -a "$LOG"

say '===== 2) 实时 60 s：位姿 + status + cmd_vel + odom 同窗对账 ====='
: > "$OUT/live.txt"
for I in $(seq 1 6); do
  timeout -k 3 15 python3 $SIM/tools/pose155.py car_7 >> "$OUT/live.txt" 2>&1
  timeout -k 3 15 ros2 topic echo /signal_drive/status --no-daemon --once >> "$OUT/live.txt" 2>&1
  timeout -k 3 15 ros2 topic echo /car_7/cmd_vel --no-daemon --once >> "$OUT/live.txt" 2>&1
  timeout -k 3 15 ros2 topic echo /car_7/odom --no-daemon --once 2>&1 \
    | grep -aE '^ *(x|z): ' | head -6 >> "$OUT/live.txt"
  sleep 4
done
say '  逐轮（位姿行 + 该车 status 片段 + cmd 两行原样）：'
grep -aE '当前位姿|car_7=\(|^ *x: |^ *z: ' "$OUT/live.txt" | cut -c1-150 | sed 's/^/    /' | tee -a "$LOG"

say '===== 3) 节点日志里 car_7 与 car_1 的最近动作行（原样）====='
grep -aE 'car_7|car_1' /tmp/signal_drive_fix299.log 2>/dev/null | tail -14 \
  | cut -c1-190 | sed 's/^/  /' | tee -a "$LOG"
say '===== 4) 灯态（口径：/traffic_light/J3_ew/state 一条原样）====='
timeout -k 3 20 ros2 topic echo /traffic_light/J3_ew/state --no-daemon --once >> "$OUT/light.txt" 2>&1
sed 's/^/  /' "$OUT/light.txt" | head -4 | tee -a "$LOG"
say 'DONE300'
exit 0
