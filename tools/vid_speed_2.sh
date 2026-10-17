#!/bin/bash
# 任务四 条目2 视频取证：基于车辆实时位姿仿真雷达测速，定点瞬时 + 区间平均两种模式。
#
# 这一段为什么这么排：
#   · 被动观测，不摆位不改阈值：五辆车按 signal_drive 的 0.7 m/s 巡航（= 2.52 km/h），
#     这个已知真值让"雷达读数对不对"在同一帧里就能判（面板两列并排：雷达 / 位姿）；
#   · 区间点位 ~150 s 才过一次车（车队一圈 ~108 s，还要等红灯），窗口必须给到 400 s 以上，
#     否则"区间平均测速"整段零事件，会被读成没实现；
#   · 明细用 speed_probe.py 直接打原始话题行（不靠面板反推），字幕里贴的就是节点发出来的
#     那一条消息，含夹角/距离/峰值这些面板放不下的字段。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
OUT="$VID_ROOT/任务四_测速路段/条目2_位姿仿真雷达测速_定点瞬时与区间平均.mp4"
PROBE=/tmp/vid_s2_probe.txt

vid_start "$OUT" 470 "$VIEW_MON,$VIEW_INTER" speed \
  "任务四 条目2  位姿仿真雷达测速：定点瞬时 + 区间平均（同帧对照真值）"
cap "雷达模型：读数 = 位姿速度在波束轴上的投影 v·cos(夹角) + 高斯噪声；位姿列是同刻真值对照"
cap "判定用的速度取 0.30 s 窗口（3 格仿真时钟）内各子区间弦速的中位数，不是整窗弦长/整窗时长"
capq "grep -aE 'radar_noise_kmh|min_pass_kmh|max_sane_kmh|vel_window_s' ~/v2x_ws/src/v2x_demo/config/speed_zones.yaml"
capq "grep -aE '^[[:space:]]+cruise_speed' ~/v2x_ws/src/v2x_demo/config/signal_drive_params.yaml | head -1; echo '^ 车队指令速度 m/s（×3.6 = 2.52 km/h 真值）'"
timeout -k 3 440 python3 -u "$TOOLS/speed_probe.py" 420 $PROBE > /tmp/vid_s2_probe.log 2>&1 &
PP=$!
sleep 8
cap "== 2a 定点瞬时：车驶过点位 center 圆、离开圆时发一条 SpeedCapture =="
sleep 70
cap "  面板'条目2 定点瞬时'区每来一条抓拍就多一行：点位/车/牌/雷达/位姿/夹角/阈值/判定"
sleep 70
cap "  夹角 0.0° = 波束正对来车（标准装法，投影即实速）；SZ-B-south-instant 波束朝 180°、"
cap "  SZ-B-east-segment 朝 -90°，同样是正对，所以三处夹角都近 0，读数与真值只差噪声"
sleep 70
cap "== 2b 区间平均：进入 entry 圆开始计时、离开 exit 圆发 SpeedSegment，均速 = 断面长度 / 行程时间 =="
sleep 70
cap "  区间行给 长度/时长/均速：反向通过不计入，中途位姿复位只作废本次通行（不产生假读数）"
sleep 60
cap "  区间点位过车稀疏（一圈 ~108 s + 等灯），这一段就是为了让两类模式都真的出事件"
sleep 60
cap "== 2c 本窗口明细（speed_probe 原样打印的话题行）：定点瞬时 =="
capq "grep -a '^\[瞬时\]' $PROBE | tail -4 | cut -c1-118"
cap "== 2d 区间平均明细 =="
capq "grep -a '^\[区间\]' $PROBE | tail -3 | cut -c1-118"
capq "wait $PP; tail -1 /tmp/vid_s2_probe.log"
vid_wait
echo DONE
