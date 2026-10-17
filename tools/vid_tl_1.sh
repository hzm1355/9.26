#!/bin/bash
# 任务二 条目1 视频取证：机动车红绿灯 + 行人信号灯，灯色切换具备真实时序特性。
# 画面取路口相机（能同时看到三组灯头），右侧面板是 6 组灯的实时相位/剩余秒/生效配时。
# 片尾把 tl_timeline.py 采到的跳变时间轴和 tl_seq_live.py 的三条在线判据写进字幕。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务二_信号灯配时/条目1_灯色切换真实时序.mp4"
INTER=$VIEW_INTER

vid_start "$OUT" 185 "$INTER" tl \
  "任务二 条目1  机动车灯+行人灯+真实时序（路口相机实拍 + 6 组灯实时状态）"
cap "== 0 摘除自适应算法节点：本条目要看的是预设配时本身的相序，不与系数混叠 =="
adap_stop
pub /traffic_light/preset default      # 基准 10/3/10，周期 23 s，一屏能看全一次完整相序
cap "验证点1：机动车相序恒为 GREEN -> YELLOW -> RED，不得跳变"
cap "验证点2：行人灯是独立子相位 WALK -> FLASHING -> DONT_WALK，且与同向机动车绿放行错开"
cap "验证点3：黄灯时长固定 3.0 s，面板第二行的 绿/黄/红 即当前生效配时"
sleep 20
cap "观察画面左侧灯头：此刻面板里 J1_ew 的相位见右栏，颜色块=机动车/行人"
sleep 20
capq "timeout -k 3 45 python3 -u $TOOLS/tl_detail.py 6 both"
cap "↑ state_detail（配时/剩余/模式）+ stats（排队/累计通过/均等待/系数）实时读回"
sleep 8
cap "接下来 20 s 采样灯态跳变时间轴（按仿真时钟），取末段完整相位写进字幕"
capq "timeout -k 3 45 python3 -u $TOOLS/tl_timeline.py 20 | tail -12"
cap "↑ 每行=一次相变的仿真时刻 + 上一相位持续（墙钟打点×实测实时倍率换算，避开 /clock 10 Hz 的量化误差）"
cap "== 在线相序合规判据：50 s 采样，数非法跳变 / 黄灯每段时长 / 行人 WALK 撞机动车绿 =="
capq "timeout -k 3 70 python3 -u $TOOLS/tl_seq_live.py 50"
vid_wait
adap_start
