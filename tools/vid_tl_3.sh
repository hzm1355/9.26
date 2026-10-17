#!/bin/bash
# 任务二 条目3 视频取证：接入外部自适应配时算法（基于场景车流量动态调周期）
#                + 早高峰/晚高峰/平峰多套预设方案定时切换。
# 自适应系数是外部节点通过 /traffic_light/<组>/adaptive_factor 灌进管理器的，
# 所以"摘掉算法节点 + 手动把 6 组系数复位成 1.0"之后，预设读回的才是方案原值，
# 不与系数混叠；片尾再把算法节点拉回来，看系数重新偏离 1.0。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务二_信号灯配时/条目3_自适应算法接入与多套预设定时切换.mp4"
INTER=$VIEW_INTER

vid_start "$OUT" 340 "$INTER" tl \
  "任务二 条目3  外部自适应算法接入 + 早/晚高峰·平峰·夜间多套预设切换"
cap "外部算法节点 = adaptive_timing_node.py（独立进程，订阅 /model_states 统计进口道排队）"
adap_start
sleep 6
cap "== 3a 自适应在环：算法侧的输入（车辆位姿）与输出（各组系数/需求量） =="
capq "timeout -k 3 45 python3 -u $TOOLS/tl_adaptive_probe.py 22"
cap "↑ 右栏 系数 列即 adaptation_factor；绿时长 = 基准绿 × 系数（下限 3 s）"
sleep 6
cap "== 3b 摘除外部算法节点，并把 6 组系数手动复位为 1.0（隔离出自适应的影响） =="
adap_stop
sleep 4
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 both"
cap "↑ 系数全部回到 1.00，绿时长=预设原值"
sleep 4
for p in default peak offpeak night; do
  case $p in
    default)  s=30; d="默认 10/3/10" ;;
    peak)     s=40; d="高峰 14/3/14（周期 31 s，需等一整个周期换相）" ;;
    offpeak)  s=26; d="平峰 7/3/7" ;;
    night)    s=20; d="夜间 5/3/5" ;;
  esac
  cap "== 3c 切换预设方案 -> $p（$d）=="
  pub /traffic_light/preset "$p"
  sleep "$s"
  capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail | sed -n '1,7p'"
done
cap "== 3d 时段自动调度：管理器日志里的准点切换记录 =="
# 管理器可能由 launch 拉起（/tmp/signal.log），也可能是取证脚本单独重启的实例（/tmp/tlmgr_vid.log）
capq "grep -ahE '时段自动切换|手动预设切换|preset ->' /tmp/signal.log /tmp/tlmgr_vid.log 2>/dev/null | tail -6 | sed 's/.*\]: //'"
sleep 4
cap "== 3e 把外部自适应算法节点重新拉起，观察系数再次偏离 1.0 =="
adap_start
capq "timeout -k 3 45 python3 -u $TOOLS/tl_adaptive_probe.py 22"
vid_wait
