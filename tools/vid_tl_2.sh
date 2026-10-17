#!/bin/bash
# 任务二 条目2 视频取证：手动配时（红/绿/黄时长可分别设置）+ 单路口独立配置 + 多路口联动配时。
# 0   先摘掉外部自适应算法节点：它会把绿/红时长再乘一遍自己的系数，读回值对不上设定值。
# 2a  只改 J2_ns 一组，其余 5 组数值不动 —— 单路口独立；
# 2b  再叠加改 J1_ew —— 两组可同时各自独立；
# 2c 一条 group:"all" 指令同时改配六组 —— 多路口联动配时；
# 2d 重启管理器让 offset 重新播种，现场量 60 s 起绿相位差，实测值应逐项等于 YAML 配置，
#     且同路口 ns/ew 的"同时为绿"应为 0 s（严格反相）。
#     注：管理器的相位状态机只在首次播种时按 offset 对齐主时钟网格，之后自由推进；
#     自适应跑久了会把相位差冲散，不重启直接量会得到一串对不上配置的数，那是假证据。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务二_信号灯配时/条目2_手动配时_单路口独立与多路口联动.mp4"
INTER=$VIEW_INTER

vid_start "$OUT" 290 "$INTER" tl \
  "任务二 条目2  手动配时：单路口独立配置 + 多路口联动配时（相位差实测）"
ack_start 285
cap "指令下发走 tl_pub.py：先等管理器订阅者出现再连发 3 次（ros2 topic pub --once 会被 DDS 发现竞态吃掉）"
cap "   字幕里的『applied』一行 = 管理器在 traffic_light/config_ack 上回发的实际生效值"
cap "== 0 摘除外部自适应算法节点并把 6 组系数复位，隔离出纯手动配时 =="
adap_stop
sleep 4
pub /traffic_light/preset default
sleep 6
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "↑ 基准：预设 default 六组 10/3/10，mode=FIXED"
sleep 4
cap "== 2a 单路口独立：只把 J2_ns 改成 绿4 / 黄2 / 红12 =="
cfg '{"group":"J2_ns","green":4,"yellow":2,"red":12}'
sleep 6
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "↑ 只有 J2_ns 一行变成 4.0/2.0/12.0，其余 5 组仍是 10/3/10 = 单路口独立生效"
sleep 6
cap "== 2b 再改 J1_ew 为 绿9 / 黄3 / 红6，验证第二组可独立叠加配置 =="
cfg '{"group":"J1_ew","green":9,"yellow":3,"red":6}'
sleep 6
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "↑ J1_ew=9/3/6 与 J2_ns=4/2/12 同时成立，其余 4 组不变"
sleep 6
cap "== 2c 多路口联动配时：一条 group=all 指令同时改配六组 8/2/8 =="
cfg '{"group":"all","green":8,"yellow":2,"red":8}'
sleep 6
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "↑ 六组 G/Y/R 同时变成 8.0/2.0/8.0 = 多路口联动一次改配"
sleep 4
cfg '{"group":"all","green":10,"yellow":3,"red":10}'
cap "== 2d 重启管理器按 offset 重新播种，再现场量 60 s 起绿相位差 =="
mgr_restart
sleep 22
cap "   配置相位差：J1_ns=0(主时钟) J3_ns=3 J2_ns=6；J1_ew complementary=红时长；ew 组 anti_phase_of 本路口 ns 组"
capq "timeout -k 3 120 python3 -u $TOOLS/tl_offset_audit.py 60 J1_ns"
cap "↑ 实测相位差应逐项等于配置值；同路口 ns/ew 同时为绿=0 s（严格反相，交叉方向不会同时放行）"
vid_wait
adap_start
