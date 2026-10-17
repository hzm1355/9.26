#!/bin/bash
# 任务三 条目3 视频取证：车辆高速通过卡口的识别效果 + Gazebo 环境光照（逆光/弱光/夜间）。
#
# 这一段为什么这么排：
#   · 高速档必须自带"本次窗口"的明细（SWEEP_CSV 指到独立文件），否则字幕里的命中率
#     混着上一轮同一 label 的旧抓拍，画面和数字对不上；
#   · 高速真正会坏的是两件事——漏拍/牌面未定位 与 字符读错，所以要出 sweep_summary
#     的失败模式拆分表，而不是只报一个百分比；
#   · 实测速度 < 0.2 m/s 的抓拍是车停在触发区附近（摆车起点导致的静止样本），
#     不能进"高速通过率"的分母，表里单列成"静止"；
#   · 光照档跑完要用**一次真实抓拍的全图均值**确认档位真的回到 day：只看话题回读
#     在实测里连发 25 次仍停在 night，会把后续条目的画面和识别全污染。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务三_卡口车牌识别/条目3_高速通过识别率与逆光弱光夜间光照.mp4"
SWEEP=/tmp/v2x_records/vid_g3_sweep.csv
G3=$VIEW_G3
G2=$VIEW_G2

vid_start "$OUT" 520 "$VIEW_INTER,$G3,$G2" plate \
  "任务三 条目3  高速通过卡口识别效果 + 逆光/弱光/夜间光照条件测试"
cap "右栏 车速 列 = 抓拍瞬间线速度；判定 列 = 识别值与真值是否逐字一致（空号=没读出字符）"
capq "bash $SIM_ROOT/tools/ensure_recorder.sh | tail -2"
sleep 6
cap "== 3a 高速通过：cruise_speed 改到 6.0 m/s，按速度放大跟车距重启驱动，采样 140 s =="
capq "grep -ac 'cruise_speed: 0.7' ~/v2x_ws/src/v2x_demo/config/signal_drive_params.yaml; echo '^ 起跑前 0.7 m/s 的车辆数'"
rm -f $SWEEP
timeout -k 3 300 env SWEEP_CSV=$SWEEP bash "$SIM_ROOT/tools/speed_sweep.sh" 6.0 140 > /tmp/vid_g3_speed.log 2>&1 &
SP=$!
sleep 24
cap "  预检已下发五副默认牌并回读材质；cruise_speed 现为 6.0（5 辆全改写），follow_gap=2.6+0.7*6=6.8 m"
sleep 42
cap "  6 m/s 档采样中……注意车队过龙门架的通过速度明显变快"
sleep 42
cap "  采样中……上下两屏是两个卡口相机在车辆驶入触发区时按次落盘的实拍帧"
sleep 42
cap "  采样即将结束，随后 speed_sweep 把 cruise_speed 恢复成 0.7 m/s 并重启驱动"
sleep 46
capq "wait $SP; grep -aE '===|drive:|窗口 ' /tmp/vid_g3_speed.log | tail -4"
cap "== 3a 明细：命中率按有效样本（剔除静止抓拍），未命中再拆牌面未定位 / 读出错字 =="
capq "grep -aA6 '窗口 140 s' /tmp/vid_g3_speed.log | tail -8"
capq "python3 -u $TOOLS/sweep_summary.py $SWEEP | sed -n '1,2p;4,10p'"
sleep 6
cap "== 3b 环境光照对照：逐档下发 backlit/lowlight/night，每档等到一张新抓拍并量画面亮度 =="
cap "  全图均值证明预设改到了相机那条渲染线；牌面均值决定这一档算不算拍得下来"
timeout -k 3 280 python3 -u "$TOOLS/gate_lighting_test.py" backlit lowlight night 60 > /tmp/vid_g3_light.log 2>&1 &
LP=$!
sleep 24
cap "  逆光档 backlit：画面整体发白、牌面反差被压"
sleep 52
cap "  弱光档 lowlight：环境光整体压低，看车牌补光顶不顶得住"
sleep 52
cap "  夜间档 night：全图必然偏黑，判据是牌面均值有没有被补光打起来"
sleep 58
capq "wait $LP; tail -7 /tmp/vid_g3_light.log"
sleep 4
cap "== 3c 恢复默认光照：连发 day 并回读，再用一次真实抓拍的全图均值确认生效 =="
capq "bash $SIM_ROOT/tools/set_lighting.sh day | tail -2"
timeout -k 3 110 python3 -u "$TOOLS/gate_lighting_test.py" day 50 > /tmp/vid_g3_day.log 2>&1 &
DP=$!
sleep 30
cap "  白天档复采中……这一行全图均值应与 3b 各档同量级或更高"
sleep 42
capq "wait $DP; tail -3 /tmp/vid_g3_day.log"
capq "timeout -k 3 15 ros2 topic echo /gate/lighting/current --once 2>/dev/null | tr -d '\\r' | grep -a data"
vid_wait
echo "--- 录制结束，恢复默认牌面（自发光）---"
bash "$SIM_ROOT/tools/apply_default_plates.sh" 2>&1 | tail -2
echo DONE
