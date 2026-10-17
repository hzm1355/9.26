#!/bin/bash
# 任务三 条目5 视频取证：黑名单车牌设置并触发告警（两阶段分级版）。
#
# 三代取证脚本的演化，全是被真实流量打的脸：
#   1) blacklist_targeted_test.py：换两块牌挂到"历史取景最稳的车"上——撞上 car_4 在 J2
#      十张里九张空号，200 s 零告警；
#   2) blacklist_live_test.py：路面五副牌整批入黑、按号码预分两级——连着两轮（240 s /
#      250 s）都只有 car_3 的京B88990 被读出来，CRITICAL 押在沪A/浙A 上就是等不到，
#      片内只剩 WARN 一档；
#   3) blacklist_two_phase.py：拿"当前真被读出来的那块牌"先 WARN 后 CRITICAL——WARN 66 s
#      就落地，但同一辆车再次过卡口要 100~150 s（实测 5 分钟 12 条记录、单车重复 2~3 次），
#      130 s 的单档超时差一口气，CRITICAL 又没演上；
#   4) 现在这版 blacklist_flip_test.py：**级别挂在名单条目上而不是号码上**，所以既不必凑两辆
#      都能被读出的车，也不必等同一辆车跑两趟。整张名单先按 WARN 入黑，任意一辆车被读出号码
#      就落一条 WARN；收到后立刻把整张名单改成还缺的那一档（op=0 原地覆盖），下一辆车过卡口
#      就落 CRITICAL。实测 2 s 落 WARN、39 s 落 CRITICAL，两级齐。
# 顺带演全 op=0 增/改、op=2 清空、名单热更新对下一条识别结果立即生效、alert_level 写回导出 CSV。
# 车队体检也换了判据：卡死时驱动节点会把某两辆车反复 teleport 回复位点，10 s 位移能到
# 30 m，按"最大位移"永远判成在动——现在按"几辆车在动"判，少于三辆就复位。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务三_卡口车牌识别/条目5_黑名单车牌设置与告警触发.mp4"
G3=$VIEW_G3
G2=$VIEW_G2

vid_start "$OUT" 130 "$VIEW_INTER,$G3,$G2" plate \
  "任务三 条目5  黑名单车牌设置与告警触发（WARN / CRITICAL 两级）"
cap "名单接口：/plate/blacklist 服务（op=0 增或改 / 1 删 / 2 清空 / 3 查询），命中后广播 /plate/alert"
cap "== 0a 记录窗口清零：重启记录器并让开旧 JSONL，导出 CSV 里只剩本片内产生的记录 =="
capq 'mv /tmp/v2x_records/plate_records.jsonl /tmp/v2x_records/plate_records_prev.jsonl 2>/dev/null; for p in $(pgrep -f "lib/v2x[_]demo/plate[_]recorder[_]node"); do kill -9 $p; done; sleep 3; bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/ensure_recorder.sh | tail -2'
sleep 4
cap "== 0b 车队体检：卡口取证要有车过，五车长跑会互锁停摆（本轮实测 60 s 零抓拍）=="
capq "bash $SIM_ROOT/tools/fleet_guard.sh | tail -8"
sleep 2
cap "== 5a 当前黑名单条目（应为空，测试收尾会清空）=="
capq "timeout -k 3 25 ros2 service call /plate/blacklist v2x_msgs/srv/SetBlacklist '{op: 3}' 2>&1 | tr -d '\\r' | tail -2"
sleep 3
cap "== 5b 整张名单先按 WARN 入黑；任意一辆车被读出号码就落一条告警 =="
capq "timeout -k 3 40 ros2 service call /plate/blacklist v2x_msgs/srv/SetBlacklist '{op: 3}' 2>&1 | tr -d '\\r' | tail -1"
timeout -k 3 90 python3 -u "$TOOLS/blacklist_flip_test.py" 55 30 > /tmp/vid_g5.out 2>&1 &
BP=$!
SENT=0
ELAPSED=0
while kill -0 "$BP" 2>/dev/null && [ "$ELAPSED" -lt 70 ]; do
  sleep 20
  ELAPSED=$((ELAPSED + 20))
  N=$(wc -l < /tmp/vid_g5.out 2>/dev/null || echo 0)
  if [ "$N" -gt "$SENT" ]; then
    capq "sed -n '$((SENT + 1)),${N}p' /tmp/vid_g5.out"
    SENT=$N
  fi
done
# 只补流式轮询没来得及送出的尾巴；整份重打会把同一批告警在字幕里演成"又来了一次"
capq "wait \$BP; sed -n '$((SENT + 1)),\$p' /tmp/vid_g5.out"
cap "== 5c 告警写回识别记录：导出 CSV 的 alert_level 列 =="
capq "timeout -k 3 60 python3 -u $TOOLS/check_export_csv.py 2>&1 | grep -aE '行数|alert|告警' | tail -4"
vid_wait
