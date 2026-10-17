#!/bin/bash
# 任务四录屏预检：开录前把"能不能录"一次性判掉，别录到一半发现画面黑或全场零事件。
# 判据都是实测踩过的坑：:8080 四路回显必须 200（新建 image_raw 订阅会随机 120 s 才匹配）；
# 同名节点双实例会抢话题、把两轮标签搅混；检测节点日志里近几分钟还有"超物理上限/复位"
# 就说明车队还在复位风暴里，这时录出来的均速是 7~12 km/h 的废读数，不是 2.52 的巡航真值。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
FAIL=0

echo "--- 1) :8080 四路回显（画面源）---"
for n in car1 speed inter monitor; do
  printf '  %-8s ' "$n"
  curl -s -o /dev/null -w 'HTTP %{http_code}  %{size_download} B\n' --max-time 8 \
      http://localhost:8080/cam/$n
done

echo "--- 2) 进程实例数（每项应为 1 个真正的 python 实例）---"
for PAT in 'gzserv[e]r' 'lib/v2x[_]demo/speed_detect_node' 'lib/v2x[_]demo/speed_recorder_node' \
           'lib/v2x[_]demo/signal_drive_node' 'lib/v2x[_]demo/traffic_light_manager' \
           'multi_view[_]server' 'live_view[_]server'; do
  N=$(pgrep -cf "$PAT" 2>/dev/null || echo 0)
  printf '  %-34s 实例 %s\n' "$PAT" "$N"
  [ "$N" = "1" ] || { echo "     !! 不是恰好 1 个"; FAIL=1; }
done

echo "--- 3) 在册测速点位 ---"
python3 $TOOLS/speed_zones_cli.py list | head -2
python3 $TOOLS/speed_zones_cli.py list | grep -ac '·' | sed 's/^/  点位数 /'

echo "--- 4) 灯态在轮转（管理器不在→灯停→车卡死→复位风暴，见 start_tlmgr.sh 说明）---"
# 采样要重试：`--once` 撞上 DDS 发现竞态会回 "does not appear to be published yet"，
# 空回包不等于灯没动（实测整栈重启后头几分钟就误报过一次）。
for i in 1 2 3 4; do
  L=$(timeout -k 3 15 ros2 topic echo /traffic_light/J1_ns/state_detail --qos-profile sensor_data \
        --no-daemon --once 2>&1 | tr -d '\r' | grep -aE 'vehicle_phase|phase_remaining|control_mode')
  [ -n "$L" ] && { echo "$L" | sed 's/^/  /'; break; }
  echo "  第 $i 次采样空回包（发现竞态），重试"; sleep 12
done
[ -n "$L" ] || { echo "     !! 四次都读不到灯态，管理器可能没在广播"; FAIL=1; }
echo "--- 5) 检测节点近 10 分钟的护栏告警（有 = 车队还在复位风暴，先别录）---"
NOW=$(date +%s)
grep -aE '超物理上限|瞬移' /tmp/speed_detect_node.log 2>/dev/null | tail -3 | cut -c1-120
CNT=$(awk -v now=$NOW '/超物理上限|瞬移/ {t=$2; gsub(/[\[\]]/,"",t); if (t+0 > now-600) c++} END {print c+0}' \
      /tmp/speed_detect_node.log 2>/dev/null)
echo "  近 600 s 内护栏行数：${CNT:-0}"
[ "${CNT:-0}" = "0" ] || { echo "     !! 车队仍在被复位，先跑 tools/recover_fleet.sh"; FAIL=1; }

echo "--- 6) 车队是否在巡航（20 s 位移；红灯排队停在 0 属正常，但要有多辆车在前进）---"
timeout -k 5 60 python3 -u $TOOLS/fleet_peek.py 20 2>&1 | tail -7

echo "--- 7) 仿真时钟与 RTF ---"
timeout -k 3 12 ros2 topic echo /clock --qos-profile sensor_data --no-daemon --once 2>&1 | tr -d '\r' | grep -aE 'sec|nanosec'

echo "--- 8) 落盘在增长（效果复核：进程数=1 不等于事件在出）---"
# 整栈重启被 start_signal.sh 的宽 pkill 咬掉检测节点那次，七项全绿但 /speed/zones 服务
# 不可用、JSONL 一行不长 —— 录 5 段才回过味来。这一项用"被影响对象的实测输出"判活。
B1=$(wc -l < /tmp/v2x_records/speed_captures.jsonl 2>/dev/null || echo 0)
sleep 60
B2=$(wc -l < /tmp/v2x_records/speed_captures.jsonl 2>/dev/null || echo 0)
echo "  60 s 内 speed_captures.jsonl：$B1 -> $B2 行（增量 $(( B2 - B1 ))）"
[ $(( B2 - B1 )) -gt 0 ] || { echo "     !! 没有新抓拍，检测节点或车队没经过点位，别开录"; FAIL=1; }

echo
[ $FAIL = 0 ] && echo "预检通过，可以开录" || echo "!! 预检有异常，先处理再录"
exit $FAIL
