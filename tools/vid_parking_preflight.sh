#!/bin/bash
# 任务五录屏预检：开录前把"能不能录"一次判掉。
#
# 每一条都对应实测踩过的坑（同 vid_speed_preflight.sh 的口径）：
#  * :8080 的 inter 路必须回 200 —— 停车场那一路画面是它的裁切，回显服务挂了整段就是黑屏；
#  * 同名节点双实例会让同一条 ParkingPass 落两行 JSONL，条目 5 的"命中 N 条"直接翻倍；
#  * 上场车（car_3 / car_2）不在运行期 world 里时摆渡节点不报错、只是不动，
#    于是"面板在走、画面空的"；
#  * BarrierState 读不到 = 道闸插件没加载，抬落杆全无实测值，条目 3 无从谈起；
#  * 最后一项用"JSONL 在不在长"判活：进程数=1、话题有发布者都不等于车真的过杆了。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
FAIL=0

echo "--- 1) :8080 回显（停车场实况那一路是 inter 的裁切）---"
for n in inter monitor; do
  printf '  %-8s ' "$n"
  curl -s -o /dev/null -w 'HTTP %{http_code}  %{size_download} B\n' --max-time 8 \
      http://localhost:8080/cam/$n
done

echo "--- 2) 进程实例数（每项应为 1）---"
for PAT in 'gzserv[e]r' 'lib/v2x[_]demo/parking[_]manager[_]node' \
           'lib/v2x[_]demo/parking[_]recorder[_]node' \
           'lib/v2x[_]demo/parking[_]shuttle[_]node' \
           'lib/v2x[_]demo/plate[_]gen[_]node' \
           'lib/v2x[_]demo/plate[_]recognizer[_]node' \
           'multi[_]view[_]server' 'live[_]view[_]server'; do
  N=$(pgrep -cf "$PAT")
  printf '  %-42s 实例 %s\n' "$PAT" "${N:-0}"
  [ "$N" = "1" ] || { echo "     !! 不是恰好 1 个"; FAIL=1; }
done

echo "--- 3) 上场摆渡车在不在运行期 world 里 ---"
# 分队后上场的是 car_3（白名单月租）与 car_2（临时车登记）；car_6 退役但模型仍在
# world 里（出生位挪到沙盘东南角），不给它起节点，所以这里只查两辆在用的。
for M in car_3 car_2; do
  C6=$(gz model -m $M -p 2>&1 | head -2)
  if echo "$C6" | grep -aq 'Unable'; then
    echo "  !! world 里没有 $M：$(echo "$C6" | tr -d '\n' | cut -c1-70)"
    echo "     运行期 gzserver 比 world 文件旧，必须整栈重启（tools/restart_clean_stack.sh）"
    FAIL=1
  else
    echo "  $M 位姿 $C6"
  fi
done

echo "--- 4) 两道闸的实测状态（BarrierState 要能读到、fault 应为 0）---"
# 采样要重试：刚 launch 的节点 DDS 发现没完成，--once 会瞬时读空（实测误报过）。
for i in 1 2 3 4; do
  GS=$(timeout -k 3 20 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 | tr -d '\r')
  echo "$GS" | grep -aq 'gate_id' && break
  echo "  第 $i 次采样空回包（发现竞态），重试"; sleep 10
done
echo "$GS" | grep -aE 'gate_id|state:|target_rad|angle_rad|fault|loop_occupied|raise_count' \
  | head -16 | sed 's/^ */  /'
echo "$GS" | grep -aq 'gate_id' || { echo "     !! 四次都读不到 BarrierState"; FAIL=1; }
NGATE=$(echo "$GS" | grep -ac 'gate_id: gate_')
echo "  本轮读到 $NGATE 条（两道闸各自广播，读到 1 条也算通，只要 gate_id 覆盖到两道就好）"

echo "--- 5) 四个服务 ---"
SV=$(ros2 service list 2>/dev/null | grep -aE '^/parking/(gate|whitelist|query|export)$')
echo "${SV:-  (一个都没有)}"
[ "$(echo "$SV" | grep -ac .)" = 4 ] || { echo "     !! 服务不全"; FAIL=1; }
echo "  gate list 回包：$(timeout -k 3 25 python3 $TOOLS/parking_cli.py gate list 2>&1 | head -1)"

echo "--- 6) 卡口相机出帧（1080p 渲染要几秒才稳，窗口给足）---"
for T in /parking/gate_in/image_raw /parking/gate_out/image_raw; do
  printf '  %-30s ' "$T"
  timeout -k 3 25 ros2 topic hz --window 10 $T 2>&1 | grep -a 'average rate' | head -1 \
    || echo "15 s 内没帧"
done

echo "--- 7) 识别链路（抓拍与结果两路都要有发布者）---"
for T in /plate/capture /plate/result; do
  printf '  %-16s ' "$T"
  timeout -k 2 12 ros2 topic info $T 2>&1 | tr -d '\r' | grep -a 'Publisher count' || echo '读不到'
done

echo "--- 8) 落盘在增长（效果复核：真过杆一次才算通）---"
P1=$(wc -l < /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null)
A1=$(wc -l < /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null)
echo "  当前：通行 ${P1:-0} 行 / 动作 ${A1:-0} 行，等 150 s（摆渡车一轮周期 90 s + 余量）"
sleep 150
P2=$(wc -l < /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null)
A2=$(wc -l < /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null)
echo "  之后：通行 ${P2:-0} 行 / 动作 ${A2:-0} 行（增量 $(( ${P2:-0} - ${P1:-0} )) / $(( ${A2:-0} - ${A1:-0} ))）"
[ $(( ${P2:-0} - ${P1:-0} )) -gt 0 ] || { echo "     !! 150 s 内没有新通行记录：车没过杆，别开录"; FAIL=1; }

echo "--- 9) 仿真时钟与 RTF ---"
timeout -k 3 12 ros2 topic echo /clock --qos-profile sensor_data --no-daemon --once 2>&1 \
  | tr -d '\r' | grep -aE 'sec|nanosec' | sed 's/^/  /'

echo
[ $FAIL = 0 ] && echo "预检通过，可以开录" || echo "!! 预检有异常，先处理再录"
exit $FAIL
