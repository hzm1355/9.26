#!/bin/bash
# 任务三 条目2 视频取证：卡口相机输出图像 ROS2 话题 + 对接外部车牌识别算法节点。
# 做法：把内置识别器停到 /plate/result 发布者归零，只跑外部桩算法节点，
# 让它订阅图像话题做定位/识别并回发 PlateResult，最后再拉回内置识别器，
# 导出记录里 external_stub_v1 与 builtin_opencv 两类并存 —— 说明下游零改动即可换算法。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务三_卡口车牌识别/条目2_图像ROS2话题与外部识别算法节点对接.mp4"
G3=$VIEW_G3
G2=$VIEW_G2

vid_start "$OUT" 320 "$VIEW_INTER,$G3,$G2" plate \
  "任务三 条目2  摄像头图像 ROS2 话题 + 外部车牌识别算法节点对接"
cap "== 0 确保记录器在线：条目2 要停/起识别器，记录器必须全程不动 =="
capq "bash $SIM_ROOT/tools/ensure_recorder.sh | tail -2"
cap "== 2a 图像话题：两路卡口相机的 image_raw（类型 / 可靠性 / 帧率）=="
capq "timeout -k 3 25 ros2 topic info -v /gate_j3_north/gate_cam/image_raw | sed -n '1,3p;7,12p'"
capq "timeout -k 3 20 ros2 topic hz /gate_j3_north/gate_cam/image_raw --window 20 2>/dev/null | tail -2"
capq "timeout -k 3 15 ros2 topic info /plate/result | sed -n '1,4p'"
cap "  识别结果话题 /plate/result（v2x_msgs/PlateResult）：车牌号 / 抓拍时刻 / 车辆位姿三项齐全"
sleep 6
cap "== 2b 停掉内置识别器，轮询到 /plate/result 发布者归零 =="
for i in 1 2 3 4 5 6 7 8; do
  ps=$(pgrep -f 'plate[_]recognizer_node' || true)
  [ -z "$ps" ] && break
  for p in $ps; do kill -9 "$p" 2>/dev/null || true; done
  sleep 1
done
# kill -9 之后 ROS graph 里那条 publisher 记录不会立刻消失（要等 DDS 租约到期，实测几十秒），
# 问一次就拿去写"归零"是假的，所以轮询到真归零为止，并把用时一并打进字幕。
PB_T0=$(date +%s)
for i in $(seq 1 14); do
  n=$(timeout -k 3 10 ros2 topic info /plate/result 2>/dev/null | tr -d '\r' |
        awk '/Publisher count/{print $3}')
  if [ "${n:-1}" = "0" ]; then
    cap "  第 $i 次读回：Publisher count=0，归零用时 $(( $(date +%s) - PB_T0 )) s（kill -9 后要等 DDS 租约到期）"
    break
  fi
  [ $((i % 4)) -eq 0 ] && cap "  第 $i 次读回：Publisher count=${n:-?}，租约未到点，继续等"
  sleep 4
done
cap "  ↑ 归零 = 内置识别器确实摘干净了，接下来 /plate/result 上的号只可能来自外部算法"
sleep 4
cap "== 2c 只跑外部算法桩节点（订阅图像话题做定位+字符识别），持续 150 s =="
timeout -k 3 180 python3 -u "$TOOLS/external_algo_demo.py" 150 > /tmp/vid_g2.out 2>&1 &
EP=$!
sleep 72
cap "  外部算法在环……右栏结果行的 algorithm 字段此时应为 external_stub_v1"
sleep 72
capq "tail -8 /tmp/vid_g2.out"
cap "== 2d 等外部桩退出，再 FORCE 拉回内置识别器（否则 ensure 会把桩当在线而不重启，桩一退链路就断）=="
capq "wait $EP; tail -4 /tmp/vid_g2.out"
capq "FORCE=1 bash $SIM_ROOT/tools/ensure_recognizer.sh | tail -4"
sleep 12
capq "timeout -k 3 45 ros2 service call /plate/export std_srvs/srv/Trigger 2>&1 | tr -d '\\r' | tail -2"
cap "  ↓ 只数本次录制开始（$(date -d @$T0 '+%F %T')）之后的桩条目，与桩自己报的输出条数对齐"
capq "timeout -k 3 30 python3 -u $TOOLS/show_external_records.py /tmp/v2x_records/plate_records.csv \"$(date -d @$T0 '+%F %T')\" 2>/dev/null | tail -10"
vid_wait
