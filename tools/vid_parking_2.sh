#!/bin/bash
# 任务五 条目2 视频取证：停车场卡口相机输出图像 ROS2 话题 → 对接车牌识别算法节点 →
# 把识别结果与通行时间记进档案。
#
# 这一段要做两件别处做不到的事：
#   · 三方对账（pk_show_capture.py）：停车场通行记录、卡口识别记录、抓拍原图三者用
#     同一个 capture_id 串起来。只看"记录里有车牌号"是不行的 —— 那可能是节点自己填的。
#   · 真把内置识别器摘掉、只让外部算法桩在环，让一次**真实进场**由外部算法放行。
#     桩回的是假号"外部算法A1"，白名单里没有，于是 manager 按 yaml 的
#     unknown_as_temp=true 走"未登记→发临时卡→放行"：这同时证明下游认的是
#     PlateResult 这条消息、不是某个算法 —— 换算法零改动。
# 代价：这一轮进场的车牌会显示成"外部算法A1/临时车"而不是"沪A12345/月租"，
# 所以桩跑完必须 FORCE 拉回内置识别器，条目 3/4 的取证要用回真号。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 中途退出会把字幕停在半路，整段作废；宁可少一步操作
SIM=$SIM_ROOT     # vid_common.sh 只定义了 SIM_ROOT，$SIM 不补会展开成 /src/...
OUT="$VID_ROOT/任务五_停车场闸机/条目2_图像话题与车牌识别算法对接_三方对账.mp4"
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml

vid_start "$OUT" 400 "$VIEW_PK,$VIEW_PIN" parking \
  "任务五 条目2  卡口相机图像话题 + 对接车牌识别算法节点 + 记录通行时间"
cap "上格：intersection_cam 裁出的停车场（下格才是相机视角，注意区分）"
cap "下格：入口卡口相机按次落盘的实拍抓拍，最新一张不断刷新"
sleep 6
cap "== 0 卡口记录器必须在环：识别侧的算法名/真实牌只落在它那份档案里 =="
capq "bash $SIM_ROOT/tools/ensure_recorder.sh 2>&1 | tail -2"
cap "== 2a 虚拟摄像头：两路卡口的 image_raw 话题（类型 / QoS / 帧率）=="
capq "timeout -k 3 25 ros2 topic info -v /parking/gate_in/image_raw | sed -n '1,3p;7,12p'"
capq "timeout -k 3 20 ros2 topic hz /parking/gate_in/image_raw --window 20 2>/dev/null | tail -2"
cap "  ↑ 配置项是 1920x1080 / update_rate 10 Hz（parking_zones.yaml 的 camera 段）；"
cap "    实测帧率约 4.7 Hz，是整机渲染负载把传感器回调拖下来（RTF<1），不是配置没生效 —— "
cap "    抓拍本身是帧精确触发 + frame_wait 3 s 内取最新帧，核验位是停稳点，不受帧率影响"
capq "timeout -k 3 15 ros2 topic info /parking/gate_out/image_raw | sed -n '1,4p'"
sleep 4
cap "== 2b 识别链路的两个接口：抓拍事件进、识别结果出（都是 v2x_msgs）=="
capq "timeout -k 3 15 ros2 topic info /plate/capture | sed -n '1,4p'"
capq "timeout -k 3 15 ros2 topic info /plate/result | sed -n '1,4p'"
cap "  /plate/capture 发布者=卡口 SensorPlugin（高速 2 路 + 停车场 2 路共用这一条协议）"
cap "  /plate/result  发布者=识别算法节点；订阅者=内置记录器 + 停车场管控 + 看板"
capq "timeout -k 3 30 ros2 node info /parking_manager 2>/dev/null | sed -n '/Subscribers:/,/Publishers:/p' | head -14"
sleep 4
cap "== 2c 接口约定本体：PlateResult 的字段就是外部算法要填的全部 =="
capq "head -16 $SIM/src/v2x_msgs/msg/PlateResult.msg"
sleep 4
cap "== 2d 内置识别器在环时的一轮真实对账（通行记录 ↔ 识别记录 ↔ 抓拍原图）=="
capq "python3 $TOOLS/pk_show_capture.py --n 2 2>&1"
sleep 8
cap "  ↑ 算法名 builtin_opencv：号来自那张实拍图上的牌；四段时间就是'记录通行时间'"
cap "== 2e 现在把内置识别器停掉，轮询到 /plate/result 发布者归零 =="
for i in 1 2 3 4 5 6 7 8; do
  ps=$(pgrep -f 'plate[_]recognizer_node' || true)
  [ -z "$ps" ] && break
  for p in $ps; do kill -9 "$p" 2>/dev/null || true; done
  sleep 1
done
# kill -9 后 graph 里的 publisher 记录要等 DDS 租约到期才消失（实测几十秒），
# 问一次就当"归零"是假证据，所以轮询到真归零并把用时打进字幕。
PB_T0=$(date +%s)
for i in $(seq 1 16); do
  n=$(timeout -k 3 10 ros2 topic info /plate/result 2>/dev/null | tr -d '\r' |
        awk '/Publisher count/{print $3}')
  if [ "${n:-1}" = "0" ]; then
    cap "  第 $i 次读回：Publisher count=0，归零用时 $(( $(date +%s) - PB_T0 )) s"
    break
  fi
  [ $((i % 4)) -eq 0 ] && cap "  第 $i 次读回：Publisher count=${n:-?}，租约未到点，继续等"
  sleep 4
done
cap "  ↑ 归零之后 /plate/result 上的号只可能来自外部节点，这条判据才立得住"
cap "== 2f 只挂外部算法桩（订阅 PlateCapture + 图像话题，回 PlateResult），等一轮真实进场 =="
timeout -k 3 240 python3 -u "$TOOLS/external_algo_demo.py" 200 > /tmp/vid_pk2.out 2>&1 &
EP=$!
cap "  桩在环……每 45 s 报一次桩的动静与摆渡车进度（桩回假号'外部算法A1'，见 2h）"
for i in 1 2 3 4; do
  sleep 45
  capq "tail -3 /tmp/vid_pk2.out"
  capq "timeout -k 3 12 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>/dev/null | head -2"
done
cap "== 2g 等桩退出，看它一共替下游回答了几次 =="
capq "wait $EP; tail -6 /tmp/vid_pk2.out"
cap "== 2h 三方对账：这一轮的号必须由外部算法给出 =="
capq "python3 $TOOLS/pk_show_capture.py --n 3 2>&1"
sleep 6
cap "  ↑ 算法=external_stub_v1 的那条：进场由外部算法放行，牌='外部算法A1'（桩的假号），"
cap "    类型=临时车、带卡号 —— 这就是 unknown_as_temp=true 的分支，下游零改动"
cap "== 2i 拉回内置识别器，并用'又有识别结果回来'复核生效（不是只看进程活着）=="
: > /tmp/pk2_after.log
setsid nohup timeout -k 3 90 ros2 topic echo /plate/result v2x_msgs/msg/PlateResult \
    > /tmp/pk2_after.log 2>&1 < /dev/null &
sleep 2
capq "FORCE=1 bash $SIM_ROOT/tools/ensure_recognizer.sh 2>&1 | tail -4"
sleep 50
capq "timeout -k 3 15 ros2 topic info /plate/result | sed -n '1,4p'"
cap "  ↓ 恢复后 /plate/result 上真的又出现结果行（含高速卡口的），才算识别链路回位"
capq "echo 'echo 侧收到行数='$(wc -l < /tmp/pk2_after.log)"
capq "head -13 /tmp/pk2_after.log"
vid_wait
