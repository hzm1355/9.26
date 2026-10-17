#!/bin/bash
# 视频取证公共壳：起录制器（独立进程，只读 ROS 话题）+ 逐行追加字幕 + 等 MP4 落盘。
# 字幕文件 /tmp/vid_caption.txt 由驱动脚本写，录制器每帧重读，
# 所以"下发了什么指令"和"画面/面板怎么变"一定在同一帧上对得上。
VID_ROOT=/mnt/d/desktop1/9.26.1/视频取证
SIM_ROOT=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
CAP=/tmp/vid_caption.txt
# 画面源统一走 :8080 实时回显（HTTP 拉取）。直接新建 image_raw 订阅会随机 120s 才匹配上，
# 那 120s 主视区全黑，等于整段取证视频作废；回显服务常驻且早已匹配，取帧不受 DDS 发现影响。
VIEW_INTER=http:inter
VIEW_MON=http:monitor
VIEW_SPEED=http:speed
VIEW_CAR=http:car1
# 卡口相机不在回显服务里，改用插件按次落盘的真实抓拍（最新一张）
VIEW_G3='file:/tmp/v2x_gates/gate_j3_north-*.png'
VIEW_G2='file:/tmp/v2x_gates/gate_j2_south-*.png'
# 停车场：高位那一路裁 (210,300) 起 390x300 那块 —— 实测 intersection_cam 的 800x600
# 原帧里停车场正好落在里面，红白闸杆"横着→竖着"在裁切后约 3 倍放大下肉眼可辨；
# 不裁的话整根杆只有几十个像素，条目 3 的动力学就成了"只有面板数字在动"。
VIEW_PK='http:inter#crop=210:300:390:300'
VIEW_PIN='file:/tmp/v2x_parking_in/*.png'
VIEW_POUT='file:/tmp/v2x_parking_out/*.png'
T0=$(date +%s)
VID=

unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

cap() { printf '[T+%3.0fs] %s\n' "$(( $(date +%s) - T0 ))" "$*" >> "$CAP"; }

vid_start() {         # $1=输出 mp4  $2=时长 s  $3=画面话题(逗号分隔)  $4=面板模式  $5=标题
  # 清零之前先把上一段的字幕流轮转出去：/tmp/vid_caption.txt 每段 vid_start 都 : > 重写，
  # 上一段如果还没归档，文本层就永久丢了（成片里只剩位图，capq 的回包数值再也读不出来）。
  if [ -s "$CAP" ]; then
    PREV="${CAP}.$(date +%m%d_%H%M%S).prev"
    cp "$CAP" "$PREV" 2>/dev/null && echo "[vid_start] 上一段字幕流已轮转 -> $PREV"
  fi
  : > "$CAP"
  T0=$(date +%s)
  VID_OUT="$1"
  VID_DUR="$2"
  mkdir -p "$(dirname "$1")"
  cap "● 录制开始 $(date '+%F %T')"
  python3 "$TOOLS/vid_rec.py" --out "$1" --dur "$2" --fps 10 --views "$3" \
          --mode "$4" --title "$5" > /tmp/vid_rec.log 2>&1 &
  VID=$!
  sleep 2
  kill -0 "$VID" 2>/dev/null || { echo "!! 录制器没起来："; cat /tmp/vid_rec.log; exit 1; }
}

capq() {              # 执行一条命令，把输出逐行写进字幕（最多 14 行）
  eval "$1" > /tmp/capq.out 2>&1 || true
  sed '/^[[:space:]]*$/d' /tmp/capq.out | head -14 | while IFS= read -r l; do
    cap "      $l"
  done
}

pub() { timeout -k 3 20 python3 -u "$TOOLS/tl_pub.py" "$1" "$2" 3 >/dev/null 2>&1 || true; }

# 管理器每应用一次手动配时，就在 traffic_light/config_ack 上回发一条"实际生效值"。
# 后台挂一个 echo 收全流，下发后直接 tail 出来当字幕，比从灯态反推更硬。
ack_start() {   # $1=存活秒数
  : > /tmp/ack.log
  setsid nohup timeout -k 3 "$1" ros2 topic echo /traffic_light/config_ack \
      > /tmp/ack.log 2>&1 < /dev/null &
  sleep 3
}

ack_show() {    # $1=条数
  capq "grep -a applied /tmp/ack.log | tail -${1:-2} | sed 's/^ *//;s/^data: //'"
}

cfg() { pub /traffic_light/config "$1"; ack_show 1; }   # 下发手动配时并立刻回读生效值

# ---- 停车场（任务五）专用等待件 ---------------------------------------------
# 等"下一条通行记录真的落盘"，而不是等话题上冒出一条消息：DDS 发现竞态会让
# `--once` 瞬时读空（memory「DDS 计数不能当即时真相」），而 JSONL 是记录器收到就追加的，
# 行数增长同时证明"过完杆 + 记录链路通"两件事。返回时顺手把最近几条通行记录写成字幕。
PKJ=/tmp/v2x_parking/parking_pass.jsonl
PKA=/tmp/v2x_parking/barrier_actions.jsonl
PKCLI="python3 $TOOLS/parking_cli.py"

pk_wait_pass() {      # $1=超时 s（默认 240，够 car_6 跑完一整个进出循环）
  local n0 n1 t=0 lim=${1:-240}
  n0=$(wc -l < $PKJ 2>/dev/null || echo 0)
  while [ "$t" -lt "$lim" ]; do
    n1=$(wc -l < $PKJ 2>/dev/null || echo 0)
    [ "${n1:-0}" -gt "${n0:-0}" ] && break
    sleep 4
    t=$((t + 4))
  done
  cap "      等了 ${t}s：通行记录 ${n0:-0} → ${n1:-0} 行（JSONL 在长 = 这一次真过完了杆）"
  capq "$PKCLI query --limit 3"
}

# 解除管制事件：连发两次并留足间隔。state_detail 只在"相位变化或 force_republish 到期"时
# 才重播，解除指令下发后若立刻读回，看到的还是管制期间最后一条广播，会把"已恢复"演成"没恢复"。
rel() { pub "$1" "$2"; sleep 6; pub "$1" "$2"; sleep 10; }

# 只重启信号灯管理器进程（不动 signal_drive / 车辆），用来让 offset 重新播种。
# 管理器的相位状态机只在"首次播种"和"时钟回退"时按 offset 对齐主时钟网格，
# 之后自由推进；自适应长期拉伸各组周期会把配好的相位差冲散，改 offset 也不会立刻见效。
TL_PARAMS=$SIM_ROOT/src/v2x_demo/config/traffic_light_params.yaml
mgr_restart() {
  for p in $(pgrep -f 'traffic_light_manager[.]py' || true); do kill -9 "$p" 2>/dev/null || true; done
  sleep 2
  setsid nohup ros2 run v2x_demo traffic_light_manager.py --ros-args \
    -p use_sim_time:=true -p enable_statistics:=true -p enable_time_schedule:=true \
    -p config_file:="$TL_PARAMS" > /tmp/tlmgr_vid.log 2>&1 < /dev/null &
  sleep 14
  cap "   管理器已重启（PID $(pgrep -f 'traffic_light_manager[.]py' | tr '\n' ' ')），六组相位按 YAML offset 重新播种"
}

# ---- 外部自适应算法节点的摘除 / 拉起 ----------------------------------------
# 自适应节点会持续把各组绿、红时长乘以自己算出的系数（面板 mode=ADAPTIVE），
# 录"手动配时"和"事件触发"时必须先摘掉它，否则读回的时长是被拉伸过的、对不上设定值。
# direction_zones 是 launch 里算出来的进口道矩形，不带 --params-file 重启会拿到空分区、
# 系数恒为 1.0，那段画面就成了假证据 —— 所以摘之前先把参数 dump 到仓库里存着。
ADAP_YAML=$TOOLS/adaptive_params_vid.yaml

adap_stop() {
  if pgrep -f 'adaptive[_]timing_node' >/dev/null 2>&1; then
    timeout -k 3 25 ros2 param dump /adaptive_timing_node > /tmp/adap_dump.yaml 2>/dev/null
    if [ -s /tmp/adap_dump.yaml ] && grep -q direction_zones /tmp/adap_dump.yaml; then
      cp /tmp/adap_dump.yaml "$ADAP_YAML"
      cap "   自适应参数已快照到 tools/adaptive_params_vid.yaml（含 direction_zones）"
    else
      cap "!! dump 到的参数里没有 direction_zones，沿用已存的快照文件"
    fi
    for p in $(pgrep -f 'adaptive[_]timing_node'); do kill -9 "$p" 2>/dev/null || true; done
    sleep 2
  fi
  for g in J1_ns J1_ew J2_ns J2_ew J3_ns J3_ew; do
    pub "/traffic_light/$g/adaptive_factor" 1.0
  done
  cap "   自适应节点实例数=$(pgrep -cf 'adaptive[_]timing_node' || true)，6 组系数已手动复位为 1.0"
}

adap_start() {
  [ -s "$ADAP_YAML" ] || { cap "!! 缺少 $ADAP_YAML，重启的自适应节点会因空分区而系数恒 1.0"; return 1; }
  for p in $(pgrep -f 'adaptive[_]timing_node'); do kill -9 "$p" 2>/dev/null || true; done
  sleep 1
  setsid nohup ros2 run v2x_demo adaptive_timing_node.py --ros-args \
    --params-file "$ADAP_YAML" > /tmp/adaptive_vid.log 2>&1 < /dev/null &
  sleep 12
  cap "   自适应节点已拉起：实例 $(pgrep -cf 'adaptive[_]timing_node' || true) 个（PID $(pgrep -f 'adaptive[_]timing_node' | tr '\n' ' ')）"
}

vid_wait() {
  cap "● 录制结束 $(date '+%F %T')"
  wait "$VID"
  tail -2 /tmp/vid_rec.log
  # 录制器是独立进程，它半路崩了驱动脚本只会"正常跑完"——实测有一次 240 s 的片只剩 12 s，
  # 退出码还是 0。所以收尾立刻体检一遍，帧数不够当场就报，别等归档后才发现是废片。
  python3 "$TOOLS/vid_check.py" "$VID_OUT" 2>&1 | tail -2
}
