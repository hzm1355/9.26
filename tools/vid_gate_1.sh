#!/bin/bash
# 任务三 条目1 视频取证：卡口虚拟摄像头（电子警察抓拍）+ 可自定义虚拟车牌号码。
# 上下两屏分别是北环 gate_j3_north 与南环 gate_j2_south 的卡口相机实时画面，
# 右栏是 /plate/capture 与 /plate/result 的实时计数和最近十条识别结果。
# 动作：给 car_1 挂一个本时线从未出现过的号码 渝D7K418，等它过卡口被拍到并逐字读回。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务三_卡口车牌识别/条目1_卡口电子警察抓拍与自定义虚拟车牌.mp4"
G3=$VIEW_G3
G2=$VIEW_G2

vid_start "$OUT" 265 "$VIEW_INTER,$G3,$G2" plate \
  "任务三 条目1  卡口虚拟摄像头电子警察抓拍 + 自定义虚拟车牌号码"
cap "上屏=北环 gate_j3_north  下屏=南环 gate_j2_south（龙门架枪机，车辆驶入触发区即抓拍）"
cap "抓拍链路：v2x_gate_plugin -> /plate/capture -> plate_recognizer_node -> /plate/result"
cap "== 0 确保记录器在线：stop_signal.sh 会把 plate_recorder_node 一起带走，缺它 JSONL 不再增长 =="
capq "bash $SIM_ROOT/tools/ensure_recorder.sh | tail -2"
sleep 8
cap "== 1a 自定义车牌号码：向 /plate/request 下发 car_1|rear|渝D7K418 =="
capq "timeout -k 3 40 python3 -u $TOOLS/plate_request_cli.py 'car_1|rear|渝D7K418'"
cap "  ↑ 有 /plate/set 回音说明材质已换到车上（换牌是 gzserver 进程内 SetMaterial）"
sleep 10
cap "== 1b 等 car_1 跑到卡口取景（自然交通，最长约 3 分钟），期间右栏计数持续累加 =="
timeout -k 3 230 python3 -u "$TOOLS/verify_item1_custom_plate.py" 200 > /tmp/vid_g1.out 2>&1 &
VP=$!
sleep 60
cap "  等待中……画面里车辆经过龙门架的瞬间就是抓拍帧"
sleep 60
cap "  等待中……"
sleep 62
cap "== 1c 端到端结果（抓拍号 / 卡口 / 图路径 / 识别值 vs 真值 / 置信 / 耗时 / 时刻 / 位姿）=="
capq "wait $VP; grep -aA11 '端到端结果' /tmp/vid_g1.out"
capq "grep -aE '^算法 |^车辆位置 |^外接框 ' /tmp/vid_g1.out"
vid_wait
