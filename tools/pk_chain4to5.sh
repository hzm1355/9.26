#!/bin/bash
# 条目 4 → 条目 5 的链式启动器。
# 为什么不是"到点就直接起 5"：条目 4 的 3a/3d 会真改 parking_zones.yaml（黑名单 +
# unknown_as_temp）并重启 manager，万一它中途挂在中止分支上，yaml 就是"改到一半"的状态，
# 而条目 5 的 census / 过滤矩阵全部以基线名单为真值 —— 直接开录会把一段无效画面当成取证。
# 所以收尾后过四道闸，全过才起：
#   甲、配置体检器 CHECK PASS（回到基线，sed 锚点原样在）；
#   乙、三节点各 1 实例（manager 在 3d 被重启过，shuttle 在最后一行被重起，双实例会把
#       同一条 ParkingPass 落两行，条目 5 的"命中 N 条"直接翻倍）；
#   丙、成片体检（帧数与时长对得上）；
#   丁、200 s 内通行账在长（进程数=1、话题有发布者都不等于车真的过杆了）。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml
P4LOG=/tmp/pk4_driver.log
P4OUT="/mnt/d/desktop1/9.26.1/视频取证/任务五_停车场闸机/条目4_管控逻辑_三类放行拒绝与手动故障.mp4"
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[链] $(date '+%T') 开始等条目 4 收尾（上限 4200 s）"
# SKIP_WAIT=1：条目 4 已经录完、只是本脚本的等待标记没对上时用，直接从四道闸开始。
# 上一版就是栽在这里 —— vid_parking_4.sh 收尾没有 `echo DONE`（条目 5 那份有），
# grep 永远匹配不上，于是空转到超时上限；而超时分支是"照样往下体检并起 5"，
# 如果这段时间里已经手工起了条目 5，就会出现两个 vid_rec 同时写盘、字幕流互相清空。
if [ -z "${SKIP_WAIT:-}" ]; then
  for i in $(seq 1 350); do
    grep -q DONE "$P4LOG" 2>/dev/null && break
    sleep 12
  done
  grep -q DONE "$P4LOG" 2>/dev/null || echo "[链] !! 等满 4200 s 仍未见 DONE，仍按现状继续体检"
else
  echo "[链] SKIP_WAIT=1，跳过等待直接体检"
fi
echo "[链] $(date '+%T') 条目 4 驱动结束"

echo "=== 甲 配置体检 ==="
# 跑一次存文件再判：这条 grep 如果内嵌再跑一遍体检器，两次读到的可能是 manager 正在
# 回灌 yaml 的中间态，出现"第一次 PASS 第二次 FAIL"的假红。
timeout -k 5 40 python3 $TOOLS/pk_yaml_check165.py $YAML > /tmp/pk_chain_yaml.txt 2>&1
tail -3 /tmp/pk_chain_yaml.txt
grep -q 'CHECK PASS' /tmp/pk_chain_yaml.txt || {
  echo "[链] !! 甲 挂：配置没回到基线，条目 5 不开录（先修 yaml）"; exit 1; }

echo "=== 乙 三节点实例数 ==="
for N in parking_manager_node parking_recorder_node parking_shuttle_node; do
  printf '  %-24s 实例 %s\n' "$N" "$(pgrep -cf lib/v2x[_]demo/$N)"
done
YI=$(pgrep -cf 'lib/v2x[_]demo/parking_manager_node')
YR=$(pgrep -cf 'lib/v2x[_]demo/parking_recorder_node')
YS=$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')
[ "$YI" = 1 ] && [ "$YR" = 1 ] && [ "$YS" = 1 ] || {
  echo "[链] !! 乙 挂：实例数不是 1/1/1，条目 5 不开录"; exit 1; }

echo "=== 丙 条目 4 成片体检 ==="
ls -l --time-style=+%H:%M:%S "$P4OUT" 2>&1 | tail -1 | cut -c1-120
python3 $TOOLS/vid_check.py "$P4OUT" 2>&1 | tail -6
echo "  字幕段头（最后 10 条）"
grep -a '== ' /tmp/vid_caption.txt | tail -10 | cut -c1-160

echo "=== 丁 200 s 落盘增长 ==="
P1=$(wc -l < /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null)
echo "  当前 $P1 行，等 200 s"
sleep 200
P2=$(wc -l < /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null)
echo "  之后 $P2 行（增量 $(( ${P2:-0} - ${P1:-0} ))）"
[ $(( ${P2:-0} - ${P1:-0} )) -gt 0 ] || {
  echo "[链] !! 丁 挂：200 s 内没有新通行，条目 5 不开录（车没过杆，录了就是空转）"; exit 1; }

echo "[链] 四道闸全过，$(date '+%T') 启动条目 5（DUR 默认 1300 s）"
setsid nohup bash $TOOLS/vid_parking_5.sh > /tmp/pk5_driver.log 2>&1 < /dev/null &
sleep 20
echo "[链] 条目 5 驱动头 12 行："
head -12 /tmp/pk5_driver.log 2>&1 | cut -c1-170
echo "[链] vid_rec 实例 $(pgrep -cf 'vid_rec[.]py')（应为 1）"
echo CHAIN_DONE
