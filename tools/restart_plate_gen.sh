#!/bin/bash
# 唯一地重启车牌生成器：先把**所有** plate_gen 实例杀干净，再只起一个，
# 然后下发五副牌并验证落盘的材质真的带着要求的 lighting 标志。
#
# 为什么要专门一个脚本：lit_control.sh / restore_default.sh 都是
# `pgrep -f plate_gen_node.py | head -1` 只杀一个实例，然后 setsid 再起一个。
# 于是每来回切一次就多留一个僵尸实例，实测积累了 6 个（4 个 lit:=off + 2 个 lit:=on）。
# 它们都订阅 /plate/request，谁最后回盘谁决定车上是哪张材质——
# 结果"受光/自发光"两轮的标签其实是随机的：受光轮的牌面恒定在贴图原色，
# 自发光轮的牌面反倒跟着场景从 121 掉到 31。
# 一道对照实验的全部结论毁在一个 head -1 上。
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
MODE=${1:-off}
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

# 杀干净：循环到两类模式都查不到为止。不能只杀一个，也不能 pkill -f plate_gen
# （会匹配到本脚本自己的 argv）。
for i in 1 2 3 4 5 6 7 8 9 10; do
  pids=$(pgrep -f 'lib/v2x_demo/plate_gen_node' || true)
  pids="$pids $(pgrep -f 'run v2x_demo plate_gen_node' || true)"
  pids=$(echo $pids | tr ' ' '\n' | sort -u | grep -v '^$' || true)
  [ -z "$pids" ] && break
  echo "残留 plate_gen: $(echo $pids | tr '\n' ' ')"
  kill $pids 2>/dev/null || true
  sleep 1
  kill -9 $pids 2>/dev/null || true   # rclpy 把 TERM 只当 shutdown 标志
done
alive=$(pgrep -f 'plate_gen_node' -c || true)
[ "${alive:-0}" -eq 0 ] || { echo "[plate_gen] 还有 $alive 个实例没退，中止"; exit 1; }

setsid nohup ros2 run v2x_demo plate_gen_node.py --ros-args -p lit:=$MODE \
  > /tmp/plate_gen.log 2>&1 < /dev/null &
sleep 5
n=$(pgrep -f 'lib/v2x_demo/plate_gen_node' -c || true)
flag=$(grep -o '牌面材质 lighting [a-z]*' /tmp/plate_gen.log | head -1)
echo "实例数 $n  $flag"
[ "${n:-0}" -eq 1 ] || { echo "[plate_gen] 实例数不是 1，中止"; exit 1; }

# 材质是 SetPlate 处理时才落盘的，不下发就还是旧文件
for p in "car_1|rear|沪A12345" "car_3|rear|京B88990" "car_2|rear|粤Z1A2B3" \
         "car_4|rear|苏E66F88" "car_5|rear|浙A66666"; do
  ros2 topic pub --once /plate/request std_msgs/msg/String "{data: '$p'}" >/dev/null 2>&1
  sleep 0.6
done
sleep 3
bad=$(ls -t /tmp/v2x_plates/*/*.material | head -5 | xargs grep -L "lighting $MODE" || true)
if [ -n "$bad" ]; then
  echo "[plate_gen] 最新 5 张材质里有不带 lighting $MODE 的："; echo "$bad"; exit 1
fi
echo "[plate_gen] 最新 5 张材质全是 lighting $MODE"
