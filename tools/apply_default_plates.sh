#!/bin/bash
# 把默认车牌重新下发到车上，并确认 plate_gen 真的应用了。
#
# 为什么需要一个专门的脚本：/plate/request 是一次性指令，plate_gen 一重启车上就退回
# 占位牌纹理，而卡口那边一切照常——抓拍照出图、识别照返回，只是全是空号。
# 没有这一步的"重启后直接开测"，跑十五分钟得到的是整张废表。
#
# 停车场那几辆车（car_6 以及现在上场的 car_3 / car_2）都要贴两块：入口卡口拍的是
# 车头牌（parking_zones.yaml 的 gate_in.camera.focus 落在停位车头），出口卡口拍的是
# 车尾牌，少一块就是那道闸永远识别不到车牌，只能走 unknown_as_temp 的临时车分支，
# 白名单校验那条要求演不出来。
#
# 用法：bash tools/apply_default_plates.sh    返回 0=全部到位，1=不完整
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

before=$([ -f /tmp/plate_gen.log ] && wc -l < /tmp/plate_gen.log || echo 0)
TARGETS=("car_1|rear|沪A12345" "car_3|rear|京B88990" "car_2|rear|粤Z1A2B3"
         "car_4|rear|苏E66F88" "car_5|rear|浙A66666"
         "car_6|front|沪A12345" "car_6|rear|沪A12345"
         "car_3|front|京B88990" "car_2|front|粤Z1A2B3")
total=${#TARGETS[@]}

# /plate/request 是一次性指令，而 `ros2 topic pub --once` 只在"发现订阅者"之后发一条：
# 实测五连发里会漏掉其中一辆（本次漏 car_4），漏的那辆车上还是占位牌，卡口拍到就是空号。
# 所以每轮都按 plate_gen 的实际回执核对一遍，缺谁补发谁（最多三轮）。
new=''
for round in 1 2 3; do
  for p in "${TARGETS[@]}"; do
    IFS='|' read -r car slot num <<< "$p"
    if printf '%s\n' "$new" | grep -q "plate_gen\] $car $slot ->"; then
      continue
    fi
    ros2 topic pub --once /plate/request std_msgs/msg/String "{data: '$p'}" \
      >/dev/null 2>&1 || true
  done
  sleep 4
  new=$(tail -n +$((before + 1)) /tmp/plate_gen.log 2>/dev/null || true)
  # 按目标点名数，不数日志行数：补发那一轮可能让同一副牌在日志里出现两次，
  # 行数够数却藏着另一副根本没下发 —— 那正好是"7/7 但其实缺 car_6 front"的形态。
  applied=0
  for p in "${TARGETS[@]}"; do
    IFS='|' read -r car slot num <<< "$p"
    if printf '%s\n' "$new" | grep -q "plate_gen\] $car $slot ->"; then
      applied=$((applied + 1))
    fi
  done
  echo "  第 $round 轮：已应用 $applied/$total"
  [ "$applied" -ge "$total" ] && break
done

echo "plate_gen 已应用 $applied/$total 副：$(printf '%s\n' "$new" | grep -oE 'car_[0-9] (front|rear) -> p[0-9]*' | tr '\n' ' ')"
[ "$applied" -ge "$total" ] || { echo '[apply_plates] 贴牌不完整（plate_gen 没收到指令 / v2x_plate 插件没换材质）'; exit 1; }
# 牌在不在车上只是一半，识别器活着才有统计表——它崩掉时长跑脚本会静默收 0 行
bash "$(dirname "$0")/ensure_recognizer.sh"
