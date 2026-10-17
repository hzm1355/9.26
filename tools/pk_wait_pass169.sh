#!/bin/bash
# 等"下一条真实通行的方向 == 指定方向"，而不是只等"行数在长"。
#
# 为什么条目 4 必须按方向等：car_6 一轮是 进场→出场→复位，两个闸各判一次。
# 换牌/摘名单这类"改身份"的动作如果落在**进场之后**，车下一个遇到的闸是出口闸，
# 而出口没有"未登记按临时车"这条分支（parking_manager.on_result 里 unknown_as_temp
# 只在 direction=='in' 生效），于是出口直接判拒 → 车等 30 s 退回 → 连废三轮触发
# #83 停驶挂起，后面几段全没有驱动。落在**出场之后**才轮到"进场→临时车发卡放行"。
#
# 用法：bash pk_wait_pass169.sh <in|out> <超时秒>
set +e
PKJ=/tmp/v2x_parking/parking_pass.jsonl
DIR=${1:-in}
LIM=${2:-300}
if [ "$DIR" = "out" ]; then NAME=出场; else NAME=进场; fi

L0=$(tail -1 $PKJ 2>/dev/null)
S0=$(printf '%s' "$L0" | grep -oE '"seq": [0-9]+' | head -1 | grep -oE '[0-9]+')
D0=$(printf '%s' "$L0" | grep -oE '进场|出场' | head -1)
echo "  起点：最后一条 seq=${S0:-?} 方向=${D0:-?}，等下一条【$NAME】"
t=0
HIT=''
while [ "$t" -lt "$LIM" ]; do
  sleep 10
  t=$((t + 10))
  L=$(tail -1 $PKJ 2>/dev/null)
  S=$(printf '%s' "$L" | grep -oE '"seq": [0-9]+' | head -1 | grep -oE '[0-9]+')
  D=$(printf '%s' "$L" | grep -oE '进场|出场' | head -1)
  printf '  %ss：最新 seq=%s 方向=%s\n' "$t" "${S:-?}" "${D:-?}"
  if [ -n "$S" ] && [ "$S" != "$S0" ] && [ "$D" = "$NAME" ]; then
    HIT=$L
    break
  fi
done
if [ -z "$HIT" ]; then
  echo "  !! 等了 ${t}s 没等到【$NAME】，最后一条仍是 seq=${S:-?}（方向 ${D:-?}）"
  exit 1
fi
echo "  ✓ 等了 ${t}s 等到【$NAME】，原文（截 260 字符）："
printf '  %s\n' "$HIT" | cut -c1-260
exit 0
