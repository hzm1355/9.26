#!/bin/bash
# 点验 1016 轮那份逐拍轨迹样本的格式与覆盖范围（只读）。
# 目的：#104 的刹车行程门在合成滞后模型上复现不出场内的 ±0.37 rad 翻转，
# 要拿真机录下来的逐拍流回放两种转向律，得先确认这份流里有哪些列、覆盖哪段时间。
set -e
sed -i 's/\r$//' "$0"
for f in /tmp/pk190_trace.txt /tmp/pk190_report.txt /tmp/pk166_status308.log; do
  echo "=== $f"
  if [ -f "$f" ]; then
    wc -l "$f"
    head -3 "$f"
    echo "  ...尾 3 行..."
    tail -3 "$f"
  else
    echo "  不存在"
  fi
done
