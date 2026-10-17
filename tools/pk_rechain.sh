#!/bin/bash
# 把空转的旧链式闸换成"跳过等待"版：先杀干净并确认 0 实例，再起新的。
# 顺序不能反 —— 旧闸还活着时起新闸，两条都会在体检通过后各起一次条目 5，
# 两个 vid_rec 同时写同一个 mp4、并互相把 /tmp/vid_caption.txt 清空（字幕层证据就没了）。
set +e
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools
for P in $(pgrep -f 'pk_chain4to[5].sh'); do
  echo "杀掉空转的旧链实例 $P"
  kill -9 "$P" 2>/dev/null
done
sleep 2
N=$(pgrep -cf 'pk_chain4to[5].sh')
echo "旧链剩余实例：${N:-0}"
[ "${N:-0}" = "0" ] || { echo "!! 没杀干净，不启新链"; exit 1; }

# 也不能有第二个录制器在跑
M=$(pgrep -cf 'vid_rec[.]py')
echo "当前 vid_rec 实例：${M:-0}"
[ "${M:-0}" = "0" ] || { echo "!! 已有录制器在跑，不启新链（会变成双录）"; exit 1; }

sed -i 's/\r$//' "$TOOLS/pk_chain4to5.sh" "$TOOLS/vid_parking_4.sh"
bash -n "$TOOLS/pk_chain4to5.sh" || { echo '!! 链脚本语法挂'; exit 1; }
bash -n "$TOOLS/vid_parking_4.sh" || { echo '!! 条目4脚本语法挂'; exit 1; }

setsid nohup env SKIP_WAIT=1 bash "$TOOLS/pk_chain4to5.sh" > /tmp/pk_chain.log 2>&1 < /dev/null &
sleep 8
echo "--- 新链头 12 行 ---"
head -12 /tmp/pk_chain.log | cut -c1-190
echo "--- 新链实例 ---"
printf '  pk_chain4to5 %s\n' "$(pgrep -cf 'pk_chain4to[5].sh')"
