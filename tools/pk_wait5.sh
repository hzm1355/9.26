#!/bin/bash
# 等"链式闸 + 条目 5"整段收尾，然后把两段成片的体检一次吐回来。
# 为什么要区分两种结束：链脚本有四道闸（配置体检/实例数/成片体检/落盘增长），任一挂掉
# 它就是 exit 1 而不会起条目 5 —— 那时 /tmp/pk5_driver.log 永远不出现，只看 DONE 会傻等到
# 超时上限，把"闸拦住了"读成"还在录"。
set +e
CHAIN=/tmp/pk_chain.log
P5=/tmp/pk5_driver.log
DIR="/mnt/d/desktop1/9.26.1/视频取证/任务五_停车场闸机"
for i in $(seq 1 400); do
  grep -q CHAIN_DONE "$CHAIN" 2>/dev/null && break
  grep -q '\[链\] !!' "$CHAIN" 2>/dev/null && break
  sleep 15
done
echo "--- 链日志全文 ---"
cut -c1-190 "$CHAIN" 2>/dev/null
if grep -q '\[链\] !!' "$CHAIN" 2>/dev/null; then
  echo "!!! 链被闸拦下，条目 5 未开录（上面点明是哪一道）"
  exit 1
fi
echo "--- 等条目 5 驱动 DONE（上限 3000 s）---"
for i in $(seq 1 250); do
  grep -q DONE "$P5" 2>/dev/null && break
  sleep 12
done
tail -20 "$P5" 2>/dev/null | cut -c1-190
echo "--- 两段成片体检 ---"
python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_check.py \
  "$DIR/条目4_管控逻辑_三类放行拒绝与手动故障.mp4" \
  "$DIR/条目5_记录管理_进出场车牌动作日志查询导出.mp4" 2>&1 | cut -c1-170
echo "--- 字幕流轮转件（每段一份，文本层证据）---"
ls -1 /tmp/vid_caption.txt.*.prev 2>/dev/null | tail -4
echo "--- 落盘账 ---"
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl
WAIT5_DONE
