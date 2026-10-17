#!/bin/bash
# 诊断"链式闸没接着起条目 5"：三件必须同时看 —— 现在几点、链还活着没、
# 条目 4 驱动到底打印了什么收尾标记。只看日志尾会漏掉"标记字面不匹配"这一类。
set +e
echo "--- 现在时刻 ---"
date '+%F %T'
echo "--- 相关文件 mtime ---"
stat -c '%y  %n' /tmp/pk4_driver.log /tmp/pk_chain.log /tmp/vid_caption.txt 2>&1 | cut -c1-110
echo "--- 进程 ---"
printf '  pk_chain4to5  %s\n' "$(pgrep -cf 'pk_chain4to[5].sh')"
printf '  vid_parking_4 %s\n' "$(pgrep -cf 'vid_parking_[4].sh')"
printf '  vid_parking_5 %s\n' "$(pgrep -cf 'vid_parking_[5].sh')"
printf '  vid_rec       %s\n' "$(pgrep -cf 'vid_rec[.]py')"
printf '  shuttle       %s\n' "$(pgrep -cf 'lib/v2x[_]demo/parking_shuttle_node')"
echo "--- 条目 4 驱动日志全文 ---"
cut -c1-200 /tmp/pk4_driver.log
echo "--- vid_parking_4.sh 的收尾 12 行（看它到底 print 什么标记）---"
tail -12 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_parking_4.sh
echo "--- vid_wait 的实现（它等到什么算完）---"
grep -n -A14 '^vid_wait()' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
