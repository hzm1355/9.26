#!/bin/bash
# 报告终检：行数、遗留占位符、表格列数是否与同表头不一致。
F=/mnt/d/desktop1/9.26.1/任务三_卡口车牌识别仿真_验证报告.md
wc -l "$F"
echo '--- 遗留占位/待办:'
grep -n 'TODO\|待补\|<!--' "$F" || echo '  none'
echo '--- 每张表的列数（按 | 分组，表头行与后续行不一致就会打印出来）:'
# 只报"表体行列数和表头不一致"的行：这种行在 markdown 里会把表格截断
awk '
  { line = $0; gsub(/\\\|/, "@", line) }
  line ~ /^\|/ { n = split(line, a, "|") - 2
         if (prev == 0) { prev = n; hdr = NR }
         else if (n != prev) printf "  行 %-4s 列数 %d，表头(行 %d) %d  %s\n", NR, n, hdr, prev, substr($0, 1, 70)
         next }
  { prev = 0 }
' "$F"
