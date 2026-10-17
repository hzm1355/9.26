#!/bin/bash
# 把任务三各次取证的原始终端输出归拢成一个文件，报告里引用它而不是我手抄的数字。
# 只读现成的日志/CSV，不启动任何东西；跑完测试后执行一次即可。
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
E=$SIM/evidence
mkdir -p $E
OUT=$E/任务三_取证原始输出.txt
: > "$OUT"

# emit <标题> <候选路径...>：第一个非空的文件算数。
# 为什么要多个候选：取证输出散在 /tmp 和 evidence/ 两处——长跑脚本当时是重定向到
# evidence/ 的（lighting_round.sh），早先几轮还在 /tmp/，只写死一条路径就会漏。
emit() {
  local title="$1"; shift
  printf '\n\n########## %s ##########\n' "$title" >> "$OUT"
  for f in "$@"; do
    if [ -s "$f" ]; then tr -d '\r' < "$f" >> "$OUT"; return; fi
  done
  echo "(缺失: $*)" >> "$OUT"
}

# 速度表由 CSV 现算，保证报告里的两张表和归拢的原始输出同源
for r in roundB before_fix roundA; do
  [ -s "$E/speed_sweep_$r.csv" ] || continue
  python3 $SIM/tools/sweep_table.py "$E/speed_sweep_$r.csv" > "$E/speed_sweep_${r}_table.txt" 2>&1
done

emit '卡口相机出帧节奏（渲染滞后实测）' /tmp/cam_rate.txt
emit '光照逐卡口取证（牌面自发光 lighting off）' "$E/lighting_lit_off.txt" /tmp/light_both.txt
emit '牌面受光对照（lit:=on，五档含夜间补光开关）' "$E/lighting_lit_on.txt" /tmp/lit_control.txt
emit '速度扫参 修复后 Round B 明细表' "$E/speed_sweep_roundB_table.txt"
emit '速度扫参 修复前对照表' "$E/speed_sweep_before_fix_table.txt" /tmp/speed_sweep_before_fix.log
emit '运动模糊容忍上限（离线换算）' /tmp/blur.txt
emit '黑名单 + 导出（自然交通，只收到 WARN）' /tmp/blacklist_export.txt
emit '黑名单定向取证（CRITICAL/WARN 两档都落在当前时线）' /tmp/bl_targeted.txt
emit '导出记录按 CSV 语义核对（告警行 + 车辆×卡口命中）' /tmp/check_export.txt

# 五条目逐条在线复验（20:00 之后这一轮）：每条一份原始终端输出，
# 报告里"条目 N 怎么验的"直接指到这里，不用手抄数字。
emit '条目1 自定义虚拟车牌端到端（新号码被拍到并逐字读回）' /tmp/item1_custom.txt
emit '条目2 外部算法桩节点替换内置识别器（三要素 + 下游原样记录）' /tmp/external_algo.txt
emit '条目3 高速档复验（配置 6 m/s 过卡）' /tmp/item3_speed.txt
emit '条目3 光照逐卡口复验（逆光/弱光/夜间）' /tmp/item3_light.txt
emit '条目3 逆光补样（两卡口各一张）' /tmp/item3_backlit.txt
emit '条目4 导出记录 CSV 语义核对（复验轮）' /tmp/check_export_2010.txt
emit '条目5 黑名单告警复验（当前时线 CRITICAL + WARN）' /tmp/item5_blacklist.txt
echo '条目4 界面实时展示截图: evidence/item4_dashboard.png（:8090 看板卡口抓拍面板）' >> "$OUT"

# 导出记录本身也归拢一份：报告 §六/§七 引用的每条告警都要能在 CSV 里查到同一行。
# 只带 CSV；JSONL 是同一批记录的逐行原文，留在 /tmp 按路径引用即可。
for f in plate_records.csv; do
  [ -s "/tmp/v2x_records/$f" ] && cp "/tmp/v2x_records/$f" "$E/$f"
done


# 不覆盖 speed_sweep_*.csv：那几份是按轮次手工定名备份的，
# /tmp/v2x_records/speed_sweep.csv 是"最近一轮"的可变指针，cp 过来会把 Round B 冲掉。

echo "写出 $OUT（$(wc -c < "$OUT") 字节）"
grep -c '^##########' "$OUT" | sed 's/^/段落数: /'
