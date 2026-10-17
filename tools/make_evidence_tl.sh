#!/bin/bash
# 把任务二（信号灯配时仿真）各次取证的原始终端输出归拢成一个文件，
# 报告里引用它而不是手抄数字。只读现成日志，不启动任何东西。
# 用法：bash tools/make_evidence_tl.sh
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
E=$SIM/evidence
mkdir -p $E
OUT=$E/任务二_取证原始输出.txt
: > "$OUT"

# emit <标题> <候选路径...>：第一个非空的文件算数（同一取证在不同轮次落在 /tmp 或 evidence/）。
emit() {
  local title="$1"; shift
  printf '\n\n########## %s ##########\n' "$title" >> "$OUT"
  for f in "$@"; do
    if [ -s "$f" ]; then tr -d '\r' < "$f" >> "$OUT"; return; fi
  done
  echo "(缺失: $*)" >> "$OUT"
}

emit '条目1 灯态跳变时间轴（仿真时钟，机动车+行人）' /tmp/tl_timeline.txt /tmp/t2_verify.txt "$E/evidence_tl_timeline.txt"
emit '条目1 时序合规量化判据（相序/黄灯时长/人车冲突）' /tmp/t2_phase_audit.txt "$E/tl_phase_audit.txt"
emit '条目2+3a 手动配时与预设（关自适应隔离环境）' /tmp/t2_presets.txt "$E/evidence_presets.txt"
emit '条目2c 联动-运行中改 offset' /tmp/t2_2c.txt
emit '条目2c 联动-播种态（配置冷启动，语义精确）' /tmp/t2_2c_seeded.txt
emit '条目3a 四套预设逐套读回（可靠下发）' /tmp/t2_3a.txt
emit '条目3b 时段自动切换日志' /tmp/signal.log
emit '条目3c+4 自适应系数/事件触发/统计输出' /tmp/t2_3c4.txt
emit '条目1~4 一把跑完（verify_task2.sh）' /tmp/t2_verify.txt "$E/evidence_verify_task2.txt"
emit '条目5 现录300s -> 回放 -> 逐条对照' /tmp/t2_replay_fresh.txt
emit '条目5 归档录像回放对照' /tmp/replay_compare.txt "$E/evidence_replay.txt" /mnt/d/desktop1/9.26.1/evidence_replay.txt

# 录像本体：报告 §条目5 的每条数字都要能在这份 JSON 里查到
for f in /tmp/tl_log_fresh.json /tmp/tl_log.json; do
  if [ -s "$f" ]; then cp -f "$f" "$E/tl_log_fresh.json"; break; fi
done
[ -s /tmp/replay_observed.json ] && cp -f /tmp/replay_observed.json "$E/replay_observed.json"

echo "写出 $OUT（$(wc -c < "$OUT") 字节）"
grep -c '^##########' "$OUT" | sed 's/^/段落数: /'
