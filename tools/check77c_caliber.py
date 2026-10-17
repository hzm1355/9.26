#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#77 改后窗口的口径核对，三件事：

1) 复位到底触发过没有 —— trace_reset_scan 报"0 次瞬移"有两种解释：要么复位没发生，
   要么发生了但不弹。必须回驱动日志和轨迹 CSV 的 phase 列区分这两种。
2) 轨迹 CSV 的分车实测速度峰值（改前 car_2 峰值 8.53 m/s）。
3) 本轮新增超速告警的速度分布 —— |v|>1.0 的样本是不是又被测速模块当成告警。
"""
import csv
import json
import re
from collections import Counter, defaultdict

LOG = '/tmp/signal_drive_fix.log'
PATH_CSV = '/mnt/d/desktop1/9.26.1/_path_77c.csv'
ALERTS = '/tmp/v2x_records/speed_alerts.jsonl'

print('== 1) 驱动日志中的复位/脱困/出界行（尾部 25 条原始文本）')
try:
    with open(LOG, encoding='utf-8', errors='replace') as f:
        lines = f.read().splitlines()
except OSError as e:
    lines = []
    print('  读不到', LOG, e)
hits = [(i + 1, s) for i, s in enumerate(lines)
        if re.search(r'复位|刹轮|脱困|卡死|出界|重取航段', s)]
print(f'  匹配 {len(hits)} 行，请求复位 {sum(1 for _, s in hits if "请求复位" in s)} 条，'
      f'实际落位 {sum(1 for _, s in hits if "复位到" in s)} 条')
for i, s in hits[-25:]:
    print(f'  L{i}: {s[-130:]}')

print('\n== 2) 轨迹 CSV phase 列分布 + 分车实测速度峰值')
phase = Counter()
sp = defaultdict(list)
with open(PATH_CSV, encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        phase[(r['car'], r['phase'])] += 1
        try:
            sp[r['car']].append(abs(float(r['speed'])))
        except (TypeError, ValueError):
            pass
print('  phase 取值及计数：', {k[1] or '(空)': v for k, v in phase.items()})
print('  分车 |speed| 峰值：', {k: round(max(v), 2) for k, v in sorted(sp.items())})
print('  分车 |speed|>0.90 的格数：', {k: sum(1 for x in v if x > 0.90) for k, v in sorted(sp.items())})
print('  分车总行数：', {k: len(v) for k, v in sorted(sp.items())})

print('\n== 3) 超速告警（尾部 26 条）')
try:
    with open(ALERTS, encoding='utf-8') as f:
        rows = [json.loads(s) for s in f if s.strip()]
except OSError as e:
    rows = []
    print('  读不到', ALERTS, e)
print('  文件总条数：', len(rows))
if rows:
    print('  字段名（原样，不猜）：', sorted(rows[-1]))
    for r in rows[-26:]:
        print('  ', json.dumps(r, ensure_ascii=False)[:220])
